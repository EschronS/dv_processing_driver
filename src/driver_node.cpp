#include <zlib.h>

#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <thread>

// #define OFFLINE_TESTING

using namespace std::chrono_literals;
constexpr auto SLICE_PERIOD = 5ms;
constexpr size_t MAX_EVENTS = 50000;

class EventCameraNode : public rclcpp::Node
{
public:
  EventCameraNode() : Node("event_camera_node")
  {
    // publishers
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    event_pub_ = create_publisher<dvs_msgs::msg::EventArray>("dvx/events", qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("dvx/imu", qos);
    compressed_event_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("dv/events_cmp", qos);

    // slicers
    event_slicer_.doEveryTimeInterval(SLICE_PERIOD, [&](const dv::EventStore & store) {
      if (store.size() > MAX_EVENTS) {
        RCLCPP_WARN(
          get_logger(), "Received very many events %d > %d", int(store.size()), int(MAX_EVENTS));
      }
      publishEvents(store);
    });

    imu_slicer_.doEveryNumberOfElements(1, [&](const dv::IMUPacket & imus) { publishImu(imus); });

#ifdef OFFLINE_TESTING
    startSimulation();
#else
    startCamera();
#endif
  }

private:
  // publisher implimentations
  void publishEvents(const dv::EventStore & store)
  {
    if (store.isEmpty()) {
      return;
    }

    dvs_msgs::msg::EventArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "events";
    msg.events.reserve(store.size());
    msg.height = 480;
    msg.width = 640;

    for (const auto & e : store) {
      dvs_msgs::msg::Event ev;
      ev.x = e.x();
      ev.y = e.y();
      ev.polarity = e.polarity();
      ev.ts = rclcpp::Time(e.timestamp());
      msg.events.push_back(ev);
    }

    // rclcpp::SerializedMessage serialized_msg;
    // rclcpp::Serialization<dvs_msgs::msg::EventArray> serializer;
    // serializer.serialize_message(&msg, &serialized_msg);
    //
    // const uint8_t * raw_data = serialized_msg.get_rcl_serialized_message().buffer;
    // size_t raw_size = serialized_msg.get_rcl_serialized_message().buffer_length;
    //
    // auto compressed = compressBuffer(raw_data, raw_size);
    //
    // if (!compressed.empty()) {
    //   std_msgs::msg::UInt8MultiArray compressed_msg;
    //   compressed_msg.data = std::move(compressed);
    //   compressed_event_pub_->publish(compressed_msg);
    // }

    event_pub_->publish(std::move(msg));
  }

  void publishImu(const dv::IMUPacket & packet)
  {
    constexpr double G = 9.81;
    constexpr double DEG_TO_RADIANS = M_PI / 180.0;

    std::vector<dv::IMU> imu_vec = packet.elements;
    for (const auto & measure : imu_vec) {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = rclcpp::Time(measure.timestamp * 1000);
      msg.header.frame_id = "imu";
      msg.angular_velocity.x = measure.gyroscopeX * DEG_TO_RADIANS;
      msg.angular_velocity.y = measure.gyroscopeY * DEG_TO_RADIANS;
      msg.angular_velocity.z = measure.gyroscopeZ * DEG_TO_RADIANS;
      msg.linear_acceleration.x = measure.accelerometerX * G;
      msg.linear_acceleration.y = measure.accelerometerY * G;
      msg.linear_acceleration.z = measure.accelerometerZ * G;
      msg.orientation_covariance[0] = -1.0;  // see ROS REP 145

      imu_pub_->publish(msg);
    }
  }

  // real camera case
  void startCamera()
  {
    camera_.emplace();

    thread_ = std::thread([&]() {
      while (rclcpp::ok() && camera_->isRunning()) {
        if (auto events = camera_->getNextEventBatch(); events.has_value()) {
          event_slicer_.accept(*events);
        }
        if (auto imus = camera_->getNextImuBatch(); imus.has_value()) {
          imu_slicer_.accept(*imus);
        }
      }
    });
    thread_.detach();
  }

  // offline signal simulation for testing
  void startSimulation()
  {
    thread_ = std::thread([&]() {
      for (int i = 0; i < 2000; i++) {
        auto imus = dv::data::generate::levelImuWithNoise(i * 1000, 0.1f, 0.01f, i);
        auto events = dv::data::generate::uniformEventsWithinTimeRange(
          i * 1000, 1ms, cv::Size(640, 480), 1000, i);

        imu_slicer_.accept(imus);
        event_slicer_.accept(events);

        std::this_thread::sleep_for(1ms);
      }
    });
    thread_.detach();
  }

  std::vector<uint8_t> compressBuffer(const uint8_t * data, size_t size)
  {
    uLongf compressedSize = compressBound(size);
    std::vector<uint8_t> output(compressedSize);

    if (compress2(output.data(), &compressedSize, data, size, Z_BEST_SPEED) != Z_OK) {
      RCLCPP_ERROR(this->get_logger(), "Compression failed!");
      return {};
    }

    output.resize(compressedSize);
    return output;
  }

  dv::EventStreamSlicer event_slicer_;
  dv::IMUStreamSlicer imu_slicer_;

  std::optional<dv::io::camera::DVXplorerM> camera_;
  rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr event_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr compressed_event_pub_;

  std::thread thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventCameraNode>());
  rclcpp::shutdown();
  return 0;
}
