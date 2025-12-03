#include <zlib.h>

#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/noise/frequency_filters.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <thread>

#include "dv-processing/core/frame/accumulator.hpp"

// #define OFFLINE_TESTING

// IMPORTANT FOR VISUALIZATION
// run `export QT_QPA_PLATFORM=xcb` before before starting the node

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

    // Initialize an accumulator with some resolution
    accumulator_ = dv::Accumulator(*camera_->getEventResolution());
    // show raw events
    accumulator_.setDecayFunction(dv::Accumulator::Decay::STEP);
    accumulator_.setMinPotential(0.f);
    accumulator_.setMaxPotential(1.f);

    // ini filter
    lp_filter_ = std::make_unique<dv::noise::LowPassFilter<dv::EventStore>>(
      *camera_->getEventResolution(), 500.0f);

    // fancy decay image reconstruct
    // accumulator_.setMinPotential(0.f);
    // accumulator_.setMaxPotential(1.f);
    // accumulator_.setNeutralPotential(0.5f);
    // accumulator_.setEventContribution(0.15f);
    // accumulator_.setDecayFunction(dv::Accumulator::Decay::EXPONENTIAL);
    // accumulator_.setDecayParam(1e+6);
    // accumulator_.setIgnorePolarity(false);
    // accumulator_.setSynchronousDecay(false);

    // slicer for vis. frames
    event_slicer_.doEveryTimeInterval(33ms, [&](const dv::EventStore & events) {
      lp_filter_->accept(events);
      const dv::EventStore filtered = lp_filter_->generateEvents();
      accumulator_.accept(filtered);
      dv::Frame frame = accumulator_.generateFrame();

      {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        latest_frame_ = frame;
      }
    });
#endif
  }

  bool getLatestFrame(dv::Frame & out)
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (!latest_frame_) return false;
    out = *latest_frame_;
    return true;
  }

private:
  // publisher implimentations
  void publishEvents(const dv::EventStore & store)
  {
    if (store.isEmpty()) {
      return;
    }

    dvs_msgs::msg::EventArray msg;
    msg.header.stamp = rclcpp::Time(store[0].timestamp() * 1000);
    msg.header.frame_id = "events";
    msg.events.reserve(store.size());
    msg.height = 480;
    msg.width = 640;

    for (const auto & e : store) {
      dvs_msgs::msg::Event ev;
      ev.x = e.x();
      ev.y = e.y();
      ev.polarity = e.polarity();
      ev.ts = rclcpp::Time(e.timestamp() * 1000);
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
    int64_t last_timestamp = 0;
    for (const auto & measure : imu_vec) {
      sensor_msgs::msg::Imu msg;
      int64_t msg_ts = measure.timestamp * 1000ULL;
      msg.header.stamp = rclcpp::Time(msg_ts);
      msg.header.frame_id = "imu";
      msg.angular_velocity.x = measure.gyroscopeX * DEG_TO_RADIANS;
      msg.angular_velocity.y = measure.gyroscopeY * DEG_TO_RADIANS;
      msg.angular_velocity.z = measure.gyroscopeZ * DEG_TO_RADIANS;
      msg.linear_acceleration.x = measure.accelerometerX * G;
      msg.linear_acceleration.y = measure.accelerometerY * G;
      msg.linear_acceleration.z = measure.accelerometerZ * G;
      msg.orientation_covariance[0] = -1.0;  // see ROS REP 145

      if (msg_ts > last_timestamp) {
        imu_pub_->publish(msg);
        last_timestamp = msg_ts;
      }
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
  std::unique_ptr<dv::noise::LowPassFilter<dv::EventStore>> lp_filter_;

  dv::Accumulator accumulator_;
  dv::Frame event_frame_;
  std::mutex frame_mutex_;
  std::optional<dv::Frame> latest_frame_;

  std::optional<dv::io::camera::DVXplorerM> camera_;
  rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr event_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr compressed_event_pub_;

  std::thread thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EventCameraNode>();

  // rosnode in an seperate thread, so vis. is main thread
  std::thread ros_spin([node]() { rclcpp::spin(node); });

  cv::namedWindow("Preview", cv::WINDOW_NORMAL);
  while (rclcpp::ok()) {
    dv::Frame frame;
    if (node->getLatestFrame(frame)) {
      cv::imshow("Preview", frame.image);
      cv::waitKey(2);
    }
    std::this_thread::sleep_for(33ms);
  }

  ros_spin.join();
  rclcpp::shutdown();
  return 0;
}
