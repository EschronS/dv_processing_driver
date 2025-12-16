#include "dv_processing_driver/driver_node.hpp"

#include <zlib.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <thread>

#include "dv_processing_driver/compressed_encoder.hpp"
#include "event_camera_msgs/msg/event_packet.hpp"

// #define OFFLINE_TESTING
// #define LIVE_VISUALIZATION

// IMPORTANT FOR VISUALIZATION
// run `export QT_QPA_PLATFORM=xcb` before before starting the node

using namespace std::chrono_literals;

EventCameraNode::EventCameraNode() : rclcpp::Node("event_camera_node")
{
  //params
  auto slice_ms = this->declare_parameter<int>("slice_period_ms", 5);
  slice_period_ = std::chrono::milliseconds(slice_ms);
  max_events_ = this->declare_parameter<int>("max_events", 50000);
  every_nth_px_ = this->declare_parameter<int>("every_nth_px", 2);

  // publishers
  rclcpp::QoS qos_event(rclcpp::KeepLast(1));
  qos_event.best_effort();  // for imu not us best effort
  rclcpp::QoS qos_imu{rclcpp::KeepAll()};
  qos_imu.reliable();

  event_pub_ = create_publisher<dvs_msgs::msg::EventArray>("event_camera/events_old", qos_event);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("event_camera/imu", qos_imu);
  last_timestamp = 0;
  static_assert(std::endian::native == std::endian::little);
  rosBaseTime_ =
    rclcpp::Time((this->get_clock()->now().nanoseconds() / 1000ULL) * 1000ULL, RCL_SYSTEM_TIME);

  compressed_event_pub_ =
    create_publisher<event_camera_msgs::msg::EventPacket>("event_camera/events", qos_event);

  // slicers
  event_slicer_.doEveryTimeInterval(slice_period_, [&](const dv::EventStore & store) {
    if (store.size() > max_events_) {
      RCLCPP_WARN(
        get_logger(), "Received very many events %d > %d", int(store.size()), int(max_events_));
    }
    // publishEvents(store);  // check real runtime of callback
    publishCompEvents(store);
  });

  imu_slicer_.doEveryNumberOfElements(1, [&](const dv::IMUPacket & imus) { publishImu(imus); });

#ifdef OFFLINE_TESTING
  startSimulation();
#else
  // ini camera
  startCamera();

  if (!camera_) {
    RCLCPP_FATAL(get_logger(), "Camera not initialized!");
    throw std::runtime_error("Camera not initialized");
  }
  // ini filters
  lp_filter_ = std::make_unique<dv::noise::LowPassFilter<dv::EventStore> >(
    *camera_->getEventResolution(), 50.0f);
  createMask(every_nth_px_, *camera_->getEventResolution());
  em_filter_ = std::make_unique<dv::EventMaskFilter<dv::EventStore> >(mask_);

#ifdef LIVE_VISUALIZATION
  // Initialize an accumulator with some resolution
  accumulator_ = dv::Accumulator(*camera_->getEventResolution());
  // show raw events
  accumulator_.setDecayFunction(dv::Accumulator::Decay::STEP);
  accumulator_.setMinPotential(0.f);
  accumulator_.setMaxPotential(1.f);
  accumulator_.setEventContribution(1.5f);

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
  event_slicer_.doEveryTimeInterval(66ms, [&](const dv::EventStore & events) {
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
#endif
}

bool EventCameraNode::getLatestFrame(dv::Frame & out)
{
  std::lock_guard<std::mutex> lk(frame_mutex_);
  if (!latest_frame_) return false;
  out = *latest_frame_;
  return true;
}

// publisher implimentations
void EventCameraNode::publishEvents(const dv::EventStore & store)
{
  if (store.isEmpty()) {
    return;
  }

  dvs_msgs::msg::EventArray msg;
  msg.header.stamp = rclcpp::Time(store[0].timestamp() * 1000ULL);
  msg.header.frame_id = "events";
  msg.events.reserve(store.size());
  msg.height = 480 / dimension_divisor_;
  msg.width = 640 / dimension_divisor_;

  for (const auto & e : store) {
    dvs_msgs::msg::Event ev;
    ev.x = e.x() / dimension_divisor_;
    ev.y = e.y() / dimension_divisor_;
    ev.polarity = e.polarity();
    ev.ts = rclcpp::Time(e.timestamp() * 1000ULL);
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

void EventCameraNode::publishCompEvents(const dv::EventStore & store)
{
  uint64_t t =
    duration_cast<rclcpp::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  if (compressed_event_pub_->get_subscription_count() > 0) {
    if (!eventMsg_) {
      eventMsg_.reset(new event_camera_msgs::msg::EventPacket());
      eventMsg_->header.frame_id = "events";
      eventMsg_->header.stamp = rclcpp::Time(t, RCL_SYSTEM_TIME);
      eventMsg_->encoding = "libcaer_cmp";
      eventMsg_->seq = seq_++;
      eventMsg_->width = 640ULL;
      eventMsg_->height = 480ULL;
      eventMsg_->is_bigendian = false;
      eventMsg_->events.reserve(store.size());  // changed from libcaer logic
      // should preallocate space for all current events
    }
    static_assert(std::is_same_v<decltype(sensorTime_0), uint64_t>);
    event_packed_compression::convert_polarity_events_compressed(
      eventMsg_.get(), store, rosBaseTime_, &sensorTime_0);
    // const auto & events = eventMsg_->events;

    compressed_event_pub_->publish(std::move(eventMsg_));  // will reset the pointer
    // lastMessageTime_ = t;
  } else {
    if (eventMsg_) {
      eventMsg_.reset();
    }
  }
}

void EventCameraNode::publishImu(const dv::IMUPacket & packet)
{
  constexpr double G = 9.81;
  constexpr double DEG_TO_RADIANS = M_PI / 180.0;

  std::vector<dv::IMU> imu_vec = packet.elements;
  for (const auto & measure : imu_vec) {
    // NOTE: imu timestamp will overflow after circa 31 minutes, need to handle this explicitly
    int64_t msg_ts = static_cast<int64_t>(measure.timestamp) * 1000ULL;
    // slicer inludes identical measurements in multiple slices, drop those
    if (msg_ts <= last_timestamp) continue;
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(msg_ts);
    msg.header.frame_id = "imu";
    msg.angular_velocity.x = measure.gyroscopeX * DEG_TO_RADIANS;
    msg.angular_velocity.y = measure.gyroscopeY * DEG_TO_RADIANS;
    msg.angular_velocity.z = measure.gyroscopeZ * DEG_TO_RADIANS;
    msg.linear_acceleration.x = measure.accelerometerX * G;
    msg.linear_acceleration.y = measure.accelerometerY * G;
    msg.linear_acceleration.z = measure.accelerometerZ * G;
    msg.orientation_covariance[0] = -1.0;  // see ROS REP 145

    last_timestamp = msg_ts;
    imu_pub_->publish(msg);
  }
}

// real camera case
void EventCameraNode::startCamera()
{
  camera_.emplace();

  thread_ = std::thread([&]() {
    while (rclcpp::ok() && camera_->isRunning()) {
      if (auto events = camera_->getNextEventBatch(); events.has_value()) {
        lp_filter_->accept(*events);
        const dv::EventStore filtered = lp_filter_->generateEvents();
#ifdef LIVE_VISUALIZATION
        accumulator_.accept(filtered);
#endif
        event_slicer_.accept(filtered);
      }
      if (auto imus = camera_->getNextImuBatch(); imus.has_value()) {
        imu_slicer_.accept(*imus);
      }
    }
  });
  thread_.detach();
}

// offline signal simulation for testing
void EventCameraNode::startSimulation()
{
  thread_ = std::thread([&]() {
    for (int i = 0; i < 20000; i++) {
      auto imus = dv::data::generate::levelImuWithNoise(i * 1000, 0.1f, 0.01f, i);
      auto events = dv::data::generate::uniformEventsWithinTimeRange(
        i * 1000, 1ms, cv::Size(640, 480), 10000, i);

      imu_slicer_.accept(imus);
      event_slicer_.accept(events);

      std::this_thread::sleep_for(1ms);
    }
  });
  thread_.detach();
}

std::vector<uint8_t> EventCameraNode::compressBuffer(const uint8_t * data, size_t size)
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

void EventCameraNode::createMask(int every_nth_px, cv::Size size)
{
  RCLCPP_INFO(this->get_logger(), "Mask size requested: %d x %d", size.width, size.height);
  int rows = size.height;
  int cols = size.width;
  assert(rows % 2 == 0 && cols % 2 == 0);
  assert(rows > 0 && cols > 0);
  cv::Mat base;
  if (every_nth_px == 2) {
    base = (cv::Mat_<uchar>(2, 2) << 255, 0, 0, 255);
    dimension_divisor_ = 2;  // scale down the max cols and rows when publishing
  } else if (every_nth_px == 4) {
    base = (cv::Mat_<uchar>(2, 2) << 255, 0, 0, 0);
    dimension_divisor_ = 2;
  }
  if (every_nth_px != 1) {
    cv::repeat(base, rows / 2, cols / 2, mask_);  // write resulting mask to mask_ attribute
  } else {
    dimension_divisor_ = 1;
  }
  assert(dimension_divisor_ > 0);  // unknown every_nth_px value
}

// uint64_t event_slicer_callback_counter_;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EventCameraNode>();

  // rosnode in an seperate thread, so vis. is main thread
  std::thread ros_spin([node]() { rclcpp::spin(node); });

#ifdef LIVE_VISUALIZATION
  cv::namedWindow("Preview", cv::WINDOW_NORMAL);
  while (rclcpp::ok()) {
    dv::Frame frame;
    if (node->getLatestFrame(frame)) {
      cv::imshow("Preview", frame.image);
      cv::waitKey(2);
    }
    std::this_thread::sleep_for(33ms);
  }
#endif
  ros_spin.join();
  rclcpp::shutdown();
  return 0;
}
