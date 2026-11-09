#include "dv_processing_driver/driver_node.hpp"

#include <sys/types.h>
#include <zlib.h>

#include <chrono>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <stdexcept>
#include <thread>
#include <variant>

#include "dv-processing/core/core.hpp"
#include "dv-processing/core/filters.hpp"
#include "dv_processing_driver/compressed_encoder.hpp"
#include "event_camera_msgs/msg/event_packet.hpp"

// #define OFFLINE_TESTING
// #define LIVE_VISUALIZATION
// #define USE_PUB_THREAD

// IMPORTANT FOR VISUALIZATION
// run `export QT_QPA_PLATFORM=xcb` before before starting the node

using namespace std::chrono_literals;

EventCameraNode::EventCameraNode() : rclcpp::Node("event_camera_node")
{
  //params
  const auto publish_period_ms = this->declare_parameter<int>("publish_period_ms", 10);
  if (publish_period_ms <= 0) {
    throw std::invalid_argument("publish_period_ms must be greater than zero");
  }
  publish_period_ = std::chrono::milliseconds(publish_period_ms);
  max_events_ = this->declare_parameter<int>("max_events", 3000);
  every_nth_px_ = this->declare_parameter<int>("every_nth_px", 4);
  if (every_nth_px_ != 1 && every_nth_px_ != 2 && every_nth_px_ != 4) {
    throw std::invalid_argument("every_nth_px must be one of 1, 2 or 4");
  }
  low_pass_freq_ = this->declare_parameter<float>("low_pass_freq", 40.0);
  encoding_ = this->declare_parameter<std::string>("encoding", "libcaer_cmp");
  if (encoding_ != "dvs" && encoding_ != "libcaer_cmp") {
    throw std::invalid_argument("encoding must be one of 'dvs' or 'libcaer_cmp'");
  }
  RCLCPP_INFO(
    get_logger(), "stored setup params (publish_period_ms=%ld, encoding=%s)",
    static_cast<long>(publish_period_ms), encoding_.c_str());

  // publishers
  rclcpp::QoS qos_event(rclcpp::KeepLast(1));
  qos_event.best_effort();  // for imu not us best effort
  rclcpp::QoS qos_imu{rclcpp::KeepAll()};
  qos_imu.reliable();
  RCLCPP_INFO(get_logger(), "defined QoS for publishers");

  event_pub_ = create_publisher<dvs_msgs::msg::EventArray>("event_camera/events_old", qos_event);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("event_camera/imu", qos_imu);
  last_timestamp = 0;
  static_assert(std::endian::native == std::endian::little);

  compressed_event_pub_ =
    create_publisher<event_camera_msgs::msg::EventPacket>("event_camera/events", qos_event);
  RCLCPP_INFO(get_logger(), "initialized publishers");

  // slicers
  event_slicer_.doEveryTimeInterval(publish_period_, [&](const dv::EventStore & store) {
    if (store.size() > max_events_) {
      RCLCPP_WARN(
        get_logger(), "Received very many events %d > %d", int(store.size()), int(max_events_));
    }
    if (encoding_ == "dvs") {
      publishEvents(store, mask_, every_nth_px_, coordinate_divisor_, lp_filter_.get());
    } else {
      publishCompEvents(store);
    }
  });
  RCLCPP_INFO(
    get_logger(), "initialized event slicer (publish every %ld ms)",
    static_cast<long>(publish_period_.count()));

  imu_slicer_.doEveryNumberOfElements(1, [&](const dv::IMUPacket & imus) { publishImu(imus); });
  RCLCPP_INFO(get_logger(), "initialized imu slicer");

#ifdef USE_PUB_THREAD
  keepPubThreadRunning_.store(true);
  pubThread_ = std::make_shared<std::thread>(&EventCameraNode::publishingThread, this);
#endif

#ifdef OFFLINE_TESTING
  startSimulation();
#else
  // ini camera
  startCamera();
  RCLCPP_INFO(get_logger(), "initialized the camera");

  if (!camera_) {
    RCLCPP_FATAL(get_logger(), "Camera not initialized!");
    throw std::runtime_error("Camera not initialized");
  }
  // ini filters
  lp_filter_ = std::make_shared<dv::noise::LowPassFilter<dv::EventStore>>(
    *camera_->getEventResolution(), low_pass_freq_);
  createMask(every_nth_px_, *camera_->getEventResolution());
  RCLCPP_INFO(get_logger(), "initialized the events filters");
  startCameraThread();
  RCLCPP_INFO(get_logger(), "started camera thread");

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

#ifdef USE_PUB_THREAD
void EventCameraNode::publishingThread()
{
  const auto duration = std::chrono::milliseconds(100);
  while (rclcpp::ok() && keepPubThreadRunning_.load()) {
    std::unique_ptr<event_camera_msgs::msg::EventPacket> msg;
    {
      std::unique_lock<std::mutex> lock(pubMutex_);
      pubCv_.wait_for(lock, duration);
      if (pubQueue_.empty()) continue;
      msg = std::move(pubQueue_.front());
      pubQueue_.pop();
      // while (!pubQueue_.empty()) {
      //   compressed_event_pub_->publish(std::move(pubQueue_.front()));
      //   pubQueue_.pop();
      // }
    }
    compressed_event_pub_->publish(std::move(msg));
  }
  RCLCPP_WARN(get_logger(), "publishing thread exited!");
}
#endif

EventCameraNode::~EventCameraNode()
{
#ifdef USE_PUB_THREAD
  if (pubThread_) {
    {
      keepPubThreadRunning_.store(false);
      std::unique_lock<std::mutex> lock(pubMutex_);
      pubCv_.notify_all();
    }
    pubThread_->join();
    pubThread_.reset();
  }
  RCLCPP_WARN(get_logger(), "publishing thread destroyed proprely!");
#endif
  // #ifdef OFFLINE_TESTING
  //   keepSimulationThreadRunning_.store(false);
  //   if (simulationThread_.joinable()) {
  //     simulationThread_.join();
  //   }
  // #else
  //   keepCameraThreadRunning_.store(false);
  //   if (cameraThread_.joinable()) {
  //     cameraThread_.join();
  //   }
  // #endif
}

// publisher implimentations
void EventCameraNode::publishEvents(
  const dv::EventStore & store, const cv::Mat & mask, int every_nth_px, int coordinate_divisor,
  dv::EventFilterBase<dv::EventStore> * event_filter)
{
  if (store.isEmpty() || event_pub_->get_subscription_count() == 0) {
    return;
  }
  // Allow the encoder to be used without filtering (e.g. offline simulation
  // where no camera resolution is available).
  const bool use_mask = every_nth_px > 1 && !mask.empty();
  const bool use_filter = event_filter != nullptr;
  const int divisor = (coordinate_divisor > 0) ? coordinate_divisor : 1;

  dvs_msgs::msg::EventArray msg;
  msg.header.stamp = rclcpp::Time(store.getHighestTime() * 1000ULL);  // like dv-ros
  msg.header.frame_id = "events";
  msg.events.reserve(store.size());
  msg.height = static_cast<uint32_t>(image_height_ / divisor);
  msg.width = static_cast<uint32_t>(image_width_ / divisor);

  for (const auto & e : store) {
    // Apply the (static) mask before the low-pass filter so that masked-out
    // events do not update the filter's time surface. Both use the original
    // sensor coordinates.
    if (use_mask && !mask.at<uint8_t>(e.y(), e.x())) {
      continue;
    }
    // retain() is stateful and must be called exactly once per candidate event.
    if (use_filter && !event_filter->retain(e)) {
      continue;
    }
    dvs_msgs::msg::Event ev;
    ev.x = e.x() / divisor;
    ev.y = e.y() / divisor;
    ev.polarity = e.polarity();
    ev.ts = rclcpp::Time(e.timestamp() * 1000ULL);
    msg.events.push_back(ev);
  }

  if (msg.events.empty()) {
    return;
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
  if (store.isEmpty()) return;

  const int64_t sliceBaseTime = store.getLowestTime();
  if (sliceBaseTime <= 0) {
    RCLCPP_WARN(get_logger(), "Store has zero/negative base timestamp, dropping slice");
    if (eventMsg_) {
      eventMsg_.reset();  // delete currently constr. eventMsg
    }
    return;
  }
  if (sliceBaseTime < lastValidSliceTime_) {
    RCLCPP_WARN(
      get_logger(), "Timestamp regression detected: %ld < %ld, dropping slice", sliceBaseTime,
      lastValidSliceTime_);
    if (eventMsg_) {
      eventMsg_.reset();  // delete currently constr. eventMsg
    }
    return;
  }
  lastValidSliceTime_ = sliceBaseTime;

  if (compressed_event_pub_->get_subscription_count() > 0) {
    if (!eventMsg_) {
      eventMsg_.reset(new event_camera_msgs::msg::EventPacket());
      eventMsg_->header.frame_id = "events";
      eventMsg_->encoding = "libcaer_cmp";
      eventMsg_->seq = seq_++;
      eventMsg_->width = static_cast<uint32_t>(image_width_ / coordinate_divisor_);
      eventMsg_->height = static_cast<uint32_t>(image_height_ / coordinate_divisor_);
      eventMsg_->is_bigendian = false;
      eventMsg_->events.reserve(reserveSize_);
      // eventMsg_->events.reserve(store.size());  // changed from libcaer logic
      // should preallocate space for all current events
      // we can do this becuase we always publish the whole event store immediately
    }
    static_assert(std::is_same_v<decltype(sensorTime_0), uint64_t>);
    const int64_t ret = event_packed_compression::convert_polarity_events_compressed(
      eventMsg_.get(), store, mask_, every_nth_px_, coordinate_divisor_, lp_filter_.get(),
      &sensorTime_0);

    if (ret < 0) {  // invalid timestamp
      RCLCPP_WARN(this->get_logger(), "Invalid timestamp %ld, skipping", ret);
      if (eventMsg_) {
        eventMsg_.reset();  // delete currently constr. eventMsg
      }
    } else if (ret == 0) {  // nothing retained by mask/low-pass filter
      if (eventMsg_) {
        eventMsg_.reset();
      }
    } else {
      reserveSize_ = std::max(reserveSize_, eventMsg_->events.size());
#ifdef USE_PUB_THREAD
      {
        std::unique_lock<std::mutex> lock(pubMutex_);
        pubQueue_.push(std::move(eventMsg_));
        pubCv_.notify_all();
      }
#else
      compressed_event_pub_->publish(std::move(eventMsg_));  // will reset the pointer
#endif
    }
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
    // TODO: imu timestamp will overflow after circa 31 minutes, need to handle this explicitly
    int64_t msg_ts = static_cast<int64_t>(measure.timestamp) * 1000LL;
    // slicer inludes identical measurements in multiple slices, drop those
    // if (msg_ts < last_timestamp) continue;
    if (msg_ts <= last_timestamp)
      msg_ts += 1000000LL;  // enforce monotonic ts (esp. for USLAM) with 1ms diff.
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(msg_ts, RCL_SYSTEM_TIME);
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
#ifndef OFFLINE_TESTING
void EventCameraNode::startCamera()
{
  RCLCPP_INFO(get_logger(), "emplace camera ...");
  camera_.emplace();
  if (!camera_) {
    RCLCPP_FATAL(get_logger(), "Camera not initialized!");
    throw std::runtime_error("Camera not initialized");
  }
  camera_->setTimeInterval(publish_period_);
  RCLCPP_INFO(
    get_logger(), "camera data commit interval set to %ld ms",
    static_cast<long>(publish_period_.count()));
}

void EventCameraNode::startCameraThread()
{
  thread_ = std::thread([&]() {
    while (rclcpp::ok() && camera_->isRunning()) {
      bool processed_data = false;

      // Drain every packet currently queued. This prevents the camera ring
      // buffer from filling up when one iteration is slower than the producer.
      while (rclcpp::ok() && camera_->isRunning()) {
        auto packet = camera_->readNext();

        if (auto events = std::get_if<dv::EventStore>(&packet)) {
          // Mask + low-pass filtering and compression happen in a single pass
          // inside publishCompEvents(), so pass the raw events to the slicer.
#ifdef LIVE_VISUALIZATION
          accumulator_.accept(*events);
#endif
          event_slicer_.accept(*events);
          processed_data = true;
        } else if (auto imus = std::get_if<std::vector<dv::IMU>>(&packet)) {
          imu_slicer_.accept(*imus);
          processed_data = true;
        } else if (
          std::holds_alternative<dv::Frame>(packet) ||
          std::holds_alternative<std::vector<dv::Trigger>>(packet)) {
          // Frames/triggers are not published by this node, but still count as
          // drained data so we don't sleep while packets remain queued.
          processed_data = true;
        } else {
          // CONTINUE (queue empty) or END_OF_FILE.
          break;
        }
      }

      if (!processed_data) {
        std::this_thread::sleep_for(1ms);
      }
    }
  });
  thread_.detach();
}

#else
// offline signal simulation for testing
void EventCameraNode::startSimulation()
{
  thread_ = std::thread([&]() {
    // auto t = std::chrono::system_clock::now();
    // duration_cast<rclcpp::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
    // .count();
    const auto period = std::chrono::milliseconds(1);
    auto next_time = std::chrono::steady_clock::now();
    for (int i = 0; i < 20000; i++) {
      next_time += period;
      auto imus = dv::data::generate::levelImuWithNoise(i * 1000ULL, 0.1f, 0.01f, i);
      auto events = dv::data::generate::uniformEventsWithinTimeRange(
        i * 1000ULL, 1ms, cv::Size(640, 480), 1000, i);

      imu_slicer_.accept(imus);
      event_slicer_.accept(events);

      // auto diff_t = std::chrono::system_clock::now() - t;
      std::this_thread::sleep_until(next_time);
    }
  });
  thread_.detach();
}
#endif

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
  image_width_ = cols;
  image_height_ = rows;
  assert(rows % 2 == 0 && cols % 2 == 0);
  assert(rows > 0 && cols > 0);
  cv::Mat base;
  if (every_nth_px == 1) {
    base = (cv::Mat_<uchar>(2, 2) << 255, 255, 255, 255);  // mask which lets everything through
    dimension_divisor_ = 1;  // scale down the max cols and rows when publishing
    coordinate_divisor_ = 1;
  } else if (every_nth_px == 2) {
    base = (cv::Mat_<uchar>(2, 2) << 255, 0, 0, 255);
    dimension_divisor_ = 2;  // scale down the max cols and rows when publishing
    // Checkerboard mask: every row/column is still used, so keep full-res
    // coordinates to avoid merging the two kept diagonals.
    coordinate_divisor_ = 1;
  } else if (every_nth_px == 4) {
    base = (cv::Mat_<uchar>(2, 2) << 255, 0, 0, 0);
    dimension_divisor_ = 2;
    // Only even rows/columns are kept, so dropping the odd ones halves the
    // resolution in each dimension.
    coordinate_divisor_ = 2;
  }
  cv::repeat(base, rows / 2, cols / 2, mask_);  // write resulting mask to mask_ attribute
  assert(dimension_divisor_ > 0);               // unknown every_nth_px value
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
