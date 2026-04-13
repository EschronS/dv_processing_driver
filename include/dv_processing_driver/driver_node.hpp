#pragma once
#include <chrono>
#include <cstdint>
#include <dv-processing/core/core.hpp>
#include <dv-processing/core/frame/accumulator.hpp>
#include <dv-processing/data/generate.hpp>
#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/noise/frequency_filters.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "dv-processing/core/filters.hpp"
#include "event_camera_msgs/msg/event_packet.hpp"

// #define USE_PUB_THREAD

class EventCameraNode : public rclcpp::Node
{
public:
  EventCameraNode();
  ~EventCameraNode();

  bool getLatestFrame(dv::Frame & out);
  void startCamera();
  void startCameraThread();
  void startSimulation();

private:
  // Publishers
  void publishEvents(const dv::EventStore & store);
  void publishImu(const dv::IMUPacket & packet);

  // Utilities
  std::vector<uint8_t> compressBuffer(const uint8_t * data, size_t size);
  void createMask(int every_nth_px = 2, cv::Size size = cv::Size(640, 480));

  // Member variables
  std::chrono::milliseconds slice_period_;
  size_t max_events_;
  size_t max_msg_duration_;
  int every_nth_px_;
  float low_pass_freq_;

  dv::EventStreamSlicer event_slicer_;
  dv::IMUStreamSlicer imu_slicer_;
  std::unique_ptr<dv::noise::LowPassFilter<dv::EventStore>> lp_filter_;
  std::unique_ptr<dv::EventMaskFilter<dv::EventStore>> em_filter_;

  dv::Accumulator accumulator_;
  dv::Frame event_frame_;
  std::mutex frame_mutex_;
  std::optional<dv::Frame> latest_frame_;

  std::optional<dv::io::camera::DVXplorerM> camera_;
  rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr event_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  int64_t last_timestamp;
  rclcpp::Publisher<event_camera_msgs::msg::EventPacket>::SharedPtr compressed_event_pub_;

  std::thread thread_;

  event_camera_msgs::msg::EventPacket::UniquePtr eventMsg_;
  void publishCompEvents(const dv::EventStore & store);
  uint64_t seq_{0};
  uint64_t reserveSize_{0};
  uint64_t lastMsgEndTime_{0};
  int64_t lastValidSliceTime_{0};
  rclcpp::Time rosBaseTime_;
  unsigned long sensorTime_0;

  // mask filtering
  cv::Mat mask_;
  int dimension_divisor_ = -1;

  // publishing thread
#ifdef USE_PUB_THREAD
  std::queue<std::unique_ptr<event_camera_msgs::msg::EventPacket>> pubQueue_;
  std::mutex pubMutex_;
  std::condition_variable pubCv_;
  std::shared_ptr<std::thread> pubThread_;
  std::atomic_bool keepPubThreadRunning_{false};
  void publishingThread();
#endif

  // #ifdef OFFLINE_TESTING
  //   std::atomic_bool keepSimulationThreadRunning_{false};
  //   std::shared_ptr<std::thread> simulationThread_;
  // #else
  //   std::atomic_bool keepCameraThreadRunning_{false};
  //   std::shared_ptr<std::thread> cameraThread_;
  // #endif
};
