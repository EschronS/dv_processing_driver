#include "dv_processing_driver/compressed_encoder.hpp"

#include <chrono>
#include <cstdint>
#include <dv-processing/core/core.hpp>
#include <dv-processing/core/filters.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <limits>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

namespace event_packed_compression
{

// Filters (mask + low-pass) and encodes the events in a single pass. Returns
// the number of retained/encoded events, 0 if nothing was retained, or a
// negative timestamp if the store has no valid base time.
int64_t convert_polarity_events_compressed(
  event_camera_msgs::msg::EventPacket_<std::allocator<void> > * msg, const dv::EventStore & store,
  const cv::Mat & mask, int every_nth_px, dv::EventFilterBase<dv::EventStore> * event_filter,
  uint64_t * sensorTime_0)
{
  if (store.isEmpty()) {
    return 0;
  }

  constexpr uint64_t TIME_MASK = (1ULL << 24) - 1ULL;
  constexpr uint64_t TIME_LOW_MASK = (1ULL << 12) - 1ULL;
  constexpr uint64_t TIME_HIGH_MASK = TIME_MASK & (~TIME_LOW_MASK);
  constexpr uint16_t Y_LOW_MASK = 0x0007;        // lowest 3 bits
  constexpr uint16_t Y_HIGH_MASK = ~Y_LOW_MASK;  // highest 13 bits

  // timestamp of the first event in the event-packet
  const int64_t sensorPacketTime = store.getLowestTime();
  if (sensorPacketTime <= 0) {
    return sensorPacketTime;
  }
  const uint64_t sensorPacketTimeU = static_cast<uint64_t>(sensorPacketTime);

  // Exactly one EventStore per message: always start from a clean message.
  auto & events = msg->events;
  events.clear();

  // ROS time at which this message is produced. This is the message stamp;
  // the event timestamps themselves are reconstructed from time_base below.
  const uint64_t nowNs = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
  msg->header.stamp = rclcpp::Time(nowNs, RCL_SYSTEM_TIME);

  *sensorTime_0 = sensorPacketTimeU;  // start time of the first event in the packet
  // The libcaer_cmp decoder reconstructs absolute event times from time_base,
  // so it must be the sensor time of the first event in this message (µs -> ns).
  msg->time_base = sensorPacketTimeU * 1000ULL;

  const uint64_t dt_0 = sensorPacketTimeU - *sensorTime_0;
  uint64_t timeHigh = dt_0 & TIME_HIGH_MASK;
  uint64_t timeLow = dt_0 & TIME_LOW_MASK;

  // Sentinel so that the first retained event always emits an AddrX token.
  uint16_t currentX = std::numeric_limits<uint16_t>::max();
  uint16_t currentY = std::numeric_limits<uint16_t>::max();
  uint16_t currentYHigh = currentY & Y_HIGH_MASK;
  std::bitset<8> currentMask(0);
  uint8_t currentPolarity = 2;

  // Allow the encoder to be used without filtering (e.g. offline simulation
  // where no camera resolution is available).
  const bool use_mask = every_nth_px > 1 && !mask.empty();
  const bool use_filter = event_filter != nullptr;

  int64_t retained = 0;
  for (const auto & e : store) {
    // Apply the (static) mask before the low-pass filter so that masked-out
    // events do not update the filter's time surface.
    if (use_mask && !mask.at<uint8_t>(e.y(), e.x())) {
      continue;
    }
    // retain() is stateful and must be called exactly once per candidate event.
    if (use_filter && !event_filter->retain(e)) {
      continue;
    }
    ++retained;

    const uint64_t dt = e.timestamp() - *sensorTime_0;

    if ((dt & TIME_HIGH_MASK) != timeHigh) {
      timeHigh = dt & TIME_HIGH_MASK;
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      add_to_events(&events, TimeHigh(static_cast<uint16_t>(timeHigh >> 12)));
    }

    if ((dt & TIME_LOW_MASK) != timeLow) {
      timeLow = dt & TIME_LOW_MASK;
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      add_to_events(&events, TimeLow(static_cast<uint16_t>(timeLow)));
    }

    if (e.x() != currentX) {
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      currentX = e.x();
      add_to_events(&events, AddrX(currentX));
    }

    if (e.polarity() != currentPolarity) {
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      currentPolarity = e.polarity();
    }

    const uint16_t y = e.y();
    const uint16_t y_high = y & Y_HIGH_MASK;
    const uint16_t y_low = y & Y_LOW_MASK;

    if (y_high != currentYHigh) {
      flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
      currentYHigh = y_high;
      currentY = y;  // for easier access in case of sparse event
    }
    // mark the current bit in the mask
    currentMask.set(y_low, 1);
  }

  flush_state(&events, &currentMask, currentY, currentYHigh, currentPolarity);
  return retained;
}
}  // namespace event_packed_compression
