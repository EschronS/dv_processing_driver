#include "dv_processing_driver/compressed_encoder.hpp"

#include <cstddef>  // size_t
#include <dv-processing/core/core.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/time.hpp>

#include "event_camera_msgs/msg/event_packet.hpp"

namespace event_packed_compression
{

size_t convert_polarity_events_compressed(
  event_camera_msgs::msg::EventPacket_<std::allocator<void> > * msg, const dv::EventStore & store,
  const rclcpp::Time & baseTime, uint64_t * sensorTime_0)
{
  if (store.isEmpty()) {
    return 0;
  }

  constexpr uint64_t TIME_MASK = (1ULL << 24) - 1ULL;
  constexpr uint64_t TIME_LOW_MASK = (1ULL << 12) - 1ULL;
  constexpr uint64_t TIME_HIGH_MASK = TIME_MASK & (~TIME_LOW_MASK);
  constexpr uint16_t Y_LOW_MASK = 0x0007;        // lowest 3 bits
  constexpr uint16_t Y_HIGH_MASK = ~Y_LOW_MASK;  // highest 13 bits

  const auto & first = store.front();
  const uint64_t sensorPacketTime = first.timestamp();

  auto & events = msg->events;
  if (events.empty()) {
    // in contrast to libcaer_driver, we use ROS-time in time-base field
    uint64_t t =
      duration_cast<rclcpp::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();

    msg->header.stamp =
      rclcpp::Time(sensorPacketTime * 1000 + baseTime.nanoseconds(), RCL_SYSTEM_TIME);
    msg->time_base = t;
    *sensorTime_0 = sensorPacketTime;
  }

  const uint64_t dt_0 = sensorPacketTime - *sensorTime_0;
  uint16_t timeHigh = dt_0 & TIME_HIGH_MASK;
  uint64_t timeLow = dt_0 & TIME_LOW_MASK;

  uint16_t currentX = first.x();
  add_to_events(&events, AddrX(currentX));

  uint16_t currentY = std::numeric_limits<uint16_t>::max();
  uint16_t currentYHigh = currentY & Y_HIGH_MASK;
  std::bitset<8> currentMask(0);
  uint8_t currentPolarity = 2;

  for (const auto & e : store) {
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
  return store.size();
}
}  // namespace event_packed_compression
