#include <time.h>

#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>
#include <thread>

#include "dv-processing/data/imu_base.hpp"

#define OFFLINE_TESTING

using namespace std::chrono_literals;
constexpr size_t MAX_EVENTS = 25000;
constexpr auto SLICE_PERIOD = 5ms;

int numberOfEvents = 0;

int main()
{
  // Initialize slicer, it will have no jobs at this time

  dv::EventStreamSlicer event_slicer;
  dv::IMUStreamSlicer imu_slicer;
  dv::EventStore buffer;

  event_slicer.doEveryTimeInterval(SLICE_PERIOD, [](const dv::EventStore & events) {
    std::cout << "* Received events time-based slicing: " << events << std::endl;
    numberOfEvents += events.size();
  });

  imu_slicer.doEveryNumberOfElements(
    1, [](const dv::IMUPacket & imus) { std::cout << "* Received IMU: " << imus << std::endl; });

#ifdef OFFLINE_TESTING
  // Generate 1000 events within 2 second interval. These will be sliced
  // correctly by the slicer.
  for (int i = 0; i < 1000; i++) {
    const auto imu_store = dv::data::generate::levelImuWithNoise(i * 1'000);
    const auto event_store =
      dv::data::generate::uniformEventsWithinTimeRange(i * 1'000, 1ms, cv::Size(640, 480), 1000);
    event_slicer.accept(event_store);
    std::this_thread::sleep_for(1ms);
    imu_slicer.accept(imu_store);
  }

#endif
#ifndef OFFLINE_TESTING

#endif

  std::cout << "Total number of events: " << numberOfEvents << std::endl;
  return 0;
}
