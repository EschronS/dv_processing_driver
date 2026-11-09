#include <time.h>

#include <chrono>
#include <dv-processing/core/core.hpp>
#include <dv-processing/data/generate.hpp>
#include <thread>

#include "dv-processing/data/imu_base.hpp"

#undef MICRO
#ifdef MICRO
#include "dv-processing/io/camera/dvxplorer_m.hpp"
#else
#include "dv-processing/io/camera/dvxplorer.hpp"
#endif

// #define OFFLINE_TESTING

using namespace std::chrono_literals;
constexpr size_t MAX_EVENTS = 25000;
constexpr auto SLICE_PERIOD = 5ms;

int numberOfEvents = 0;

int main()
{
  // Initialize slicer, it will have no jobs at this time

  dv::EventStreamSlicer event_slicer;
  dv::IMUStreamSlicer imu_slicer;
  // dv::EventStore buffer;

  event_slicer.doEveryTimeInterval(SLICE_PERIOD, [](const dv::EventStore & events) {
    static auto lastLog = std::chrono::steady_clock::time_point{};
    const auto now     = std::chrono::steady_clock::now();
    if (now - lastLog >= std::chrono::seconds(1)) {
      lastLog = now;
      std::cout << "* Received events time-based slicing: " << events << std::endl;
    }
    numberOfEvents += events.size();
  });

  imu_slicer.doEveryNumberOfElements(1, [](const dv::IMUPacket & imus) {
    static auto lastLog = std::chrono::steady_clock::time_point{};
    const auto now     = std::chrono::steady_clock::now();
    if (now - lastLog >= std::chrono::seconds(1)) {
      lastLog = now;
      std::cout << "* Received IMU: " << imus << std::endl;
    }
  });

#ifdef OFFLINE_TESTING
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

#ifdef MICRO
  auto camera = dv::io::camera::DVXplorerM();
#else
  auto camera = dv::io::camera::DVXplorer();
#endif

  while (camera.isRunning()) {
    // Handle events
    if (const auto events = camera.getNextEventBatch(); events.has_value()) {
      event_slicer.accept(*events);
    }

    // Handle frames
    if (const auto imus = camera.getNextImuBatch(); imus.has_value()) {
      imu_slicer.accept(*imus);
    }
  }

#endif

  std::cout << "Total number of events: " << numberOfEvents << std::endl;
  return 0;
}
