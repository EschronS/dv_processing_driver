# Simple Driver for DVXplorer (-micro) event camera
- uses the modern dv-processing lib. from Inivation
- publishes highly compressed event data in the ROS2 ecosystem

# About this Repo
While working with the DVXplorer-micro event camera I encountered the problem of the firmware deprecating support for libcaer. This made it impossible for me to use awesome ROS2 event-camera [driver-stack](https://github.com/ros-event-camera) from Bernd Pfrommer because of it's dependence on libcaer. This repository here is a minimal ROS2 driver node which uses dv-processing and the compression implementation from Bernd Pfrommer to efficiently publish event data from the DVXplorer-micro under ROS2.

# Other Solutions
- if your application can use ROS1 the maintained ROS1-driver [dv-ros](https://gitlab.com/inivation/dv/dv-ros) from Inivation is a good alternative

# How To Use
## Prerequisites
- A ROS 2 installation (source it for your distro, e.g. `source /opt/ros/jazzy/setup.bash`)
- `git`, `colcon`, and `rosdep`
- CMake >= 3.22 and a C++23-capable compiler. The vendored dv-processing requires GCC >= 13 or Clang >= 18.
- The system libraries required by dv-processing: Boost >= 1.80 with `nowide`, OpenSSL, Eigen3 >= 3.4, fmt >= 9.1, libusb >= 1.0.23, OpenCV >= 4.2, zlib, and `pkg-config`.

[`dv-processing`](https://gitlab.com/inivation/dv/dv-processing) is vendored in this repository as the git submodule `thirdparty/dv-processing` and is built from source as part of this package. You do **not** need to install dv-processing separately.

## Get the source
Clone the repository including submodules:

```bash
git clone --recurse-submodules <repository-url>
```

If you already cloned without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

The `--recursive` flag is required because dv-processing has its own submodules (for example CLI11), which its CMake build uses.

## Install ROS dependencies (from `package.xml`)
From the root of your ROS 2 workspace (the directory containing `src/`):

```bash
# One-time setup per machine (skip if already done)
sudo rosdep init
rosdep update

# Install dependencies declared in src/**/package.xml
rosdep install --from-paths src --ignore-src -r -y
```

Notes:
- `--ignore-src` avoids trying to install packages that are already in the workspace.
- `-r` continues if a dependency key cannot be resolved on your platform.
- The vendored dv-processing is not installed by `rosdep`. Its remaining transitive system dependencies are not declared in this package's `package.xml`, so install them with your distribution's package manager. On Ubuntu 24.04 / ROS 2 Jazzy, for example:

  ```bash
  sudo apt install build-essential cmake libboost-nowide-dev libssl-dev \
    libeigen3-dev libfmt-dev libusb-1.0-0-dev libopencv-dev zlib1g-dev pkg-config
  ```

## Build
```bash
colcon build --packages-select dv_processing_driver
source install/setup.bash
```

### Vendored dv-processing event buffer patch
The build applies `patches/dv-processing-event-ringbuffer.patch` to the vendored dv-processing source before adding it as a subdirectory. The patch:

- increases only the event ring buffer from the upstream default of 256 packets to 1024 packets,
- leaves the frame, IMU, and trigger buffers at their upstream defaults,
- prints a warning to stderr when the event buffer is full.

The patch is applied idempotently during CMake configuration. The submodule working tree will show the patched header as modified; this is expected. The patch itself is tracked in this repository. If you need to reset the submodule manually:

```bash
git -C thirdparty/dv-processing checkout -- include/dv-processing/io/camera/camera_input_base.hpp
```

Upstream dv-processing 2.0.4 already provides the protected `SortedPackets<T, BUFFER_SIZE>` template used by this patch, but it does not expose a public runtime buffer-size API yet. The local patch is therefore still required.

## Use
```bash
ros2 launch dv_processing_driver driver_node.launch.py
```

### Parameters
- `publish_period_ms` (default `10`): publish one event message per N milliseconds of event time. The camera parser commit interval is set to the same value.
- `max_events` (default `3000`): warn when one publish period contains more events than this. It no longer controls the publish cadence.
- `every_nth_px` (default `4`): event mask downsampling.
- `low_pass_freq` (default `40.0`): low-pass filter cutoff frequency.

### Preview
you can preview the published events with [event_camera_renderer](https://github.com/ros-event-camera/event_camera_renderer/)


## Credit
The compression is adapted from [libcaer_driver](https://github.com/ros-event-camera/libcaer_driver/blob/master/src/message_converter.cpp) by Bernd Pfrommer
- [dv-processing source code](https://gitlab.com/inivation/dv/dv-processing)
- [dv-processing documentation](https://dv-processing.inivation.com/)
- [dv-processing API reference](https://dv-processing.inivation.com/master/api.html)

