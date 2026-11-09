// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIBCAER_DRIVER__COMPRESSION_TOKENS_HPP_
#define LIBCAER_DRIVER__COMPRESSION_TOKENS_HPP_

#include <bitset>
#include <cstdint>
#ifdef MICRO
#include <dv-processing/io/camera/dvxplorer_m.hpp>
#else
#include <dv-processing/io/camera/dvxplorer.hpp>
#endif
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <vector>

#include "dv-processing/core/core.hpp"
#include "dv-processing/core/filters.hpp"
#include "event_camera_msgs/msg/event_packet.hpp"

namespace event_packed_compression
{
enum Code { TIME_HIGH, TIME_LOW, ADDR_X, ADDR_Y, VECT_BASE_Y, VECT_8 };

struct __attribute__((packed)) TimeLow
{
  TimeLow(uint16_t ts_usec) : t(ts_usec), code(Code::TIME_LOW) {}
  uint16_t t : 12;
  uint16_t code : 4;
};

struct __attribute__((packed)) TimeHigh
{
  TimeHigh(uint16_t ts) : t(ts), code(Code::TIME_HIGH) {}
  uint16_t t : 12;
  uint16_t code : 4;
};

struct __attribute__((packed)) AddrX
{
  AddrX(uint16_t xa) : x(xa), code(Code::ADDR_X) {}
  uint16_t x : 11;
  uint16_t unused : 1;
  uint16_t code : 4;
};

struct __attribute__((packed)) AddrY
{
  AddrY(uint16_t ya, uint8_t p) : y(ya), polarity(p), code(Code::ADDR_Y) {}
  uint16_t y : 11;
  uint16_t polarity : 1;
  uint16_t code : 4;
};

struct __attribute__((packed)) VectBaseY
{
  VectBaseY(uint16_t ya, uint16_t pa) : y(ya), polarity(pa), code(Code::VECT_BASE_Y) {}
  uint16_t y : 11;
  uint16_t polarity : 1;
  uint16_t code : 4;
};

struct __attribute__((packed)) Vect8
{
  Vect8(uint16_t v) : valid(v), code(Code::VECT_8) {}
  uint16_t valid : 8;
  uint16_t unused : 4;
  uint16_t code : 4;
};

template <class T>
static void add_to_events(std::vector<uint8_t> * events, const T & token)
{
  const uint8_t * p = reinterpret_cast<const uint8_t *>(&token);
  events->push_back(p[0]);  // token must be 16 bytes or else!
  events->push_back(p[1]);
}

static inline void flush_state(
  std::vector<uint8_t> * events, std::bitset<8> * currentMask, uint16_t currentY,
  uint16_t currentYHigh, uint8_t currentPolarity)
{
  if (currentMask->any()) {
    if (currentMask->count() == 1) {
      add_to_events<AddrY>(events, AddrY(currentY, currentPolarity));
    } else {
      add_to_events(events, VectBaseY(currentYHigh, currentPolarity));
      add_to_events(events, Vect8(static_cast<uint16_t>(currentMask->to_ulong())));
    }
    currentMask->reset();
  }
}

int64_t convert_polarity_events_compressed(
  event_camera_msgs::msg::EventPacket_<std::allocator<void> > * msg, const dv::EventStore & store,
  const cv::Mat & mask, int every_nth_px, int coordinate_divisor,
  dv::EventFilterBase<dv::EventStore> * event_filter, uint64_t * sensorTime_0);

}  // namespace event_packed_compression

#endif  // LIBCAER_DRIVER__COMPRESSION_TOKENS_HPP_
