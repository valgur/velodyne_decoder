// Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "velodyne_decoder/time_conversion.h"

#include <cmath>
#include <cstdint>

namespace velodyne_decoder {

Time resolveHourAmbiguity(const Time packet_time, const Time reference_time) {
  const double HOUR      = 3600;
  const double HALF_HOUR = 1800;
  if (packet_time > reference_time + HALF_HOUR)
    return packet_time - HOUR;
  if (packet_time < reference_time - HALF_HOUR)
    return packet_time + HOUR;
  return packet_time;
}

Time getPacketTimestamp(const uint32_t toh_usec, const Time reference_time) {
  const int HOUR_TO_SEC = 3600;
  uint32_t cur_hour     = (uint32_t)std::floor(reference_time) / HOUR_TO_SEC;
  Time packet_time      = cur_hour * HOUR_TO_SEC + toh_usec * 1e-6;
  if (reference_time > 0)
    return resolveHourAmbiguity(packet_time, reference_time);
  return packet_time;
}

} // namespace velodyne_decoder
