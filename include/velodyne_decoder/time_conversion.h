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

#pragma once

#include <cstdint>

namespace velodyne_decoder {

using Time = double;

constexpr uint32_t parseUint32(const uint8_t *data) {
  return (((uint32_t)data[3]) << 24u | //
          ((uint32_t)data[2]) << 16u | //
          ((uint32_t)data[1]) << 8u |  //
          ((uint32_t)data[0]));
}

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 *
 * @param packet_time timestamp recovered from velodyne
 * @param reference_time time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
Time resolveHourAmbiguity(Time packet_time, Time reference_time);

/**
 * @brief Merges the the top of the hour timestamp from the Velodyne lidar with a
 * reference timestamp, i.e. time from the computer's clock or a GPS receiver.
 *
 * @param toh_usec number of microseconds from the top of the hour
 * @param reference_time reference time as a timestamp in seconds since the Unix epoch
 * @return timestamp in seconds since the Unix epoch
 */
Time getPacketTimestamp(uint32_t toh_usec, Time reference_time);

} // namespace velodyne_decoder
