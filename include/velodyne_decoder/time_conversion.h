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

using Time = double;

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 *
 * @param packet_time timestamp recovered from velodyne
 * @param arrival_time time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
Time resolveHourAmbiguity(const Time packet_time, const Time arrival_time) {
  const int HALFHOUR_TO_SEC = 1800;
  Time retval               = packet_time;
  uint32_t packet_sec       = (uint32_t)std::floor(packet_time);
  uint32_t arrival_sec      = (uint32_t)std::floor(arrival_time);
  if (arrival_sec > packet_sec) {
    if (arrival_sec - packet_sec > HALFHOUR_TO_SEC) {
      retval += 2 * HALFHOUR_TO_SEC;
    }
  } else if (packet_sec - arrival_sec > HALFHOUR_TO_SEC) {
    retval -= 2 * HALFHOUR_TO_SEC;
  }
  return retval;
}

Time getPacketTimestamp(const uint8_t *const data, Time arrival_time) {
  // time for each packet is a 4 byte uint
  // It is the number of microseconds from the top of the hour
  uint32_t usecs        = (((uint32_t)data[3]) << 24u | //
                    ((uint32_t)data[2]) << 16u | //
                    ((uint32_t)data[1]) << 8u |  //
                    ((uint32_t)data[0]));
  const int HOUR_TO_SEC = 3600;
  uint32_t cur_hour     = (uint32_t)std::floor(arrival_time) / HOUR_TO_SEC;
  Time packet_time      = (cur_hour * HOUR_TO_SEC) + (usecs * 1e-6);
  if (arrival_time > 0) {
    packet_time = resolveHourAmbiguity(packet_time, arrival_time);
  }
  return packet_time;
}
