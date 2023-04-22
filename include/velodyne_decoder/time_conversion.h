// Copyright (c) 2019, Matthew Pitropov, Joshua Whitley
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

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
 * @param usec_since_toh number of microseconds from the top of the hour
 * @param reference_time reference time as a timestamp in seconds since the Unix epoch
 * @return timestamp in seconds since the Unix epoch
 */
Time getPacketTimestamp(uint32_t usec_since_toh, Time reference_time);

} // namespace velodyne_decoder
