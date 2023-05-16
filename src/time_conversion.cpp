// Copyright (c) 2019, Matthew Pitropov, Joshua Whitley
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

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

Time getPacketTimestamp(const uint32_t usec_since_toh, const Time reference_time) {
  const int HOUR_TO_SEC = 3600;
  uint32_t cur_hour     = (uint32_t)std::floor(reference_time) / HOUR_TO_SEC;
  Time packet_time      = cur_hour * HOUR_TO_SEC + usec_since_toh * 1e-6;
  if (reference_time > 0)
    return resolveHourAmbiguity(packet_time, reference_time);
  return packet_time;
}

Time getPacketTimestamp(gsl::span<const uint8_t, PACKET_SIZE> pkt_data, Time reference_time) {
  uint32_t usec_since_toh = parseUint32(&pkt_data[PACKET_SIZE - 6]);
  return getPacketTimestamp(usec_since_toh, reference_time);
}

} // namespace velodyne_decoder
