// Copyright (C) 2007-2021 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley, Martin Valgur
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

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "velodyne_decoder/calibration.h"
#include "velodyne_decoder/config.h"
#include "velodyne_decoder/pointcloud_aggregator.h"
#include "velodyne_decoder/types.h"

namespace velodyne_decoder {

/** \brief Velodyne data conversion class */
class PacketDecoder {
public:
  explicit PacketDecoder(const Config &config);

  void unpack(const VelodynePacket &pkt, PointCloudAggregator &data, Time scan_start_time);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  int scansPerPacket() const;

  /** configuration parameters */
  Config config_;

  /** calibration file */
  velodyne_decoder::Calibration calibration_;

protected:
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // Caches the azimuth percent offset for the VLS-128 laser firings
  float vls_128_laser_azimuth_cache[16];

  // timing offset lookup table
  std::vector<std::vector<float>> timing_offsets_;

  /** \brief setup per-point timing offsets
   *
   *  Runs during initialization and determines the firing time for each point in the scan
   */
  static std::vector<std::vector<float>> buildTimings(const std::string &model);

  void setupSinCosCache();
  void setupAzimuthCache();

  /** add private function to handle the VLP16 **/
  void unpack_vlp16(const VelodynePacket &pkt, PointCloudAggregator &data,
                    Time scan_start_time) const;

  void unpack_vlp32_vlp64(const VelodynePacket &pkt, PointCloudAggregator &data,
                          Time scan_start_time) const;

  void unpack_vls128(const VelodynePacket &pkt, PointCloudAggregator &data,
                     Time scan_start_time) const;

  void unpackPointCommon(PointCloudAggregator &data, const LaserCorrection &corrections,
                         const raw_measurement_t &measurement, uint16_t azimuth, float time) const;

  /** in-line test whether a point is in range */
  constexpr bool pointInRange(float range) const {
    return range >= config_.min_range && range <= config_.max_range;
  }
};

} // namespace velodyne_decoder
