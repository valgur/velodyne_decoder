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

#include <cstdint>
#include <string>
#include <vector>

#include "velodyne_decoder/calibration.h"
#include "velodyne_decoder/config.h"
#include "velodyne_decoder/types.h"

namespace velodyne_decoder {

/** \brief Velodyne data conversion class */
class PacketDecoder {
public:
  explicit PacketDecoder(const Config &config);

  void unpack(const VelodynePacket &pkt, PointCloud &cloud, Time scan_start_time);

private:
  void initModel(ModelId model_id);
  void initModel(PacketModelId packet_model_id);
  void initCalibration(const Calibration &calibration);

  void setupSinCosCache();
  void setupCalibrationCache(const Calibration &calibration);

  /** \brief setup per-point timing offsets
   *
   *  Runs during initialization and determines the firing time for each point in the scan
   */
  [[nodiscard]] static std::vector<std::vector<float>> buildTimings(ModelId model);

  static void verifyPacketModelId(PacketModelId packet_model_id, ModelId model_id);

  void unpack_16_32_beam(const raw_packet_t &raw, Time udp_stamp, PointCloud &cloud,
                         Time scan_start_time) const;
  void unpack_hdl64e(const raw_packet_t &raw, Time stamp, PointCloud &cloud,
                     Time scan_start_time) const;
  void unpack_vls128(const raw_packet_t &raw, Time stamp, PointCloud &cloud, Time scan_start_time);

  void unpackPoint(PointCloud &cloud, int laser_idx, const raw_measurement_t &measurement,
                   uint16_t azimuth, float time, bool last_return_mode) const;

  /** in-line test whether a point is in range */
  [[nodiscard]] bool distanceInRange(float range) const;
  [[nodiscard]] bool azimuthInRange(uint16_t azimuth) const;

private:
  velodyne_decoder::Calibration calibration_;
  bool calib_initialized_ = false;

  std::optional<ModelId> model_id_;
  float min_range_;
  float max_range_;
  uint16_t min_azimuth_;
  uint16_t max_azimuth_;

  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // Cache of basic calibration parameters
  bool apply_advanced_calibration_;
  std::vector<float> cos_rot_correction_;  ///< cosine of rot_corrections
  std::vector<float> sin_rot_correction_;  ///< sine of rot_corrections
  std::vector<float> cos_vert_correction_; ///< cosine of vert_corrections
  std::vector<float> sin_vert_correction_; ///< sine of vert_corrections
  std::vector<uint16_t> ring_cache_;       ///< cache for ring lookup

  // timing offset lookup table
  std::vector<std::vector<float>> timing_offsets_;

  // First azimuth in the previous packet.
  // Needed for dual-return mode VLS-128, where only a single column is stored per-packet.
  uint16_t prev_packet_azimuth_ = std::numeric_limits<uint16_t>::max();
  float prev_rotation_rate_     = 0;
};

} // namespace velodyne_decoder
