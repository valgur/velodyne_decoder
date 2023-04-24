// Copyright (c) 2007-2019, Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
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

  void unpack(Time stamp, const raw_packet_t &pkt_data, PointCloud &cloud, Time scan_start_time);

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
  [[nodiscard]] static std::array<std::array<float, 32>, 12> buildTimings(ModelId model);

  void correctVls128Timings(uint32_t stamp, bool dual_return);

  static void verifyPacketModelId(PacketModelId packet_model_id, ModelId model_id);

  void unpack_16_32_beam(const raw_packet_t &raw, float rel_packet_stamp, PointCloud &cloud);
  void unpack_hdl64e_s1(const raw_packet_t &raw, float rel_packet_stamp, PointCloud &cloud);
  void unpack_hdl64e(const raw_packet_t &raw, float rel_packet_stamp, PointCloud &cloud);
  void unpack_vls128(const raw_packet_t &raw, float rel_packet_stamp, PointCloud &cloud);

  void unpackPointDual(PointCloud &cloud, int laser_idx, uint16_t azimuth, float time,
                       raw_measurement_t last, raw_measurement_t strongest) const;

  void unpackPoint(PointCloud &cloud, int laser_idx, uint16_t azimuth, float time,
                   raw_measurement_t measurement, ReturnModeFlag return_mode_flag) const;

  /** in-line test whether a point is in range */
  [[nodiscard]] bool distanceInRange(float range) const;
  [[nodiscard]] bool azimuthInRange(uint16_t azimuth) const;

private:
  velodyne_decoder::Calibration calibration_;
  bool calib_initialized_ = false;

  std::optional<ModelId> model_id_;
  std::optional<DualReturnMode> return_mode_;
  float min_range_;
  float max_range_;
  uint16_t min_azimuth_;
  uint16_t max_azimuth_;
  bool single_return_mode_info_;

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
  std::array<std::array<float, 32>, 12> timing_offsets_;

  // First azimuth in the previous packet.
  // Needed for dual-return mode VLS-128, where only a single column is stored per-packet.
  uint16_t prev_packet_azimuth_ = std::numeric_limits<uint16_t>::max();
  float prev_rotation_rate_     = 0;

  // For VLS-128 timing corrections, which differ based on firmware version
  std::optional<uint32_t> prev_packet_stamp_;
  static constexpr double default_vls_128_channel_duration_ = 2.89 * 1e-6; // initial value
  bool vls_128_channel_duration_corrected_                  = false;
};

} // namespace velodyne_decoder
