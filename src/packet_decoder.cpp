/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2019, Kaarta Inc, Shawn Hanna
 *  Copyright (C) 2021, Martin Valgur
 *
 *  License: Modified BSD Software License Agreement
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Shawn Hanna
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include "velodyne_decoder/packet_decoder.h"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace velodyne_decoder {
template <typename T> constexpr T SQR(T val) { return val * val; }

PacketDecoder::PacketDecoder(const Config &config) : config_(config) {
  if (config_.model.empty()) {
    throw std::invalid_argument("No Velodyne sensor model specified!");
  }
  if (!config_.calibration) {
    throw std::invalid_argument("No calibration provided!");
  }
  calibration_ = *config_.calibration;

  setupCalibrationCache(calibration_);
  timing_offsets_ = buildTimings(config_.model);
  setupSinCosCache();
  setupAzimuthCache();
}

bool PacketDecoder::distanceInRange(float range) const {
  return range >= config_.min_range && range <= config_.max_range;
}

bool PacketDecoder::azimuthInRange(int azimuth) const {
  if (config_.min_angle <= config_.max_angle) {
    return azimuth >= config_.min_angle && azimuth <= config_.max_angle;
  } else {
    return azimuth <= config_.min_angle && azimuth >= config_.max_angle;
  }
}

/**
 * Build a timing table for each block/firing.
 */
std::vector<std::vector<float>> PacketDecoder::buildTimings(const std::string &model) {
  std::vector<std::vector<float>> timing_offsets;
  if (model == "VLP-16") {
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (auto &timing_offset : timing_offsets) {
      timing_offset.resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6; // seconds
    double single_firing     = 2.304 * 1e-6;  // seconds
    bool dual_mode           = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        double dataBlockIndex = 2 * (dual_mode ? x / 2 : x) + y / 16;
        double dataPointIndex = y % 16;
        // timing_offsets[block][firing]
        timing_offsets[x][y] =
            (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
      }
    }
  } else if (model == "VLP-32C") {
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (auto &timing_offset : timing_offsets) {
      timing_offset.resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6; // seconds
    double single_firing     = 2.304 * 1e-6;  // seconds
    bool dual_mode           = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        double dataBlockIndex = dual_mode ? x / 2 : x;
        double dataPointIndex = y / 2;
        timing_offsets[x][y] =
            (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
      }
    }
  } else if (model == "HDL-32E") {
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (auto &timing_offset : timing_offsets) {
      timing_offset.resize(32);
    }
    // constants
    double full_firing_cycle = 46.080 * 1e-6; // seconds
    double single_firing     = 1.152 * 1e-6;  // seconds
    bool dual_mode           = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        double dataBlockIndex = dual_mode ? x / 2 : x;
        double dataPointIndex = y / 2;
        timing_offsets[x][y] =
            (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
      }
    }
  } else if (model == "Alpha Prime") {
    timing_offsets.resize(3);
    for (auto &timing_offset : timing_offsets) {
      timing_offset.resize(17); // 17 (+1 for the maintenance time after firing group 8)
    }

    double full_firing_cycle = VLS128_SEQ_TDURATION * 1e-6;     // seconds
    double single_firing     = VLS128_CHANNEL_TDURATION * 1e-6; // seconds
    double offset_paket_time = VLS128_TOH_ADJUSTMENT * 1e-6;    // seconds
    // Compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x) {
      for (size_t y = 0; y < timing_offsets[x].size(); ++y) {
        double sequenceIndex    = x;
        double firingGroupIndex = y;
        timing_offsets[x][y]    = (full_firing_cycle * sequenceIndex) +
                               (single_firing * firingGroupIndex) - offset_paket_time;
      }
    }
  } else if (std::find(Config::SUPPORTED_MODELS.begin(), Config::SUPPORTED_MODELS.end(), model) ==
             Config::SUPPORTED_MODELS.end()) {
    throw std::runtime_error("Unsupported Velodyne model: " + model);
  } else {
    // TODO: add require_timings
    // throw std::runtime_error("Timings not available for Velodyne model " + model);
  }
  return timing_offsets;
}

void PacketDecoder::setupSinCosCache() {
  // Set up cached values for sin and cos of all the possible headings
  constexpr float deg2rad = M_PI / 180.f;
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation            = ROTATION_RESOLUTION * rot_index * deg2rad;
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
}

void PacketDecoder::setupAzimuthCache() {
  if (config_.model == "Alpha Prime") {
    for (uint8_t i = 0; i < 16; i++) {
      vls_128_laser_azimuth_cache_[i] =
          (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }
  }
}

void PacketDecoder::setupCalibrationCache(const Calibration &calibration) {
  apply_advanced_calibration_ = calibration.isAdvancedCalibration();
  cos_rot_correction_.resize(calibration.num_lasers);
  sin_rot_correction_.resize(calibration.num_lasers);
  cos_vert_correction_.resize(calibration.num_lasers);
  sin_vert_correction_.resize(calibration.num_lasers);
  ring_cache_.resize(calibration.num_lasers);
  for (int i = 0; i < calibration.num_lasers; i++) {
    cos_rot_correction_[i]  = cosf(calibration.laser_corrections[i].rot_correction);
    sin_rot_correction_[i]  = sinf(calibration.laser_corrections[i].rot_correction);
    cos_vert_correction_[i] = cosf(calibration.laser_corrections[i].vert_correction);
    sin_vert_correction_[i] = sinf(calibration.laser_corrections[i].vert_correction);
    ring_cache_[i]          = calibration.laser_corrections[i].laser_ring;
  }
}

/** @brief convert raw packet to point cloud
 */
void PacketDecoder::unpack(const VelodynePacket &pkt, PointCloud &cloud, Time scan_start_time) {
  const raw_packet_t &raw_packet = *reinterpret_cast<const raw_packet_t *>(pkt.data.data());

  if (config_.model == "Alpha Prime") {
    unpack_vls128(raw_packet, pkt.stamp, cloud, scan_start_time);
    return;
  }

  if (calibration_.num_lasers == 16) {
    unpack_vlp16(raw_packet, pkt.stamp, cloud, scan_start_time);
    return;
  }

  unpack_vlp32_vlp64(raw_packet, pkt.stamp, cloud, scan_start_time);
}

/** @brief convert raw VLP16 packet to point cloud
 */
void PacketDecoder::unpack_vlp16(const raw_packet_t &raw, Time stamp, PointCloud &cloud,
                                 Time scan_start_time) const {
  float time_diff_start_to_this_packet = stamp - scan_start_time;

  float last_azimuth_diff = 0;

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    const auto &block = raw.blocks[i];

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != block.header) {
      return; // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    float azimuth      = block.rotation;
    float azimuth_diff = last_azimuth_diff;
    if (i < (BLOCKS_PER_PACKET - 1)) {
      int raw_azimuth_diff = raw.blocks[i + 1].rotation - block.rotation;
      azimuth_diff         = (float)((36000 + raw_azimuth_diff) % 36000);
      // some packets contain an angle overflow where azimuth_diff < 0
      if (raw_azimuth_diff < 0) {
        // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change
        // very much and use the same difference
        if (last_azimuth_diff > 0) {
          azimuth_diff = last_azimuth_diff;
        }
        // otherwise we are not able to use this data
        // TODO: we might just not use the second 16 firings
        else {
          continue;
        }
      }
      last_azimuth_diff = azimuth_diff;
    }

    for (int firing = 0, j = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, j++) {
        if (block.data[j].distance == 0)
          continue;

        /** correct for the laser rotation as a function of timing during the firings **/
        float azimuth_corrected_f =
            azimuth + (azimuth_diff * (dsr * VLP16_DSR_TOFFSET + firing * VLP16_FIRING_TOFFSET) /
                       VLP16_BLOCK_TDURATION);
        int azimuth_corrected = std::lround(azimuth_corrected_f + 36000) % 36000;

        if (!azimuthInRange(azimuth_corrected))
          continue;

        float time = 0;
        if (!timing_offsets_.empty())
          time = timing_offsets_[i][firing * 16 + dsr] + time_diff_start_to_this_packet;

        const auto &measurement = block.data[j];
        unpackPointCommon(cloud, dsr, measurement, azimuth_corrected, time);
      }
    }
  }
}

/** @brief convert raw VLP-32/64 packet to point cloud
 */
void PacketDecoder::unpack_vlp32_vlp64(const raw_packet_t &raw, Time stamp, PointCloud &cloud,
                                       Time scan_start_time) const {
  float time_diff_start_to_this_packet = stamp - scan_start_time;

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    const auto &block = raw.blocks[i];

    // upper bank lasers are numbered [0..31]
    // lower bank lasers are [32..63]
    int bank_origin = (block.header == UPPER_BANK) ? 0 : 32;

    uint16_t azimuth = block.rotation;

    if (!azimuthInRange(azimuth))
      continue;

    for (int j = 0; j < SCANS_PER_BLOCK; j++) {
      if (block.data[j].distance == 0)
        continue;

      const uint8_t laser_number = j + bank_origin;

      float time = 0;
      if (!timing_offsets_.empty()) {
        time = timing_offsets_[i][j] + time_diff_start_to_this_packet;
      }

      const auto &measurement = block.data[j];
      unpackPointCommon(cloud, laser_number, measurement, azimuth, time);
    }
  }
}

/** @brief Applies calibration, converts to a Cartesian 3D point and appends to the cloud.
 */
void PacketDecoder::unpackPointCommon(PointCloud &cloud, int laser_idx,
                                      const raw_measurement_t &measurement, uint16_t azimuth,
                                      float time) const {
  float distance = measurement.distance * calibration_.distance_resolution_m;

  float cos_vert_angle     = cos_vert_correction_[laser_idx];
  float sin_vert_angle     = sin_vert_correction_[laser_idx];
  float cos_rot_correction = cos_rot_correction_[laser_idx];
  float sin_rot_correction = sin_rot_correction_[laser_idx];

  // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
  // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
  float cos_rot_angle = cos_rot_table_[azimuth] * cos_rot_correction + //
                        sin_rot_table_[azimuth] * sin_rot_correction;
  float sin_rot_angle = sin_rot_table_[azimuth] * cos_rot_correction - //
                        cos_rot_table_[azimuth] * sin_rot_correction;

  if (!apply_advanced_calibration_) {
    if (!distanceInRange(distance))
      return;

    float xy_distance = distance * cos_vert_angle;
    // Use the standard ROS coordinate system (x - forward, y - left, z - up)
    float x = xy_distance * cos_rot_angle;
    float y = -xy_distance * sin_rot_angle;
    float z = distance * sin_vert_angle;

    cloud.emplace_back(x, y, z, (float)measurement.intensity, ring_cache_[laser_idx], time);

  } else {
    const auto &corrections = calibration_.laser_corrections[laser_idx];
    distance += corrections.dist_correction;

    if (!distanceInRange(distance))
      return;

    float horiz_offset = corrections.horiz_offset_correction;
    float vert_offset  = corrections.vert_offset_correction;

    // Get 2points calibration values, Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    float distance_corr_x = 0;
    float distance_corr_y = 0;
    if (corrections.two_pt_correction_available) {
      // Compute the distance in the xy plane (w/o accounting for rotation)
      float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

      float xx = std::abs(xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle);
      float yy = std::abs(xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle);

      distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                            (xx - 2.4f) / (25.04f - 2.4f) +
                        corrections.dist_correction_x;
      distance_corr_x -= corrections.dist_correction;
      distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                            (yy - 1.93f) / (25.04f - 1.93f) +
                        corrections.dist_correction_y;
      distance_corr_y -= corrections.dist_correction;
    }

    float distance_x  = distance + distance_corr_x;
    float xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
    /// the expression with '-' is proved to be better than the one with '+'
    float x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

    float distance_y = distance + distance_corr_y;
    xy_distance      = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
    float y          = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

    // Using distance_y is not symmetric, but the velodyne manual does this.
    float z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

    /** Use standard ROS coordinate system */
    float x_coord = y;
    float y_coord = -x;
    float z_coord = z;

    /** Intensity Calculation */
    float min_intensity = corrections.min_intensity;
    float max_intensity = corrections.max_intensity;

    float focal_offset  = SQR(1 - corrections.focal_distance / 13100.f);
    float raw_intensity = measurement.intensity;
    float raw_distance  = measurement.distance;
    float intensity     = raw_intensity + 256 * corrections.focal_slope *
                                          std::abs(focal_offset - SQR(1 - raw_distance / 65535.f));
    intensity = (intensity < min_intensity) ? min_intensity : intensity;
    intensity = (intensity > max_intensity) ? max_intensity : intensity;

    cloud.emplace_back(x_coord, y_coord, z_coord, intensity, corrections.laser_ring, time);
  }
}

/** @brief convert raw VLS-128 / Alpha Prime packet to point cloud
 */
void PacketDecoder::unpack_vls128(const raw_packet_t &raw, Time stamp, PointCloud &cloud,
                                  Time scan_start_time) const {
  float time_diff_start_to_this_packet = stamp - scan_start_time;

  bool dual_return = (raw.return_mode == DualReturnMode::DUAL_RETURN);

  uint16_t azimuth_next   = raw.blocks[0].rotation;
  float last_azimuth_diff = 0;

  for (int block = 0; block < BLOCKS_PER_PACKET - (4 * dual_return); block++) {
    // cache block for use
    const raw_block_t &current_block = raw.blocks[block];

    int bank_origin = 0;
    // Used to detect which bank of 32 lasers is in this block
    switch (current_block.header) {
    case VLS128_BANK_1:
      bank_origin = 0;
      break;
    case VLS128_BANK_2:
      bank_origin = 32;
      break;
    case VLS128_BANK_3:
      bank_origin = 64;
      break;
    case VLS128_BANK_4:
      bank_origin = 96;
      break;
    default:
      return; // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    uint16_t azimuth = azimuth_next;
    float azimuth_diff;
    if (block < (BLOCKS_PER_PACKET - (1 + dual_return))) {
      // Get the next block rotation to calculate how far we rotate between blocks
      azimuth_next = raw.blocks[block + (1 + dual_return)].rotation;

      // Finds the difference between two successive blocks
      azimuth_diff = (float)((36000 + azimuth_next - azimuth) % 36000);

      // This is used when the last block is next to predict rotation amount
      last_azimuth_diff = azimuth_diff;
    } else {
      // This makes the assumption the difference between the last block and the next packet is the
      // same as the last to the second to last.
      // Assumes RPM doesn't change much between blocks
      azimuth_diff = (block == BLOCKS_PER_PACKET - (4 * dual_return) - 1) ? 0 : last_azimuth_diff;
    }

    if (!azimuthInRange(azimuth))
      continue;

    for (int j = 0; j < SCANS_PER_BLOCK; j++) {
      // distance extraction
      uint16_t raw_distance = current_block.data[j].distance;
      if (raw_distance == 0)
        continue;

      // Offset the laser in this block by which block it's in
      uint8_t laser_number = j + bank_origin;
      uint8_t firing_order = laser_number / 8; // VLS-128 fires 8 lasers at a time

      float time = 0;
      if (!timing_offsets_.empty()) {
        time = timing_offsets_[block / 4][firing_order + laser_number / 64] +
               time_diff_start_to_this_packet;
      }

      // correct for the laser rotation as a function of timing during the firings
      float azimuth_corrected_f =
          azimuth + azimuth_diff * vls_128_laser_azimuth_cache_[firing_order];
      uint16_t azimuth_corrected = std::lround(azimuth_corrected_f) % 36000;

      const auto &measurement = current_block.data[j];
      unpackPointCommon(cloud, laser_number, measurement, azimuth_corrected, time);
    }
  }
}

} // namespace velodyne_decoder
