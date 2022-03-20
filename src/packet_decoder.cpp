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
#include <fstream>
#include <limits>
#include <stdexcept>

namespace velodyne_decoder {
template <typename T> constexpr T SQR(T val) { return val * val; }
constexpr float nan = std::numeric_limits<float>::quiet_NaN();

PacketDecoder::PacketDecoder(const Config &config) : config_(config) {
  if (config_.model.empty()) {
    throw std::invalid_argument("No Velodyne sensor model specified!");
  }

  timing_offsets_ = buildTimings(config_.model);

  // get path to angles.config file for this device
  if (config_.calibration_file.empty()) {
    throw std::invalid_argument("Calibration config file not provided ");
  }

  calibration_.read(config_.calibration_file);
  if (!calibration_.initialized) {
    throw std::runtime_error("Unable to open calibration file: " + config_.calibration_file);
  }

  setupSinCosCache();
  setupAzimuthCache();
}

/** Update parameters: conversions and update */
void PacketDecoder::setParameters(double min_range, double max_range, double view_direction,
                                  double view_width) {
  config_.min_range = min_range;
  config_.max_range = max_range;

  // converting angle parameters into the velodyne reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep these angles within [0;2*M_PI]
  tmp_min_angle = fmod(fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = fmod(fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 performs a centered double to int conversion
  config_.min_angle = 100 * (2 * M_PI - tmp_min_angle) * 180 / M_PI + 0.5;
  config_.max_angle = 100 * (2 * M_PI - tmp_max_angle) * 180 / M_PI + 0.5;
  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

int PacketDecoder::scansPerPacket() const {
  if (calibration_.num_lasers == 16) {
    return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK * VLP16_SCANS_PER_FIRING;
  } else {
    return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
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
      vls_128_laser_azimuth_cache[i] =
          (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }
  }
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void PacketDecoder::unpack(const VelodynePacket &pkt, PointCloudAggregator &data,
                           Time scan_start_time) {
  /** special parsing for the VLS-128 and Alpha Prime **/
  if (pkt.data[1205] == VLS128_MODEL_ID) {
    unpack_vls128(pkt, data, scan_start_time);
    return;
  }

  /** special parsing for the VLP16 **/
  if (calibration_.num_lasers == 16) {
    unpack_vlp16(pkt, data, scan_start_time);
    return;
  }

  unpack_vlp32_vlp64(pkt, data, scan_start_time);
}

/** @brief convert raw VLP16 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void PacketDecoder::unpack_vlp16(const VelodynePacket &pkt, PointCloudAggregator &data,
                                 Time scan_start_time) const {
  auto *raw = reinterpret_cast<const raw_packet_t *>(&pkt.data[0]);

  float time_diff_start_to_this_packet = pkt.stamp - scan_start_time;

  float last_azimuth_diff = 0;

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    const auto &block = raw->blocks[i];

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != block.header) {
      return; // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    float azimuth      = block.rotation;
    float azimuth_diff = last_azimuth_diff;
    if (i < (BLOCKS_PER_PACKET - 1)) {
      int raw_azimuth_diff = raw->blocks[i + 1].rotation - block.rotation;
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
        /** correct for the laser rotation as a function of timing during the firings **/
        float azimuth_corrected_f =
            azimuth +
            (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) /
             VLP16_BLOCK_TDURATION);
        int azimuth_corrected = std::lround(azimuth_corrected_f) % 36000;

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if (!((config_.min_angle < config_.max_angle && (azimuth_corrected >= config_.min_angle &&
                                                         azimuth_corrected <= config_.max_angle)) ||
              (config_.min_angle > config_.max_angle &&
               (azimuth_corrected >= config_.min_angle || azimuth_corrected <= config_.max_angle))))
          continue;

        float time = 0;
        if (!timing_offsets_.empty())
          time = timing_offsets_[i][firing * 16 + dsr] + time_diff_start_to_this_packet;

        const auto &corrections = calibration_.laser_corrections[dsr];
        const auto &measurement = block.data[j];
        unpackPointCommon(data, corrections, measurement, azimuth_corrected, time);
      }
      data.newLine();
    }
  }
}

/** @brief convert raw VLP-32/64 packet to point cloud
 */
void PacketDecoder::unpack_vlp32_vlp64(const VelodynePacket &pkt, PointCloudAggregator &data,
                                       Time scan_start_time) const {
  auto *raw = reinterpret_cast<const raw_packet_t *>(&pkt.data[0]);

  float time_diff_start_to_this_packet = pkt.stamp - scan_start_time;

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    const auto &block = raw->blocks[i];

    // upper bank lasers are numbered [0..31]
    // lower bank lasers are [32..63]
    // NOTE: this is a change from the old velodyne_common implementation
    int bank_origin = (block.header == UPPER_BANK) ? 0 : 32;

    uint16_t azimuth = block.rotation;

    /*condition added to avoid calculating points which are not
      in the interesting defined area (min_angle < area < max_angle)*/
    if (!((config_.min_angle < config_.max_angle &&
           (azimuth >= config_.min_angle && azimuth <= config_.max_angle)) ||
          (config_.min_angle > config_.max_angle &&
           (azimuth >= config_.min_angle || azimuth <= config_.max_angle))))
      continue;

    for (int j = 0; j < SCANS_PER_BLOCK; j++) {
      const uint8_t laser_number = j + bank_origin;

      float time = 0;
      if (!timing_offsets_.empty()) {
        time = timing_offsets_[i][j] + time_diff_start_to_this_packet;
      }

      const auto &corrections = calibration_.laser_corrections[laser_number];
      const auto &measurement = block.data[j];
      unpackPointCommon(data, corrections, measurement, azimuth, time);
    }
    data.newLine();
  }
}

/** @brief Point decoding logic common to VLP-16/32/64
 */
void PacketDecoder::unpackPointCommon(PointCloudAggregator &data,
                                      const LaserCorrection &corrections,
                                      const raw_measurement_t &measurement, uint16_t azimuth,
                                      float time) const {
  /** Position Calculation */
  if (measurement.distance == 0) // no valid laser beam return
  {
    // call to addPoint is still required since output could be organized
    data.addPoint(nan, nan, nan, corrections.laser_ring, azimuth, nan, nan, time);
    return;
  }
  float distance = measurement.distance * calibration_.distance_resolution_m;
  distance += corrections.dist_correction;

  float cos_vert_angle     = corrections.cos_vert_correction;
  float sin_vert_angle     = corrections.sin_vert_correction;
  float cos_rot_correction = corrections.cos_rot_correction;
  float sin_rot_correction = corrections.sin_rot_correction;

  // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
  // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
  float cos_rot_angle = cos_rot_table_[azimuth] * cos_rot_correction + //
                        sin_rot_table_[azimuth] * sin_rot_correction;
  float sin_rot_angle = sin_rot_table_[azimuth] * cos_rot_correction - //
                        cos_rot_table_[azimuth] * sin_rot_correction;

  float horiz_offset = corrections.horiz_offset_correction;
  float vert_offset  = corrections.vert_offset_correction;

  // Compute the distance in the xy plane (w/o accounting for rotation)
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathematical
   * model we used.
   */
  float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

  // Calculate temporal X, use absolute value.
  float xx = std::abs(xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle);
  // Calculate temporal Y, use absolute value
  float yy = std::abs(xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle);

  // Get 2points calibration values,Linear interpolation to get distance
  // correction for X and Y, that means distance correction use
  // different value at different distance
  float distance_corr_x = 0;
  float distance_corr_y = 0;
  if (corrections.two_pt_correction_available) {
    distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) * (xx - 2.4f) /
                          (25.04f - 2.4f) +
                      corrections.dist_correction_x;
    distance_corr_x -= corrections.dist_correction;
    distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) * (yy - 1.93f) /
                          (25.04f - 1.93f) +
                      corrections.dist_correction_y;
    distance_corr_y -= corrections.dist_correction;
  }

  float distance_x = distance + distance_corr_x;
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathematical
   * model we used.
   */
  xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
  /// the expression with '-' is proved to be better than the one with '+'
  float x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

  float distance_y = distance + distance_corr_y;
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathematical
   * model we used.
   */
  xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
  float y     = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

  // Using distance_y is not symmetric, but the velodyne manual
  // does this.
  /**the new term of 'vert_offset * cos_vert_angle'
   * was added to the expression due to the mathematical
   * model we used.
   */
  float z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

  /** Use standard ROS coordinate system (right-hand rule) */
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

  data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth, distance, intensity,
                time);
}

/** @brief convert raw VLS-128 / Alpha Prime packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void PacketDecoder::unpack_vls128(const VelodynePacket &pkt, PointCloudAggregator &data,
                                  Time scan_start_time) const {
  auto *raw = reinterpret_cast<const raw_packet_t *>(&pkt.data[0]);

  float time_diff_start_to_this_packet = pkt.stamp - scan_start_time;

  bool dual_return = (pkt.data[1204] == 57);

  uint16_t azimuth_next   = raw->blocks[0].rotation;
  float last_azimuth_diff = 0;

  for (int block = 0; block < BLOCKS_PER_PACKET - (4 * dual_return); block++) {
    // cache block for use
    const raw_block_t &current_block = raw->blocks[block];

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
      azimuth_next = raw->blocks[block + (1 + dual_return)].rotation;

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

    // condition added to avoid calculating points which are not in the interesting defined area
    // (min_angle < area < max_angle)
    if (!((config_.min_angle < config_.max_angle &&
           (azimuth >= config_.min_angle && azimuth <= config_.max_angle)) ||
          (config_.min_angle > config_.max_angle &&
           (azimuth >= config_.min_angle || azimuth <= config_.max_angle))))
      continue;

    for (int j = 0; j < SCANS_PER_BLOCK; j++) {
      // distance extraction
      uint16_t raw_distance = current_block.data[j].distance;
      float distance        = raw_distance * VLS128_DISTANCE_RESOLUTION;

      if (pointInRange(distance)) {
        uint8_t laser_number =
            j + bank_origin; // Offset the laser in this block by which block it's in
        uint8_t firing_order = laser_number / 8; // VLS-128 fires 8 lasers at a time

        float time = 0;
        if (!timing_offsets_.empty()) {
          time = timing_offsets_[block / 4][firing_order + laser_number / 64] +
                 time_diff_start_to_this_packet;
        }

        const velodyne_decoder::LaserCorrection &corrections =
            calibration_.laser_corrections[laser_number];

        // correct for the laser rotation as a function of timing during the firings
        float azimuth_corrected_f =
            azimuth + (azimuth_diff * vls_128_laser_azimuth_cache[firing_order]);
        uint16_t azimuth_corrected = std::lround(azimuth_corrected_f) % 36000;

        // convert polar coordinates to Euclidean XYZ
        float cos_vert_angle     = corrections.cos_vert_correction;
        float sin_vert_angle     = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle = cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
        float sin_rot_angle = sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                              cos_rot_table_[azimuth_corrected] * sin_rot_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        float xy_distance = distance * cos_vert_angle;

        data.addPoint(xy_distance * cos_rot_angle, -(xy_distance * sin_rot_angle),
                      distance * sin_vert_angle, corrections.laser_ring, azimuth_corrected,
                      distance, current_block.data[j].intensity, time);
      }
    }
    data.newLine();
  }
}

} // namespace velodyne_decoder
