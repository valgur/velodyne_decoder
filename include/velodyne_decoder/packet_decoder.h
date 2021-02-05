// Copyright (C) 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
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
/**
 * Raw Velodyne packet constants and structures.
 */
constexpr int SIZE_BLOCK      = 100;
constexpr int RAW_SCAN_SIZE   = 3;
constexpr int SCANS_PER_BLOCK = 32;
constexpr int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

constexpr float ROTATION_RESOLUTION   = 0.01f;  // [deg]
constexpr uint16_t ROTATION_MAX_UNITS = 36000u; // [deg/100]

/** @todo make this work for both big and little-endian machines */
constexpr uint16_t UPPER_BANK = 0xeeff;
constexpr uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
constexpr int VLP16_FIRINGS_PER_BLOCK = 2;
constexpr int VLP16_SCANS_PER_FIRING  = 16;
constexpr float VLP16_BLOCK_TDURATION = 110.592f; // [µs]
constexpr float VLP16_DSR_TOFFSET     = 2.304f;   // [µs]
constexpr float VLP16_FIRING_TOFFSET  = 55.296f;  // [µs]

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
struct raw_block_t {
  uint16_t header;   ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation; ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
};

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};

constexpr int PACKET_SIZE        = 1206;
constexpr int BLOCKS_PER_PACKET  = 12;
constexpr int PACKET_STATUS_SIZE = 4;
constexpr int SCANS_PER_PACKET   = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** Special Definitions for VLS128 support **/
// These are used to detect which bank of 32 lasers is in this block
constexpr uint16_t VLS128_BANK_1 = 0xeeff;
constexpr uint16_t VLS128_BANK_2 = 0xddff;
constexpr uint16_t VLS128_BANK_3 = 0xccff;
constexpr uint16_t VLS128_BANK_4 = 0xbbff;

constexpr float VLS128_CHANNEL_TDURATION = 2.665f; // [µs] Channels corresponds to one laser firing
constexpr float VLS128_SEQ_TDURATION =
    53.3f; // [µs] Sequence is a set of laser firings including recharging
constexpr float VLS128_TOH_ADJUSTMENT =
    8.7f; // [µs] μs. Top Of the Hour is aligned with the fourth firing group in a firing sequence.
constexpr float VLS128_DISTANCE_RESOLUTION = 0.004f; // [m]
constexpr float VLS128_MODEL_ID            = 161;

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
struct raw_packet_t {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
};

/** \brief Velodyne data conversion class */
class PacketDecoder {
public:
  explicit PacketDecoder(const Config &config);

  void unpack(const VelodynePacket &pkt, PointCloudAggregator &data, Time scan_start_time);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  int scansPerPacket() const;

  /** configuration parameters */
  Config config_;

  /**
   * Calibration file
   */
  velodyne_decoder::Calibration calibration_;

protected:
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // Caches the azimuth percent offset for the VLS-128 laser firings
  float vls_128_laser_azimuth_cache[16];

  // timing offset lookup table
  std::vector<std::vector<float>> timing_offsets;

  /** \brief setup per-point timing offsets
   *
   *  Runs during initialization and determines the firing time for each point in the scan
   */
  static std::vector<std::vector<float>> buildTimings(const std::string &model);

  void setupSinCosCache();
  void setupAzimuthCache();

  /** add private function to handle the VLP16 **/
  void unpack_vlp16(const VelodynePacket &pkt, PointCloudAggregator &data, Time scan_start_time);

  void unpack_vls128(const VelodynePacket &pkt, PointCloudAggregator &data, Time scan_start_time);

  /** in-line test whether a point is in range */
  constexpr bool pointInRange(float range) const {
    return range >= config_.min_range && range <= config_.max_range;
  }
};

} // namespace velodyne_decoder
