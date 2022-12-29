// Copyright (C) 2007-2021 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <utility>
#include <vector>

namespace velodyne_decoder {

/**
 * Raw Velodyne packet constants and structures.
 */
constexpr int SIZE_BLOCK      = 100;
constexpr int SCANS_PER_BLOCK = 32;

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

constexpr int PACKET_SIZE       = 1206;
constexpr int BLOCKS_PER_PACKET = 12;
constexpr int SCANS_PER_PACKET  = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** Special Definitions for VLS-128 / Alpha Prime support **/
// These are used to detect which bank of 32 lasers is in this block
constexpr uint16_t VLS128_BANK_1 = 0xeeff;
constexpr uint16_t VLS128_BANK_2 = 0xddff;
constexpr uint16_t VLS128_BANK_3 = 0xccff;
constexpr uint16_t VLS128_BANK_4 = 0xbbff;

// [µs] Channels corresponds to one laser firing
constexpr float VLS128_CHANNEL_TDURATION = 2.665f;
// [µs] Sequence is a set of laser firings including recharging
constexpr float VLS128_SEQ_TDURATION = 53.3f;

enum class ModelId : uint8_t {
  HDL64E_S1  = 1,
  HDL64E_S2  = 2,
  HDL64E_S3  = 3,
  HDL32E     = 4,
  VLP32A     = 5,
  VLP32B     = 6,
  VLP32C     = 7,
  VLP16      = 8,
  PuckLite   = 8, // = VLP-16
  PuckHiRes  = 9, // aka VLP-16 Hi-Res
  VLS128     = 10,
  AlphaPrime = 10, // = VLS-128
};

enum class DualReturnMode : uint8_t {
  STRONGEST_RETURN            = 0x37,
  LAST_RETURN                 = 0x38,
  DUAL_RETURN                 = 0x39,
  TRIPLE_RETURN               = 0x3A,
  DUAL_RETURN_WITH_CONFIDENCE = 0x3B,
};

enum class PacketModelId : uint8_t {
  HDL32E     = 0x21, // decimal: 33
  VLP16      = 0x22, // decimal: 34
  VLP32AB    = 0x23, // decimal: 35
  VLP16HiRes = 0x24, // decimal: 36
  VLP32C     = 0x28, // decimal: 40
  Velarray   = 0x31, // decimal: 49
  VLS128     = 0xa1, // decimal: 161
};

#pragma pack(push, 1)
struct raw_measurement_t {
  uint16_t distance;
  uint8_t intensity;
};

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 */
struct raw_block_t {
  uint16_t header;   ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation; ///< 0-35999, divide by 100 to get degrees
  raw_measurement_t data[SCANS_PER_BLOCK];
};

struct raw_packet_t {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint32_t stamp;
  DualReturnMode return_mode;
  PacketModelId model_id;
  // In HDL-64E, the last bytes have a different meaning, but we are ignoring these.
  // uint16_t revolution;
  // uint8_t status[4];
};
#pragma pack(pop)

using Time          = double;
using RawPacketData = std::array<uint8_t, PACKET_SIZE>;

struct VelodynePacket {
  Time stamp;
  RawPacketData data;

  VelodynePacket() = default;
  VelodynePacket(Time stamp, const RawPacketData &data) : stamp(stamp), data(data) {}
};

struct VelodyneScan {
  Time stamp;
  std::vector<VelodynePacket> packets;

  VelodyneScan() = default;
  VelodyneScan(Time stamp, std::vector<VelodynePacket> packets)
      : stamp(stamp), packets(std::move(packets)) {}
};

struct alignas(16) PointXYZIRT {
  struct alignas(16) {
    float x;
    float y;
    float z;
  };
  float intensity;
  uint16_t ring;
  float time;

  PointXYZIRT() = default;
  PointXYZIRT(float x, float y, float z, float intensity, uint16_t ring, float time)
      : x(x), y(y), z(z), intensity(intensity), ring(ring), time(time) {}
};

using PointCloud = std::vector<PointXYZIRT>;

} // namespace velodyne_decoder
