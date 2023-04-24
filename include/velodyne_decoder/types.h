// Copyright (c) 2007-2019, Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
// Copyright (c) 2021-2023, Martin Valgur
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

constexpr int PACKET_SIZE       = 1206;
constexpr int BLOCKS_PER_PACKET = 12;

constexpr int TELEMETRY_PACKET_SIZE = 512;

// These are used to detect which bank of 32 lasers is contained in this block
enum class LaserBankId : uint16_t {
  BANK_0 = 0xeeff, // lasers [0..31], aka upper bank for HDL-64E
  BANK_1 = 0xddff, // lasers [32..63], aka lower bank for HDL-64E
  BANK_2 = 0xccff, // lasers [64..95]
  BANK_3 = 0xbbff, // lasers [96..127]
};

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

enum class PacketModelId : uint8_t {
  HDL32E     = 0x21, // decimal: 33
  VLP16      = 0x22, // decimal: 34
  VLP32AB    = 0x23, // decimal: 35
  VLP16HiRes = 0x24, // decimal: 36
  VLP32C     = 0x28, // decimal: 40
  Velarray   = 0x31, // decimal: 49
  VLS128     = 0xa1, // decimal: 161
};

enum class DualReturnMode : uint8_t {
  STRONGEST_RETURN = 0x37, // decimal: 55
  LAST_RETURN      = 0x38, // decimal: 56
  DUAL_RETURN      = 0x39, // decimal: 57
};

// Offset added to ring values depending on the type of point
enum ReturnModeFlag : uint16_t {
  SINGLE_RETURN_FLAG = 0, // point is from single-return mode (if single_return_mode_info is false)
  BOTH_RETURN_FLAG   = 0, // point is both the last and strongest one
  STRONGEST_RETURN_FLAG = 1024, // the strongest point in firing
  LAST_RETURN_FLAG      = 2048, // the last point in firing
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
  LaserBankId bank_id; ///< which bank of 32 lasers is contained in this block
  uint16_t rotation;   ///< 0-35999, divide by 100 to get degrees
  std::array<raw_measurement_t, SCANS_PER_BLOCK> data;
};

struct raw_packet_t {
  std::array<raw_block_t, BLOCKS_PER_PACKET> blocks;
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
