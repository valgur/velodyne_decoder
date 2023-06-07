// Copyright (c) 2007-2019, Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <gsl/span>
#include <utility>
#include <vector>

namespace velodyne_decoder {

constexpr size_t PACKET_SIZE           = 1206;
constexpr size_t TELEMETRY_PACKET_SIZE = 512;

using Time = double;
struct TimePair {
  /// Time of arrival of the packet at the host machine.
  Time host;
  /// Timestamp of the packet as reported by the device.
  /// Relies on the host time to convert relative top-of-hour timestamps to absolute timestamps.
  Time device;

  TimePair() = default;
  TimePair(Time host, Time device);
  TimePair(Time host_stamp, gsl::span<const uint8_t, PACKET_SIZE> data);
};

using RawPacketData = std::array<uint8_t, PACKET_SIZE>;

struct VelodynePacket {
  TimePair stamp;
  RawPacketData data;

  VelodynePacket() = default;
  VelodynePacket(TimePair stamp, gsl::span<const uint8_t, PACKET_SIZE> data);
  VelodynePacket(Time host_stamp, gsl::span<const uint8_t, PACKET_SIZE> data);
};

struct PacketView {
  TimePair stamp;
  gsl::span<const uint8_t, PACKET_SIZE> data;

  PacketView(TimePair stamp, gsl::span<const uint8_t, PACKET_SIZE> data);
  PacketView(Time host_stamp, gsl::span<const uint8_t, PACKET_SIZE> data);
  PacketView(const VelodynePacket &packet);
};

enum class ReturnMode : uint8_t {
  STRONGEST = 1,     // the strongest point in firing
  LAST      = 2,     // the last point in firing
  BOTH      = 1 | 2, // point is both the last and strongest one
};
ReturnMode operator|(ReturnMode lhs, ReturnMode rhs);
ReturnMode operator&(ReturnMode lhs, ReturnMode rhs);

struct alignas(16) VelodynePoint {
  struct alignas(16) {
    float x;
    float y;
    float z;
    float intensity;
  };
  uint8_t ring;
  float time;
  ReturnMode return_type;

  VelodynePoint() = default;
  VelodynePoint(float x, float y, float z, float intensity, uint8_t ring, float time,
                ReturnMode return_type);
};
static_assert(sizeof(VelodynePoint) == 32, "VelodynePoint is not 32 bytes");
using PointCloud = std::vector<VelodynePoint>;

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

enum class PacketReturnMode : uint8_t {
  STRONGEST = 0x37, // decimal: 55
  LAST      = 0x38, // decimal: 56
  DUAL      = 0x39, // decimal: 57
};

/**
 * Constants and structures specific to raw Velodyne packet structure.
 */
constexpr int SIZE_BLOCK              = 100;
constexpr int SCANS_PER_BLOCK         = 32;
constexpr int BLOCKS_PER_PACKET       = 12;
constexpr float ROTATION_RESOLUTION   = 0.01f;  // [deg]
constexpr uint16_t ROTATION_MAX_UNITS = 36000u; // [deg/100]

// These are used to detect which bank of 32 lasers is contained in this block
enum class LaserBankId : uint16_t {
  BANK_0 = 0xeeff, // lasers [0..31], aka upper bank for HDL-64E
  BANK_1 = 0xddff, // lasers [32..63], aka lower bank for HDL-64E
  BANK_2 = 0xccff, // lasers [64..95]
  BANK_3 = 0xbbff, // lasers [96..127]
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
  PacketReturnMode return_mode;
  PacketModelId model_id;
  // In HDL-64E, the last bytes have a different meaning, but we are ignoring these.
  // uint16_t revolution;
  // uint8_t status[4];
};
#pragma pack(pop)

} // namespace velodyne_decoder
