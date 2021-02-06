#pragma once

#include <array>
#include <utility>
#include <vector>

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

using Time = double;

struct VelodynePacket {
  Time stamp;
  std::array<uint8_t, PACKET_SIZE> data;

  VelodynePacket() = default;
  VelodynePacket(Time stamp, const std::array<uint8_t, PACKET_SIZE> &data)
      : stamp(stamp), data(data) {}
};

struct VelodyneScan {
  Time stamp;
  std::vector<VelodynePacket> packets;

  VelodyneScan() = default;
  VelodyneScan(Time stamp, std::vector<VelodynePacket> packets)
      : stamp(stamp), packets(std::move(packets)) {}
};

struct PointXYZIRT {
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  float time;
};

using PointCloud = std::vector<PointXYZIRT>;

} // namespace velodyne_decoder
