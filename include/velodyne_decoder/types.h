#pragma once

#include <array>
#include <vector>

namespace velodyne_decoder {

using Time = double;

struct VelodynePacket {
  Time stamp;
  std::array<uint8_t, 1206> data;
};

struct VelodyneScan {
  Time stamp;
  std::vector<VelodynePacket> packets;
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
