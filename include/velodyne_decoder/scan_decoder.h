// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <string>

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/packet_decoder.h"
#include "velodyne_decoder/types.h"

namespace velodyne_decoder {

class ScanDecoder {
public:
  explicit ScanDecoder(const Config &config);

  PointCloud decode(const VelodyneScan &scan);
  PointCloud decode(Time scan_stamp, const std::vector<VelodynePacket> &scan_packets);

  velodyne_decoder::PacketDecoder packet_decoder_;
  velodyne_decoder::PointCloud cloud_;
};

} // namespace velodyne_decoder
