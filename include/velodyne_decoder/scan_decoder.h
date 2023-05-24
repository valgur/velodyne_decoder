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

  std::pair<TimePair, PointCloud> decode(const std::vector<VelodynePacket> &scan_packets);
  std::pair<TimePair, PointCloud> decode(const std::vector<PacketView> &scan_packets);

  /// Detected or configured model ID of the sensor
  [[nodiscard]] std::optional<ModelId> modelId() const;

  /// The return mode of the sensor based on the last received packet
  [[nodiscard]] std::optional<DualReturnMode> returnMode() const;

private:
  velodyne_decoder::PacketDecoder packet_decoder_;
  velodyne_decoder::PointCloud cloud_;

  bool timestamp_first_packet_;
};

} // namespace velodyne_decoder
