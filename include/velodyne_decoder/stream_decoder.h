// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/scan_batcher.h"
#include "velodyne_decoder/scan_decoder.h"
#include "velodyne_decoder/types.h"

#include <optional>
#include <utility>

namespace velodyne_decoder {

class StreamDecoder {
public:
  explicit StreamDecoder(const Config &config);

  std::optional<std::pair<TimePair, PointCloud>> decode(Time packet_stamp,
                                                        const RawPacketData &packet);
  std::optional<std::pair<TimePair, PointCloud>> decode(const VelodynePacket &packet);

  /**
   * @brief Decodes all remaining packets in the buffer and returns the last scan, if any.
   */
  std::optional<std::pair<TimePair, PointCloud>> finish();

private:
  std::pair<TimePair, PointCloud> decodeCollectedPackets();

private:
  ScanBatcher<> scan_batcher_;
  ScanDecoder scan_decoder_;
};

} // namespace velodyne_decoder
