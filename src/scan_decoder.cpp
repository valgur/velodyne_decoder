/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 */

#include "velodyne_decoder/scan_decoder.h"

namespace velodyne_decoder {

ScanDecoder::ScanDecoder(const Config &config)
    : packet_decoder_(config),
      cloud_aggregator_(config.max_range, config.min_range, packet_decoder_.scansPerPacket()) {}

PointCloud ScanDecoder::processScan(const VelodyneScan &scan) {
  cloud_aggregator_.init(scan);
  for (const auto &packet : scan.packets) {
    packet_decoder_.unpack(packet, cloud_aggregator_, scan.stamp);
  }
  return cloud_aggregator_.cloud;
}

} // namespace velodyne_decoder
