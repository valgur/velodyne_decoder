// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <ros/time.h>
#include <velodyne_decoder/time_conversion.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <memory>
#include <string>

namespace velodyne_decoder {
inline PacketView toPacketView(const velodyne_msgs::VelodynePacket &packet_msg) {
  return {packet_msg.stamp.toSec(), gsl::span<const uint8_t, PACKET_SIZE>{packet_msg.data}};
}
} // namespace velodyne_decoder

#include <velodyne_decoder/config.h>
#include <velodyne_decoder/scan_batcher.h>

namespace velodyne_decoder {

class RosScanBatcher {
public:
  using Base = ScanBatcher<velodyne_msgs::VelodynePacket>;

  inline explicit RosScanBatcher(const Config &config, const std::string frame_id)
      : frame_id_(frame_id), timestamp_first_packet_(config.timestamp_first_packet),
        scan_batcher_(config) {
    reset();
  }

  inline void reset() {
    scan_msg_ = boost::make_shared<velodyne_msgs::VelodyneScan>();
    scan_batcher_.reset({&scan_msg_->packets, [](auto *) {}});
  }

  inline bool push(const velodyne_msgs::VelodynePacket &packet) {
    return scan_batcher_.push(packet);
  }

  [[nodiscard]] inline bool scanComplete() const { return scan_batcher_.scanComplete(); }

  [[nodiscard]] inline const boost::shared_ptr<velodyne_msgs::VelodyneScan> &scanMsg() const {
    scan_msg_->header.frame_id = frame_id_;
    scan_msg_->header.stamp    = timestamp_first_packet_ ? scan_msg_->packets.front().stamp
                                                         : scan_msg_->packets.back().stamp;
    return scan_msg_;
  }

private:
  std::string frame_id_;
  bool timestamp_first_packet_;
  ScanBatcher<velodyne_msgs::VelodynePacket> scan_batcher_;
  boost::shared_ptr<velodyne_msgs::VelodyneScan> scan_msg_;
};

} // namespace velodyne_decoder
