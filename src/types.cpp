#include "velodyne_decoder/types.h"
#include "velodyne_decoder/time_conversion.h"

namespace velodyne_decoder {

VelodynePacket::VelodynePacket(TimePair stamp, const RawPacketData &data)
    : stamp(stamp), data(data) {}

VelodynePacket::VelodynePacket(Time host_stamp, const RawPacketData &data)
    : stamp({host_stamp, getPacketTimestamp(data, host_stamp)}), data(data) {}

VelodyneScan::VelodyneScan(TimePair stamp, std::vector<VelodynePacket> packets)
    : stamp(stamp), packets(std::move(packets)) {}

PointXYZIRT::PointXYZIRT(float x, float y, float z, float intensity, uint16_t ring, float time)
    : x(x), y(y), z(z), intensity(intensity), ring(ring), time(time) {}

} // namespace velodyne_decoder