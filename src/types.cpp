#include "velodyne_decoder/types.h"
#include "velodyne_decoder/time_conversion.h"

#include <algorithm>

namespace velodyne_decoder {

TimePair::TimePair(Time host, Time device) : host(host), device(device) {}

TimePair::TimePair(Time host_stamp, gsl::span<const uint8_t, PACKET_SIZE> data)
    : host(host_stamp), device(getPacketTimestamp(data, host_stamp)) {}

VelodynePacket::VelodynePacket(TimePair stamp, gsl::span<const uint8_t, PACKET_SIZE> data)
    : stamp(stamp), data() {
  std::copy(data.begin(), data.end(), this->data.begin());
}

VelodynePacket::VelodynePacket(Time host_stamp, gsl::span<const uint8_t, PACKET_SIZE> data)
    : VelodynePacket(TimePair(host_stamp, data), data) {}

PacketView::PacketView(TimePair stamp, gsl::span<const uint8_t, PACKET_SIZE> data)
    : stamp(stamp), data(data) {}

PacketView::PacketView(Time host_stamp, gsl::span<const uint8_t, PACKET_SIZE> data)
    : stamp(host_stamp, data), data(data) {}

PacketView::PacketView(const VelodynePacket &packet) : stamp(packet.stamp), data(packet.data) {}

PointXYZIRT::PointXYZIRT(float x, float y, float z, float intensity, uint16_t ring, float time)
    : x(x), y(y), z(z), intensity(intensity), ring(ring), time(time) {}
} // namespace velodyne_decoder