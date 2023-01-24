#include "velodyne_decoder/position_packet.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

namespace velodyne_decoder {

namespace {
bool validate_nmea_checksum(const std::string &nmea) {
  if (nmea.size() < 5)
    return false;
  size_t body_len  = nmea.size() - 5;
  uint8_t checksum = 0;
  for (size_t i = 1; i < body_len; i++)
    checksum ^= nmea[i];
  return checksum == strtoul(&nmea[body_len + 1], nullptr, 16);
}

/// Parse NMEA sentence
std::optional<NmeaInfo> parse_nmea(const std::string &nmea) {
  if (strncmp(nmea.c_str(), "$GPGGA,", 6) == 0) {
    if (!validate_nmea_checksum(nmea))
      return std::nullopt;
    NmeaInfo info = {};
    // https://gpsd.gitlab.io/gpsd/NMEA.html#_gga_global_positioning_system_fix_data
    uint8_t lat_deg, lon_deg;
    double lat_min, lon_min;
    char lat_sign, lon_sign;
    uint8_t quality;
    double altitude_msl, geoid_separation;
    if (sscanf(nmea.c_str(),
               "$GPGGA,"       //
               "%2hhd%2hhd%f," // UTC time
               "%2hhd%lf,%c,"  // Latitude
               "%3hhd%lf,%c,"  // Longitude
               "%hhd,"         // Quality
               "%*[^,],"       // Number of satellites
               "%*[^,],"       // Horizontal dilution of precision
               "%lf,M,"        // Altitude above mean sea level
               "%lf,M,",       // Height of geoid
               &info.utc_hours, &info.utc_minutes, &info.utc_seconds, //
               &lat_deg, &lat_min, &lat_sign,                         //
               &lon_deg, &lon_min, &lon_sign,                         //
               &quality,                                              //
               &altitude_msl, &geoid_separation) != 12)
      return std::nullopt;
    info.longitude     = (lon_deg + lon_min / 60.0) * (lon_sign == 'E' ? 1 : -1);
    info.latitude      = (lat_deg + lat_min / 60.0) * (lat_sign == 'N' ? 1 : -1);
    info.altitude      = altitude_msl + geoid_separation;
    info.fix_available = quality > 0;
    // date info is not available in GPGGA, leaving DMY as 0
    return info;

  } else if (strncmp(nmea.c_str(), "$GPRMC,", 6) == 0) {
    if (!validate_nmea_checksum(nmea))
      return std::nullopt;
    NmeaInfo info = {};
    // https://gpsd.gitlab.io/gpsd/NMEA.html#_rmc_recommended_minimum_navigation_information
    char status;
    uint8_t lat_deg, lon_deg;
    double lat_min, lon_min;
    char lat_sign, lon_sign;
    if (sscanf(nmea.c_str(),
               "$GPRMC,"                                              //
               "%2hhd%2hhd%f,"                                        // UTC time
               "%c,"                                                  // Status
               "%2hhd%lf,%c,"                                         // Latitude
               "%3hhd%lf,%c,"                                         // Longitude
               "%*[^,],"                                              // Speed over ground
               "%*[^,],"                                              // Track made good
               "%2hhd%2hhd%2hd",                                      // Date
               &info.utc_hours, &info.utc_minutes, &info.utc_seconds, //
               &status,                                               //
               &lat_deg, &lat_min, &lat_sign,                         //
               &lon_deg, &lon_min, &lon_sign,                         //
               &info.utc_day, &info.utc_month, &info.utc_year) != 13)
      return std::nullopt;
    info.longitude = (lon_deg + lon_min / 60.0) * (lon_sign == 'E' ? 1 : -1);
    info.latitude  = (lat_deg + lat_min / 60.0) * (lat_sign == 'N' ? 1 : -1);
    // altitude is not available in GPRMC, leaving it as 0
    info.fix_available = status == 'A';
    if (info.utc_year > 0)
      info.utc_year += 2000;
    return info;
  }
  return std::nullopt;
}

} // namespace

PositionPacket::PositionPacket(const std::array<uint8_t, POSITION_PACKET_SIZE> &data) {
  temp_board_top                    = data[0xBB];
  temp_board_bottom                 = data[0xBC];
  temp_during_adc_calibration       = data[0xBD];
  temp_change_since_adc_calibration = static_cast<int16_t>(data[0xBF] << 8 | data[0xBE]);
  seconds_since_adc_calibration =
      static_cast<uint32_t>(data[0xC3] << 24u | data[0xC2] << 16u | data[0xC1] << 8u | data[0xC0]);
  adc_calibration_reason        = static_cast<AdcCalibReason>(data[0xC4]);
  adc_calib_in_progress         = data[0xC5] & 1u;
  adc_delta_temp_limit_exceeded = data[0xC5] & 2u;
  adc_period_exceeded           = data[0xC5] & 4u;
  seconds_since_toh =
      static_cast<double>(data[0xC9] << 24u | data[0xC8] << 16u | data[0xC7] << 8u | data[0xC6]) *
      1e-6;
  pps_status       = static_cast<PpsStatus>(data[0xCA]);
  thermal_shutdown = data[0xCB] == 1;
  temp_at_shutdown = data[0xCC];
  temp_at_powerup  = data[0xCD];
  nmea_sentence.assign(&data[0xCE], std::find(&data[0xCE], &data[0xCE] + 128, '\0'));
}

std::optional<NmeaInfo> PositionPacket::parseNmea() const { return parse_nmea(nmea_sentence); }

} // namespace velodyne_decoder
