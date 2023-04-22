// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "velodyne_decoder/types.h"

namespace velodyne_decoder {

struct NmeaInfo {
  double longitude    = 0;     ///< Longitude in degrees
  double latitude     = 0;     ///< Latitude in degrees
  double altitude     = 0;     ///< Altitude above WGS84 ellipsoid in meters (always 0 for GPRMC)
  uint16_t utc_year   = 0;     ///< UTC year (always 0 for GPGGA)
  uint8_t utc_month   = 0;     ///< UTC month (always 0 for GPGGA)
  uint8_t utc_day     = 0;     ///< UTC day (always 0 for GPGGA)
  uint8_t utc_hours   = 0;     ///< UTC hours
  uint8_t utc_minutes = 0;     ///< UTC minutes
  float utc_seconds   = 0;     ///< UTC seconds
  bool fix_available  = false; ///< Position fix available

  /// UTC timestamp as Unix time, or seconds since midnight if date is missing
  [[nodiscard]] Time utcTime() const;
};

struct TelemetryPacket {
  /// Reason for the last ADC calibration
  enum class AdcCalibReason : uint8_t {
    NO_CALIBRATION    = 0, ///< No calibration
    POWER_ON          = 1, ///< Power-on calibration performed
    MANUAL            = 2, ///< Manual calibration performed
    DELTA_TEMPERATURE = 3, ///< Delta temperature calibration performed
    PERIODIC          = 4, ///< Periodic calibration performed
  };
  /// Pulse Per Second (PPS) status
  enum class PpsStatus : uint8_t {
    ABSENT        = 0, ///< No PPS detected
    SYNCHRONIZING = 1, ///< Synchronizing to PPS
    LOCKED        = 2, ///< PPS Locked
    ERROR         = 3, ///< Error
  };

  // fields set by all firmware versions
  uint32_t usec_since_toh;   ///< Number of microseconds elapsed since the top of the hour
  PpsStatus pps_status;      ///< Pulse Per Second (PPS) status
  std::string nmea_sentence; ///< GPRMC or GPGGA NMEA sentence

  // fields set by newer firmware versions only (since mid-2018)
  uint8_t temp_board_top;              ///< Temperature of top board, 0 to 150 °C
  uint8_t temp_board_bottom;           ///< Temperature of bottom board, 0 to 150 °C
  bool thermal_shutdown;               ///< Thermal status, true if thermal shutdown
  uint8_t temp_at_shutdown;            ///< Temperature of unit when thermal shutdown occurred
  uint8_t temp_at_powerup;             ///< Temperature of unit (bottom board) at power up
  uint8_t temp_during_adc_calibration; ///< Temperature when ADC calibration last ran, 0 to 150 °C
  int16_t temp_change_since_adc_calibration; ///< Change in temperature since last ADC calibration,
                                             ///< -150 to 150°C
  uint32_t seconds_since_adc_calibration;    ///< Elapsed seconds since last ADC calibration
  AdcCalibReason adc_calibration_reason;     ///< Reason for the last ADC calibration
  bool adc_calib_in_progress;                ///< ADC calibration in progress
  bool adc_delta_temp_limit_exceeded; ///< ADC calibration: delta temperature limit has been met
  bool adc_period_exceeded;           ///< ADC calibration: periodic time elapsed limit has been met

  explicit TelemetryPacket(const std::array<uint8_t, TELEMETRY_PACKET_SIZE> &raw_data);

  [[nodiscard]] std::optional<NmeaInfo> parseNmea() const;

  /// Timestamp from the NMEA sentence
  [[nodiscard]] std::optional<Time> nmeaTime() const;

  /// Timestamp from the PPS signal combined with the NMEA timestamp
  [[nodiscard]] std::optional<Time> ppsTime() const;
};

} // namespace velodyne_decoder
