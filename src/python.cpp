// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/scan_decoder.h"
#include "velodyne_decoder/stream_decoder.h"
#include "velodyne_decoder/telemetry_packet.h"
#include "velodyne_decoder/types.h"

namespace nb = nanobind;
using namespace nanobind::literals;
using namespace velodyne_decoder;

constexpr int ncols   = 8;
using NumpyPointCloud = nb::ndarray<float, nb::numpy, nb::shape<-1, ncols>, nb::c_contig>;

nb::dict get_point_dtype() {
  nb::dict dtype_dict;
  dtype_dict["names"] =
      nb::make_tuple("x", "y", "z", "intensity", "time", "column", "ring", "return_type");
  dtype_dict["formats"]  = nb::make_tuple("<f4", "<f4", "<f4", "<f4", "<f4", "<u2", "u1", "u1");
  dtype_dict["offsets"]  = nb::make_tuple(0, 4, 8, 12, 16, 20, 22, 23);
  dtype_dict["itemsize"] = 32;
  return dtype_dict;
}

nb::object as_recarray(PointCloud &&cloud) {
  const size_t n = cloud.size();
  auto tmp       = new PointCloud;
  tmp->swap(cloud);
  nb::capsule deleter(tmp, [](void *p) noexcept { delete static_cast<PointCloud *>(p); });
  nb::object arr    = nb::cast(NumpyPointCloud{tmp->data(), {n, ncols}, deleter});
  arr.attr("dtype") = get_point_dtype();
  arr.attr("shape") = nb::make_tuple(n);
  return arr;
}

NumpyPointCloud as_contiguous(const PointCloud &cloud) {
  const size_t n = cloud.size();
  float *arr     = new float[n * ncols];
  nb::capsule deleter(arr, [](void *p) noexcept { delete static_cast<float *>(p); });
  NumpyPointCloud py_arr = {arr, {n, ncols}, deleter};
  for (size_t i = 0; i < n; i++) {
    const auto &p = cloud[i];
    py_arr(i, 0)  = p.x;
    py_arr(i, 1)  = p.y;
    py_arr(i, 2)  = p.z;
    py_arr(i, 3)  = p.intensity;
    py_arr(i, 4)  = p.time;
    py_arr(i, 5)  = static_cast<float>(p.column);
    py_arr(i, 6)  = static_cast<float>(p.ring);
    py_arr(i, 7)  = static_cast<float>(p.return_type);
  }
  return py_arr;
}

nb::object convert(PointCloud &&cloud, bool as_pcl_structs) {
  if (as_pcl_structs) {
    return as_recarray(std::move(cloud));
  }
  return nb::cast(as_contiguous(cloud));
}

nb::object to_datetime(Time time) {
  static auto fromtimestamp =
      nb::module_::import_("datetime").attr("datetime").attr("fromtimestamp");
  return fromtimestamp(time);
}

nb::object convert_gps_time(std::optional<Time> time) {
  if (!time)
    return nb::none();
  if (*time < 1e5) // a full datetime is unavailable, only time of day
    return to_datetime(*time).attr("time")();
  return to_datetime(*time);
}

NB_MAKE_OPAQUE(std::vector<VelodynePacket>);

NB_MODULE(velodyne_decoder_pylib, m) {
  m.doc() = "";

  nb::class_<Config>(m, "Config")
      .def(
          "__init__",
          [](Config *t, std::optional<ModelId> model, const std::optional<Calibration> &calibration,
             float min_range, float max_range, float min_angle, float max_angle,
             std::optional<float> cut_angle, bool timestamp_first_packet) {
            auto *cfg                   = new (t) Config;
            cfg->model                  = model;
            cfg->calibration            = calibration;
            cfg->min_range              = min_range;
            cfg->max_range              = max_range;
            cfg->min_angle              = min_angle;
            cfg->max_angle              = max_angle;
            cfg->timestamp_first_packet = timestamp_first_packet;
            cfg->cut_angle              = cut_angle;
          },
          nb::kw_only(),                                  //
          nb::arg("model")                  = nb::none(), //
          nb::arg("calibration")            = nb::none(), //
          nb::arg("min_range")              = 0.1,        //
          nb::arg("max_range")              = 200,        //
          nb::arg("min_angle")              = 0,          //
          nb::arg("max_angle")              = 360,        //
          nb::arg("cut_angle")              = nb::none(), //
          nb::arg("timestamp_first_packet") = false       //
          )
      .def_prop_rw(
          "model", [](const Config &cfg) { return cfg.model; },
          [](Config &cfg, std::optional<ModelId> model) { cfg.model = model; }, "model"_a.none())
      .def_prop_rw(
          "calibration", [](const Config &cfg) { return cfg.calibration; },
          [](Config &cfg, std::optional<Calibration> calib) { cfg.calibration = std::move(calib); },
          "calibration"_a.none())
      .def_rw("min_range", &Config::min_range)
      .def_rw("max_range", &Config::max_range)
      .def_rw("min_angle", &Config::min_angle)
      .def_rw("max_angle", &Config::max_angle)
      .def_rw("timestamp_first_packet", &Config::timestamp_first_packet)
      .def_prop_rw(
          "cut_angle", [](const Config &cfg) { return cfg.cut_angle; },
          [](Config &cfg, std::optional<float> cut_angle) { cfg.cut_angle = cut_angle; },
          "cut_angle"_a.none());

  nb::class_<TimePair>(m, "TimePair")
      .def(nb::init<>())
      .def(nb::init<Time, Time>())
      .def_rw("host", &TimePair::host)
      .def_rw("device", &TimePair::device)
      .def("__repr__", [](const TimePair &t) {
        return "TimePair(host=" + std::to_string(t.host) + //
               ", device=" + std::to_string(t.device) + ")";
      });

  nb::class_<VelodynePacket>(m, "VelodynePacket")
      .def(nb::init<>())
      .def(nb::init<Time, const RawPacketData &>(), nb::arg("host_stamp"), nb::arg("data"))
      .def(nb::init<TimePair, const RawPacketData &>(), nb::arg("stamp"), nb::arg("data"))
      .def_rw("stamp", &VelodynePacket::stamp)
      .def_rw("data", &VelodynePacket::data);

  nb::bind_vector<std::vector<VelodynePacket>>(m, "PacketVector");

  nb::class_<ScanDecoder>(m, "ScanDecoder")
      .def(nb::init<const Config &>(), nb::arg("config") = Config())
      .def(
          "decode",
          [](ScanDecoder &decoder, const std::vector<VelodynePacket> &scan_packets,
             bool as_pcl_structs) {
            auto [stamp, cloud] = decoder.decode(scan_packets);
            return std::pair{stamp, convert(std::move(cloud), as_pcl_structs)};
          },
          nb::arg("scan_packets"), nb::arg("as_pcl_structs") = false, //
          nb::rv_policy::move)
      .def(
          "decode_message",
          [](ScanDecoder &decoder, const nb::object &scan_msg, bool as_pcl_structs) {
            std::vector<VelodynePacket> packets;
            nb::iterable packets_py = scan_msg.attr("packets");
            for (const auto &packet_py : packets_py) {
              auto packet = nb::cast<RawPacketData>(packet_py.attr("data"));
              auto stamp  = nb::cast<double>(packet_py.attr("stamp").attr("to_sec")());
              packets.emplace_back(stamp, packet);
            }
            auto [stamp, cloud] = decoder.decode(packets);
            return std::pair{stamp, convert(std::move(cloud), as_pcl_structs)};
          },
          nb::arg("scan_msg"), nb::arg("as_pcl_structs") = false, //
          nb::rv_policy::move)
      .def_prop_ro("model_id", &ScanDecoder::modelId,
                   "Detected or configured model ID of the sensor")
      .def_prop_ro("return_mode", &ScanDecoder::returnMode,
                   "The return mode of the sensor based on the last received packet");

  nb::class_<StreamDecoder>(m, "StreamDecoder")
      .def(nb::init<const Config &>(), nb::arg("config") = Config())
      .def(
          "decode",
          [](StreamDecoder &decoder, Time host_stamp, const RawPacketData &packet,
             bool as_pcl_structs) -> std::optional<std::pair<TimePair, nb::object>> {
            auto result = decoder.decode({host_stamp, packet});
            if (result) {
              auto &[scan_stamp, cloud] = *result;
              return std::make_pair(scan_stamp, convert(std::move(cloud), as_pcl_structs));
            }
            return std::nullopt;
          },
          nb::arg("stamp"), nb::arg("packet"), nb::arg("as_pcl_structs") = false, //
          nb::rv_policy::move)
      .def(
          "finish",
          [](StreamDecoder &decoder,
             bool as_pcl_structs) -> std::optional<std::pair<TimePair, nb::object>> {
            auto result = decoder.finish();
            if (result) {
              auto &[scan_stamp, cloud] = *result;
              return std::make_pair(scan_stamp, convert(std::move(cloud), as_pcl_structs));
            }
            return std::nullopt;
          },
          nb::arg("as_pcl_structs") = false, nb::rv_policy::move);

  nb::class_<Calibration>(m, "Calibration")
      .def_static("read", &Calibration::read)
      .def("write", &Calibration::write)
      .def_static("from_string", &Calibration::fromString)
      .def("to_string", &Calibration::toString)
      .def("__str__", &Calibration::toString)
      .def_prop_ro_static("default_calibs",
                          [](const nb::object &) { return CalibDB().getAllDefaultCalibrations(); });

  auto TelemetryPacket_ =
      nb::class_<TelemetryPacket>(m, "TelemetryPacket")
          .def(nb::init<const std::array<uint8_t, TELEMETRY_PACKET_SIZE> &>(), nb::arg("raw_data"))
          .def_ro("temp_board_top", &TelemetryPacket::temp_board_top,
                  "Temperature of top board, 0 to 150 °C")
          .def_ro("temp_board_bottom", &TelemetryPacket::temp_board_bottom,
                  "Temperature of bottom board, 0 to 150 °C")
          .def_ro("temp_during_adc_calibration", &TelemetryPacket::temp_during_adc_calibration,
                  "Temperature when ADC calibration last ran, 0 to 150 °C")
          .def_ro("temp_change_since_adc_calibration",
                  &TelemetryPacket::temp_change_since_adc_calibration,
                  "Change in temperature since last ADC calibration, -150 to 150 °C")
          .def_ro("seconds_since_adc_calibration", &TelemetryPacket::seconds_since_adc_calibration,
                  "Elapsed seconds since last ADC calibration")
          .def_ro("adc_calibration_reason", &TelemetryPacket::adc_calibration_reason,
                  "Reason for the last ADC calibration")
          .def_ro("adc_calib_in_progress", &TelemetryPacket::adc_calib_in_progress,
                  "ADC calibration in progress")
          .def_ro("adc_delta_temp_limit_exceeded", &TelemetryPacket::adc_delta_temp_limit_exceeded,
                  "ADC calibration: delta temperature limit has been met")
          .def_ro("adc_period_exceeded", &TelemetryPacket::adc_period_exceeded,
                  "ADC calibration: periodic time elapsed limit has been met")
          .def_ro("thermal_shutdown", &TelemetryPacket::thermal_shutdown,
                  "Thermal status, true if thermal shutdown")
          .def_ro("temp_at_shutdown", &TelemetryPacket::temp_at_shutdown,
                  "Temperature of unit when thermal shutdown occurred")
          .def_ro("temp_at_powerup", &TelemetryPacket::temp_at_powerup,
                  "Temperature of unit (bottom board) at power up")
          .def_ro("usec_since_toh", &TelemetryPacket::usec_since_toh,
                  "Number of microseconds elapsed since the top of the hour")
          .def_ro("pps_status", &TelemetryPacket::pps_status, "Pulse Per Second (PPS) status")
          .def_ro("nmea_sentence", &TelemetryPacket::nmea_sentence, "GPRMC or GPGGA NMEA sentence")
          .def("parse_nmea", &TelemetryPacket::parseNmea,
               "Parse the NMEA sentence in the packet, if it exists.")
          .def_prop_ro(
              "gps_position",
              [](const TelemetryPacket &packet) -> nb::object {
                auto nmea_info = packet.parseNmea();
                if (!nmea_info || !nmea_info->fix_available)
                  return nb::none();
                return nb::make_tuple(nmea_info->longitude, nmea_info->latitude,
                                      nmea_info->altitude);
              },
              "GPS position (longitude, latitude, altitude) from the NMEA sentence. "
              "None if no NMEA sentence or no fix available.")
          .def_prop_ro(
              "nmea_time",
              [](const TelemetryPacket &packet) { return convert_gps_time(packet.nmeaTime()); },
              "UTC time from the NMEA sentence. datetime.datetime if date is available, "
              "datetime.time otherwise. None if no valid NMEA sentence in packet.")
          .def_prop_ro(
              "pps_time",
              [](const TelemetryPacket &packet) { return convert_gps_time(packet.ppsTime()); },
              "UTC time from the PPS signal + NMEA sentence. datetime.datetime if date is "
              "available in NMEA, datetime.time otherwise. None if no valid PPS or NMEA.")
          .def("__repr__", [](const TelemetryPacket &packet) {
            std::stringstream ss;
            std::string nmea = packet.nmea_sentence;
            if (size_t pos = nmea.find("\r\n"); pos != std::string::npos)
              nmea.replace(pos, 2, "\\r\\n");
            ss << "TelemetryPacket(";
            // Some fields are only set in newer firmware versions. Only print them if they are set.
            if (packet.temp_board_top != 0 || packet.temp_board_bottom != 0 ||
                packet.temp_during_adc_calibration != 0 ||
                packet.temp_change_since_adc_calibration != 0 ||
                packet.seconds_since_adc_calibration != 0 ||
                (int)packet.adc_calibration_reason != 0 || packet.adc_calib_in_progress != 0 ||
                packet.adc_delta_temp_limit_exceeded != 0 || packet.adc_period_exceeded != 0 ||
                packet.thermal_shutdown != 0 || packet.temp_at_shutdown != 0 ||
                packet.temp_at_powerup != 0) {
              ss << "temp_board_top=" << (int)packet.temp_board_top                             //
                 << ", temp_board_bottom=" << (int)packet.temp_board_bottom                     //
                 << ", temp_during_adc_calibration=" << (int)packet.temp_during_adc_calibration //
                 << ", temp_change_since_adc_calibration="
                 << packet.temp_change_since_adc_calibration                                   //
                 << ", seconds_since_adc_calibration=" << packet.seconds_since_adc_calibration //
                 << ", adc_calibration_reason=" << (int)packet.adc_calibration_reason          //
                 << ", adc_calib_in_progress=" << packet.adc_calib_in_progress                 //
                 << ", adc_delta_temp_limit_exceeded=" << packet.adc_delta_temp_limit_exceeded //
                 << ", adc_period_exceeded=" << packet.adc_period_exceeded                     //
                 << ", thermal_shutdown=" << packet.thermal_shutdown                           //
                 << ", temp_at_shutdown=" << (int)packet.temp_at_shutdown                      //
                 << ", temp_at_powerup=" << (int)packet.temp_at_powerup                        //
                 << ", ";
            }
            ss << "usec_since_toh=" << packet.usec_since_toh //
               << ", pps_status=" << (int)packet.pps_status  //
               << ", nmea_sentence='" << nmea << "')";
            return ss.str();
          });

  nb::enum_<TelemetryPacket::AdcCalibReason>(TelemetryPacket_, "AdcCalibrationReason",
                                             "Reason for the last ADC calibration")
      .value("NO_CALIBRATION", TelemetryPacket::AdcCalibReason::NO_CALIBRATION, "No calibration")
      .value("POWER_ON", TelemetryPacket::AdcCalibReason::POWER_ON,
             "Power-on calibration performed")
      .value("MANUAL", TelemetryPacket::AdcCalibReason::MANUAL, "Manual calibration performed")
      .value("DELTA_TEMPERATURE", TelemetryPacket::AdcCalibReason::DELTA_TEMPERATURE,
             "Delta temperature calibration performed")
      .value("PERIODIC", TelemetryPacket::AdcCalibReason::PERIODIC,
             "Periodic calibration performed");

  nb::enum_<TelemetryPacket::PpsStatus>(TelemetryPacket_, "PpsStatus",
                                        "Pulse Per Second (PPS) status")
      .value("ABSENT", TelemetryPacket::PpsStatus::ABSENT, "No PPS detected")
      .value("SYNCHRONIZING", TelemetryPacket::PpsStatus::SYNCHRONIZING, "Synchronizing to PPS")
      .value("LOCKED", TelemetryPacket::PpsStatus::LOCKED, "PPS Locked")
      .value("ERROR", TelemetryPacket::PpsStatus::ERROR, "Error");

  nb::class_<NmeaInfo>(TelemetryPacket_, "NmeaInfo")
      .def_ro("longitude", &NmeaInfo::longitude, "Longitude in degrees")
      .def_ro("latitude", &NmeaInfo::latitude, "Latitude in degrees")
      .def_ro("altitude", &NmeaInfo::altitude,
              "Altitude above WGS84 ellipsoid in meters (always 0 for GPRMC)")
      .def_ro("utc_year", &NmeaInfo::utc_year, "UTC year (always 0 for GPGGA)")
      .def_ro("utc_month", &NmeaInfo::utc_month, "UTC month (always 0 for GPGGA)")
      .def_ro("utc_day", &NmeaInfo::utc_day, "UTC day (always 0 for GPGGA)")
      .def_ro("utc_hours", &NmeaInfo::utc_hours, "UTC hours")
      .def_ro("utc_minutes", &NmeaInfo::utc_minutes, "UTC minutes")
      .def_ro("utc_seconds", &NmeaInfo::utc_seconds, "UTC seconds")
      .def_ro("fix_available", &NmeaInfo::fix_available, "Position fix available")
      .def("__repr__", [](const NmeaInfo &info) {
        std::ostringstream os;
        os << std::setprecision(9) << std::fixed;
        os << "NmeaInfo(longitude=" << info.longitude                      //
           << ", latitude=" << info.latitude                               //
           << ", altitude=" << info.altitude                               //
           << ", utc_year=" << info.utc_year                               //
           << ", utc_month=" << (int)info.utc_month                        //
           << ", utc_day=" << (int)info.utc_day                            //
           << ", utc_hours=" << (int)info.utc_hours                        //
           << ", utc_minutes=" << (int)info.utc_minutes                    //
           << ", utc_seconds=" << std::setprecision(3) << info.utc_seconds //
           << ", fix_available=" << info.fix_available                     //
           << ")";
        return os.str();
      });

  nb::enum_<ModelId>(m, "Model", nb::is_arithmetic())
      .value("HDL64E_S1", ModelId::HDL64E_S1, "HDL-64E S1")
      .value("HDL64E_S2", ModelId::HDL64E_S2, "HDL-64E S2 and S2.1")
      .value("HDL64E_S3", ModelId::HDL64E_S3, "HDL-64E S3")
      .value("HDL32E", ModelId::HDL32E, "HDL-32E")
      .value("VLP32A", ModelId::VLP32A, "VLP-32A")
      .value("VLP32B", ModelId::VLP32B, "VLP-32B")
      .value("VLP32C", ModelId::VLP32C, "VLP-32C")
      .value("VLP16", ModelId::VLP16, "VLP-16")
      .value("PuckLite", ModelId::PuckLite, "Puck Lite (aka VLP-16)")
      .value("PuckHiRes", ModelId::PuckHiRes, "Puck Hi-Res (aka VLP-16 Hi-Res)")
      .value("VLS128", ModelId::VLS128, "VLS-128 (aka Alpha Prime)")
      .value("AlphaPrime", ModelId::AlphaPrime, "Alpha Prime (aka VLS-128)");

  nb::enum_<ReturnMode>(m, "ReturnMode",
                        "Flag added to the 'ring' field of a point depending on its return type.")
      .value("STRONGEST", ReturnMode::STRONGEST, "The strongest reflection in a firing")
      .value("LAST", ReturnMode::LAST, "The last reflection in a firing")
      .value("BOTH", ReturnMode::BOTH,
             "Point is both the last and the strongest reflection in a firing");

  m.attr("PACKET_SIZE")           = PACKET_SIZE;
  m.attr("TELEMETRY_PACKET_SIZE") = TELEMETRY_PACKET_SIZE;

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
