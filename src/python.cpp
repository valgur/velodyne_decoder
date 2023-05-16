// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/scan_decoder.h"
#include "velodyne_decoder/stream_decoder.h"
#include "velodyne_decoder/telemetry_packet.h"
#include "velodyne_decoder/types.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace velodyne_decoder;

/**
 * Zero-copy conversion to py::array.
 */
template <typename Sequence, typename dtype = typename Sequence::value_type,
          class = typename std::enable_if<std::is_rvalue_reference<Sequence &&>::value>::type>
inline py::array_t<dtype> as_pyarray(Sequence &&seq) {
  auto *seq_ptr = new Sequence(seq);
  auto capsule  = py::capsule(seq_ptr, [](void *p) { delete reinterpret_cast<Sequence *>(p); });
  return py::array(seq_ptr->size(), seq_ptr->data(), capsule);
}

py::array as_contiguous(const PointCloud &cloud) {
  const int ncols = 6;
  std::vector<std::array<float, ncols>> arr;
  arr.reserve(cloud.size());
  for (const auto &p : cloud) {
    arr.push_back({p.x, p.y, p.z, p.intensity, (float)p.ring, p.time});
  }
  return as_pyarray<decltype(arr), float>(std::move(arr));
}

py::array convert(PointCloud &cloud, bool as_pcl_structs) {
  if (as_pcl_structs) {
    return as_pyarray(std::move(cloud));
  } else {
    return as_contiguous(cloud);
  }
}

py::object to_datetime(Time time) {
  static auto fromtimestamp = py::module::import("datetime").attr("datetime").attr("fromtimestamp");
  return fromtimestamp(time);
}

py::object convert_gps_time(std::optional<Time> time) {
  if (!time)
    return py::none();
  if (*time < 1e5) // a full datetime is unavailable, only time of day
    return to_datetime(*time).attr("time")();
  return to_datetime(*time);
}

PYBIND11_MAKE_OPAQUE(std::vector<VelodynePacket>);

PYBIND11_MODULE(velodyne_decoder_pylib, m) {
  m.doc() = "";

  py::class_<Config>(m, "Config")
      .def(py::init([](std::optional<ModelId> model, const std::optional<Calibration> &calibration,
                       bool single_return_mode_info, float min_range, float max_range,
                       float min_angle, float max_angle, std::optional<float> cut_angle,
                       bool timestamp_first_packet) {
             auto cfg                     = std::make_unique<Config>();
             cfg->model                   = model;
             cfg->calibration             = calibration;
             cfg->min_range               = min_range;
             cfg->max_range               = max_range;
             cfg->min_angle               = min_angle;
             cfg->max_angle               = max_angle;
             cfg->timestamp_first_packet  = timestamp_first_packet;
             cfg->cut_angle               = cut_angle;
             cfg->single_return_mode_info = single_return_mode_info;
             return cfg;
           }),
           py::kw_only(),                                   //
           py::arg("model")                   = py::none(), //
           py::arg("calibration")             = py::none(), //
           py::arg("single_return_mode_info") = false,      //
           py::arg("min_range")               = 0.1,        //
           py::arg("max_range")               = 200,        //
           py::arg("min_angle")               = 0,          //
           py::arg("max_angle")               = 360,        //
           py::arg("cut_angle")               = py::none(), //
           py::arg("timestamp_first_packet")  = false       //
           )
      .def_readwrite("model", &Config::model)
      .def_readwrite("calibration", &Config::calibration)
      .def_readwrite("single_return_mode_info", &Config::single_return_mode_info)
      .def_readwrite("min_range", &Config::min_range)
      .def_readwrite("max_range", &Config::max_range)
      .def_readwrite("min_angle", &Config::min_angle)
      .def_readwrite("max_angle", &Config::max_angle)
      .def_readwrite("timestamp_first_packet", &Config::timestamp_first_packet)
      .def_readwrite("cut_angle", &Config::cut_angle);

  py::class_<TimePair>(m, "TimePair")
      .def(py::init<>())
      .def(py::init<Time, Time>())
      .def_readwrite("host", &TimePair::host)
      .def_readwrite("device", &TimePair::device)
      .def("__repr__", [](const TimePair &t) {
        return "TimePair(host=" + std::to_string(t.host) + //
               ", device=" + std::to_string(t.device) + ")";
      });

  py::class_<VelodynePacket>(m, "VelodynePacket")
      .def(py::init<>())
      .def(py::init<Time, const RawPacketData &>())
      .def(py::init<TimePair, const RawPacketData &>())
      .def_readwrite("stamp", &VelodynePacket::stamp)
      .def_readwrite("data", &VelodynePacket::data);

  py::bind_vector<std::vector<VelodynePacket>>(m, "PacketVector");

  PYBIND11_NUMPY_DTYPE(PointXYZIRT, x, y, z, intensity, ring, time);

  py::class_<ScanDecoder>(m, "ScanDecoder")
      .def(py::init<const Config &>(), py::arg("config"))
      .def(
          "decode",
          [](ScanDecoder &decoder, TimePair stamp, const std::vector<VelodynePacket> &scan_packets,
             bool as_pcl_structs) {
            auto cloud = decoder.decode(stamp, scan_packets);
            return convert(cloud, as_pcl_structs);
          },
          py::arg("scan_stamp"), py::arg("scan_packets"), py::arg("as_pcl_structs") = false, //
          py::return_value_policy::move)
      .def(
          "decode",
          [](ScanDecoder &decoder, Time host_stamp, const std::vector<VelodynePacket> &scan_packets,
             bool as_pcl_structs) {
            auto cloud = decoder.decode(host_stamp, scan_packets);
            return convert(cloud, as_pcl_structs);
          },
          py::arg("scan_host_stamp"), py::arg("scan_packets"), py::arg("as_pcl_structs") = false, //
          py::return_value_policy::move)
      .def(
          "decode_message",
          [](ScanDecoder &decoder, const py::object &scan_msg, bool as_pcl_structs) {
            std::vector<VelodynePacket> packets;
            py::iterable packets_py = scan_msg.attr("packets");
            for (const auto &packet_py : packets_py) {
              auto packet = packet_py.attr("data").cast<RawPacketData>();
              auto stamp  = packet_py.attr("stamp").attr("to_sec")().cast<double>();
              packets.emplace_back(stamp, packet);
            }
            Time msg_host_stamp =
                scan_msg.attr("header").attr("stamp").attr("to_sec")().cast<double>();
            auto cloud = decoder.decode(msg_host_stamp, packets);
            return convert(cloud, as_pcl_structs);
          },
          py::arg("scan_msg"), py::arg("as_pcl_structs") = false, //
          py::return_value_policy::move)
      .def_property_readonly("model_id", &ScanDecoder::modelId,
                             "Detected or configured model ID of the sensor")
      .def_property_readonly("return_mode", &ScanDecoder::returnMode,
                             "The return mode of the sensor based on the last received packet");

  py::class_<StreamDecoder>(m, "StreamDecoder")
      .def(py::init<const Config &>(), py::arg("config"))
      .def(
          "decode",
          [](StreamDecoder &decoder, Time stamp, const RawPacketData &packet,
             bool as_pcl_structs) -> std::optional<std::pair<TimePair, py::array>> {
            auto result = decoder.decode(stamp, packet);
            if (result) {
              auto &[scan_stamp, cloud] = *result;
              return std::make_pair(scan_stamp, convert(cloud, as_pcl_structs));
            }
            return std::nullopt;
          },
          py::arg("stamp"), py::arg("packet"), py::arg("as_pcl_structs") = false, //
          py::return_value_policy::move)
      .def(
          "finish",
          [](StreamDecoder &decoder,
             bool as_pcl_structs) -> std::optional<std::pair<TimePair, py::array>> {
            auto result = decoder.finish();
            if (result) {
              auto &[scan_stamp, cloud] = *result;
              return std::make_pair(scan_stamp, convert(cloud, as_pcl_structs));
            }
            return std::nullopt;
          },
          py::arg("as_pcl_structs") = false, py::return_value_policy::move);

  py::class_<Calibration>(m, "Calibration")
      .def_static("read", &Calibration::read)
      .def("write", &Calibration::write)
      .def_static("from_string", &Calibration::fromString)
      .def("to_string", &Calibration::toString)
      .def("__str__", &Calibration::toString)
      .def_property_readonly_static("default_calibs", [](const py::object &) {
        return CalibDB().getAllDefaultCalibrations();
      });

  auto TelemetryPacket_ =
      py::class_<TelemetryPacket>(m, "TelemetryPacket")
          .def(py::init<const std::array<uint8_t, TELEMETRY_PACKET_SIZE> &>(), py::arg("raw_data"))
          .def_readonly("temp_board_top", &TelemetryPacket::temp_board_top,
                        "Temperature of top board, 0 to 150 °C")
          .def_readonly("temp_board_bottom", &TelemetryPacket::temp_board_bottom,
                        "Temperature of bottom board, 0 to 150 °C")
          .def_readonly("temp_during_adc_calibration",
                        &TelemetryPacket::temp_during_adc_calibration,
                        "Temperature when ADC calibration last ran, 0 to 150 °C")
          .def_readonly("temp_change_since_adc_calibration",
                        &TelemetryPacket::temp_change_since_adc_calibration,
                        "Change in temperature since last ADC calibration, -150 to 150 °C")
          .def_readonly("seconds_since_adc_calibration",
                        &TelemetryPacket::seconds_since_adc_calibration,
                        "Elapsed seconds since last ADC calibration")
          .def_readonly("adc_calibration_reason", &TelemetryPacket::adc_calibration_reason,
                        "Reason for the last ADC calibration")
          .def_readonly("adc_calib_in_progress", &TelemetryPacket::adc_calib_in_progress,
                        "ADC calibration in progress")
          .def_readonly("adc_delta_temp_limit_exceeded",
                        &TelemetryPacket::adc_delta_temp_limit_exceeded,
                        "ADC calibration: delta temperature limit has been met")
          .def_readonly("adc_period_exceeded", &TelemetryPacket::adc_period_exceeded,
                        "ADC calibration: periodic time elapsed limit has been met")
          .def_readonly("thermal_shutdown", &TelemetryPacket::thermal_shutdown,
                        "Thermal status, true if thermal shutdown")
          .def_readonly("temp_at_shutdown", &TelemetryPacket::temp_at_shutdown,
                        "Temperature of unit when thermal shutdown occurred")
          .def_readonly("temp_at_powerup", &TelemetryPacket::temp_at_powerup,
                        "Temperature of unit (bottom board) at power up")
          .def_readonly("usec_since_toh", &TelemetryPacket::usec_since_toh,
                        "Number of microseconds elapsed since the top of the hour")
          .def_readonly("pps_status", &TelemetryPacket::pps_status, "Pulse Per Second (PPS) status")
          .def_readonly("nmea_sentence", &TelemetryPacket::nmea_sentence,
                        "GPRMC or GPGGA NMEA sentence")
          .def("parse_nmea", &TelemetryPacket::parseNmea,
               "Parse the NMEA sentence in the packet, if it exists.")
          .def_property_readonly(
              "gps_position",
              [](const TelemetryPacket &packet) -> py::object {
                auto nmea_info = packet.parseNmea();
                if (!nmea_info || !nmea_info->fix_available)
                  return py::none();
                return py::make_tuple(nmea_info->longitude, nmea_info->latitude,
                                      nmea_info->altitude);
              },
              "GPS position (longitude, latitude, altitude) from the NMEA sentence. "
              "None if no NMEA sentence or no fix available.")
          .def_property_readonly(
              "nmea_time",
              [](const TelemetryPacket &packet) { return convert_gps_time(packet.nmeaTime()); },
              "UTC time from the NMEA sentence. datetime.datetime if date is available, "
              "datetime.time otherwise. None if no valid NMEA sentence in packet.")
          .def_property_readonly(
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

  py::enum_<TelemetryPacket::AdcCalibReason>(TelemetryPacket_, "AdcCalibrationReason",
                                             "Reason for the last ADC calibration")
      .value("NO_CALIBRATION", TelemetryPacket::AdcCalibReason::NO_CALIBRATION, "No calibration")
      .value("POWER_ON", TelemetryPacket::AdcCalibReason::POWER_ON,
             "Power-on calibration performed")
      .value("MANUAL", TelemetryPacket::AdcCalibReason::MANUAL, "Manual calibration performed")
      .value("DELTA_TEMPERATURE", TelemetryPacket::AdcCalibReason::DELTA_TEMPERATURE,
             "Delta temperature calibration performed")
      .value("PERIODIC", TelemetryPacket::AdcCalibReason::PERIODIC,
             "Periodic calibration performed");

  py::enum_<TelemetryPacket::PpsStatus>(TelemetryPacket_, "PpsStatus",
                                        "Pulse Per Second (PPS) status")
      .value("ABSENT", TelemetryPacket::PpsStatus::ABSENT, "No PPS detected")
      .value("SYNCHRONIZING", TelemetryPacket::PpsStatus::SYNCHRONIZING, "Synchronizing to PPS")
      .value("LOCKED", TelemetryPacket::PpsStatus::LOCKED, "PPS Locked")
      .value("ERROR", TelemetryPacket::PpsStatus::ERROR, "Error");

  py::class_<NmeaInfo>(TelemetryPacket_, "NmeaInfo")
      .def_readonly("longitude", &NmeaInfo::longitude, "Longitude in degrees")
      .def_readonly("latitude", &NmeaInfo::latitude, "Latitude in degrees")
      .def_readonly("altitude", &NmeaInfo::altitude,
                    "Altitude above WGS84 ellipsoid in meters (always 0 for GPRMC)")
      .def_readonly("utc_year", &NmeaInfo::utc_year, "UTC year (always 0 for GPGGA)")
      .def_readonly("utc_month", &NmeaInfo::utc_month, "UTC month (always 0 for GPGGA)")
      .def_readonly("utc_day", &NmeaInfo::utc_day, "UTC day (always 0 for GPGGA)")
      .def_readonly("utc_hours", &NmeaInfo::utc_hours, "UTC hours")
      .def_readonly("utc_minutes", &NmeaInfo::utc_minutes, "UTC minutes")
      .def_readonly("utc_seconds", &NmeaInfo::utc_seconds, "UTC seconds")
      .def_readonly("fix_available", &NmeaInfo::fix_available, "Position fix available")
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

  py::enum_<ModelId>(m, "Model")
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

  py::enum_<ReturnModeFlag>(
      m, "ReturnModeFlag",
      "Flag added to the 'ring' field of a point depending on its return type.")
      .value("SINGLE_RETURN", ReturnModeFlag::SINGLE_RETURN_FLAG,
             "Single-return mode (if single_return_mode_info is False)")
      .value("STRONGEST", ReturnModeFlag::STRONGEST_RETURN_FLAG,
             "The strongest reflection in a firing")
      .value("LAST", ReturnModeFlag::STRONGEST_RETURN_FLAG, "The last reflection in a firing")
      .value("BOTH", ReturnModeFlag::BOTH_RETURN_FLAG,
             "Point is both the last and the strongest reflection in a firing");

  py::enum_<DualReturnMode>(
      m, "DualReturnMode",
      "Possible values of the the 'return_mode' attribute for the PacketDecoder.")
      .value("STRONGEST", DualReturnMode::STRONGEST_RETURN)
      .value("LAST", DualReturnMode::LAST_RETURN)
      .value("DUAL", DualReturnMode::DUAL_RETURN);

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
