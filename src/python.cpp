// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include <string>
#include <type_traits>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/scan_decoder.h"
#include "velodyne_decoder/stream_decoder.h"
#include "velodyne_decoder/types.h"

namespace py = pybind11;
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

std::string get_default_calibration(const std::string &model) {
  py::function files = py::module::import("importlib_resources").attr("files");
  return py::str(files("velodyne_decoder.calibrations").attr("joinpath")(model + ".yml"))
      .cast<std::string>();
}

PYBIND11_MAKE_OPAQUE(std::vector<VelodynePacket>);

PYBIND11_MODULE(velodyne_decoder_pylib, m) {
  m.doc() = "";

  py::class_<Config>(m, "Config")
      .def(py::init<>())
      .def(py::init([](const std::string &model, const std::string &calibration_file,
                       float min_range, float max_range, double min_angle, double max_angle,
                       double rpm, bool timestamp_first_packet, bool gps_time) {
             auto cfg   = std::make_unique<Config>();
             cfg->model = Config::standardizeModelId(model);
             cfg->calibration_file =
                 calibration_file.empty() ? get_default_calibration(model) : calibration_file;
             cfg->min_range = min_range;
             cfg->max_range = max_range;
             cfg->setMinAngleDeg(min_angle);
             cfg->setMaxAngleDeg(max_angle);
             cfg->rpm                    = rpm;
             cfg->timestamp_first_packet = timestamp_first_packet;
             cfg->gps_time               = gps_time;
             return cfg;
           }),
           py::arg("model"),                          //
           py::kw_only(),                             //
           py::arg("calibration_file")       = "",    //
           py::arg("min_range")              = 0.1,   //
           py::arg("max_range")              = 200,   //
           py::arg("min_angle")              = 0,     //
           py::arg("max_angle")              = 360,   //
           py::arg("rpm")                    = -1,    //
           py::arg("timestamp_first_packet") = false, //
           py::arg("gps_time")               = false  //
           )
      .def_property(
          "model", [](const Config &c) { return c.model; },
          [](Config &c, const std::string &model) {
            c.model = Config::standardizeModelId(model);
            if (c.calibration_file.empty()) {
              c.calibration_file = get_default_calibration(c.model);
            }
          })
      .def_readwrite("calibration_file", &Config::calibration_file)
      .def_readwrite("min_range", &Config::min_range)
      .def_readwrite("max_range", &Config::max_range)
      .def_property("min_angle", &Config::getMinAngleDeg, &Config::setMinAngleDeg)
      .def_property("max_angle", &Config::getMaxAngleDeg, &Config::setMaxAngleDeg)
      .def_readwrite("rpm", &Config::rpm)
      .def_readwrite("timestamp_first_packet", &Config::timestamp_first_packet)
      .def_readwrite("gps_time", &Config::gps_time)
      .def_readonly_static("SUPPORTED_MODELS", &Config::SUPPORTED_MODELS)
      .def_readonly_static("TIMINGS_AVAILABLE", &Config::TIMINGS_AVAILABLE);

  m.def("get_default_calibration", &get_default_calibration);

  py::class_<VelodynePacket>(m, "VelodynePacket")
      .def(py::init<>())
      .def(py::init<Time, const RawPacketData &>())
      .def_readwrite("stamp", &VelodynePacket::stamp)
      .def_readwrite("data", &VelodynePacket::data);

  py::bind_vector<std::vector<VelodynePacket>>(m, "PacketVector");

  PYBIND11_NUMPY_DTYPE(PointXYZIRT, x, y, z, intensity, ring, time);

  py::class_<ScanDecoder>(m, "ScanDecoder")
      .def(py::init<const Config &>(), py::arg("config"))
      .def(
          "decode",
          [](ScanDecoder &decoder, Time stamp, const std::vector<VelodynePacket> &scan_packets,
             bool as_pcl_structs) {
            auto cloud = decoder.decode(stamp, scan_packets);
            return convert(cloud, as_pcl_structs);
          },
          py::arg("scan_stamp"), py::arg("scan_packets"), py::arg("as_pcl_structs") = false, //
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
            auto stamp = scan_msg.attr("header").attr("stamp").attr("to_sec")().cast<double>();
            auto cloud = decoder.decode(stamp, packets);
            return convert(cloud, as_pcl_structs);
          },
          py::arg("scan_msg"), py::arg("as_pcl_structs") = false, //
          py::return_value_policy::move);

  py::class_<StreamDecoder>(m, "StreamDecoder")
      .def(py::init<const Config &>(), py::arg("config"))
      .def(
          "decode",
          [](StreamDecoder &decoder, Time stamp, const RawPacketData &packet,
             bool as_pcl_structs) -> std::optional<std::pair<Time, py::array>> {
            auto result = decoder.decode(stamp, packet);
            if (result) {
              auto &[scan_stamp, cloud] = *result;
              return std::make_pair(scan_stamp, convert(cloud, as_pcl_structs));
            }
            return std::nullopt;
          },
          py::arg("stamp"), py::arg("packet"), py::arg("as_pcl_structs") = false, //
          py::return_value_policy::move)
      .def("calc_packets_per_scan", &StreamDecoder::calc_packets_per_scan);

  m.attr("PACKET_SIZE") = PACKET_SIZE;

#define STRING(s) #s
#ifdef VERSION_INFO
  m.attr("__version__") = STRING(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
