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

PYBIND11_MAKE_OPAQUE(std::vector<VelodynePacket>);

PYBIND11_MODULE(velodyne_decoder_pylib, m) {
  m.doc() = "";

  py::enum_<ModelId>(m, "Model")
      .value("HDL32E", ModelId::HDL32E, "HDL-32E")
      .value("HDL64E", ModelId::HDL64E, "HDL-64E")
      .value("VLP32A", ModelId::VLP32A, "VLP-32A")
      .value("VLP32B", ModelId::VLP32B, "VLP-32B")
      .value("VLP32C", ModelId::VLP32C, "VLP-32C")
      .value("VLP16", ModelId::VLP16, "VLP-16")
      .value("PuckLite", ModelId::PuckLite, "Puck Lite (aka VLP-16)")
      .value("PuckHiRes", ModelId::PuckHiRes, "Puck Hi-Res (aka VLP-16 Hi-Res)")
      .value("VLS128", ModelId::VLS128, "VLS-128 (aka Alpha Prime)")
      .value("AlphaPrime", ModelId::AlphaPrime, "Alpha Prime (aka VLS-128)");

  py::class_<Config>(m, "Config")
      .def(py::init<>())
      .def(py::init([](std::optional<ModelId> model, const std::optional<Calibration> &calibration,
                       float min_range, float max_range, float min_angle, float max_angle,
                       bool timestamp_first_packet, bool gps_time) {
             auto cfg                    = std::make_unique<Config>();
             cfg->model                  = model;
             cfg->calibration            = calibration;
             cfg->min_range              = min_range;
             cfg->max_range              = max_range;
             cfg->min_angle              = min_angle;
             cfg->max_angle              = max_angle;
             cfg->timestamp_first_packet = timestamp_first_packet;
             cfg->gps_time               = gps_time;
             return cfg;
           }),
           py::kw_only(),                                  //
           py::arg("model")                  = py::none(), //
           py::arg("calibration")            = py::none(), //
           py::arg("min_range")              = 0.1,        //
           py::arg("max_range")              = 200,        //
           py::arg("min_angle")              = 0,          //
           py::arg("max_angle")              = 360,        //
           py::arg("timestamp_first_packet") = false,      //
           py::arg("gps_time")               = false       //
           )
      .def_readwrite("model", &Config::model)
      .def_readwrite("calibration", &Config::calibration)
      .def_readwrite("min_range", &Config::min_range)
      .def_readwrite("max_range", &Config::max_range)
      .def_readwrite("min_angle", &Config::min_angle)
      .def_readwrite("max_angle", &Config::max_angle)
      .def_readwrite("timestamp_first_packet", &Config::timestamp_first_packet)
      .def_readwrite("gps_time", &Config::gps_time)
      .def_readonly_static("TIMINGS_AVAILABLE", &Config::TIMINGS_AVAILABLE);

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
      .def(
          "finish",
          [](StreamDecoder &decoder,
             bool as_pcl_structs) -> std::optional<std::pair<Time, py::array>> {
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

  m.attr("PACKET_SIZE") = PACKET_SIZE;

#define STRING(s) #s
#ifdef VERSION_INFO
  m.attr("__version__") = STRING(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
