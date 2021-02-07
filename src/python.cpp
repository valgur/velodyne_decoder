/*
 *  Copyright (C) 2021 Martin Valgur
 *  License: Modified BSD Software License Agreement
 */

#include <string>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/scan_decoder.h"
#include "velodyne_decoder/types.h"

namespace py = pybind11;
using namespace velodyne_decoder;

PYBIND11_MAKE_OPAQUE(std::vector<VelodynePacket>);

PYBIND11_MODULE(velodyne_decoder, m) {
  m.doc() = "";

  py::class_<Config>(m, "Config")
      .def(py::init<>())
      .def(py::init<std::string, std::string, float, float, double, double>(), py::arg("model"),
           py::arg("calibration_file"), py::arg("min_range") = 10, py::arg("max_range") = 200,
           py::arg("min_angle") = 0, py::arg("max_angle") = 360)
      .def_readwrite("model", &Config::model)
      .def_readwrite("calibration_file", &Config::calibration_file)
      .def_readwrite("min_range", &Config::min_range)
      .def_readwrite("max_range", &Config::max_range)
      .def_property("min_angle", &Config::getMinAngleDeg, &Config::setMinAngleDeg)
      .def_property("max_angle", &Config::getMaxAngleDeg, &Config::setMaxAngleDeg)
      .def_readonly_static("SUPPORTED_MODELS", &Config::SUPPORTED_MODELS);

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
          [](ScanDecoder &decoder, Time stamp, const std::vector<VelodynePacket> &scan_packets) {
            auto cloud = decoder.decode(stamp, scan_packets);
            return py::array(cloud.size(), cloud.data());
          },
          py::arg("scan_stamp"), py::arg("scan_packets"))
      .def(
          "decode_message",
          [](ScanDecoder &decoder, const py::object &scan_msg) {
            std::vector<VelodynePacket> packets;
            py::iterable packets_py = scan_msg.attr("packets");
            for (const auto &packet_py : packets_py) {
              auto packet = packet_py.attr("data").cast<RawPacketData>();
              auto stamp  = packet_py.attr("stamp").attr("to_sec")().cast<double>();
              packets.emplace_back(stamp, packet);
            }
            auto stamp = scan_msg.attr("header").attr("stamp").attr("to_sec")().cast<double>();
            auto cloud = decoder.decode(stamp, packets);
            return py::array(cloud.size(), cloud.data());
          },
          py::arg("scan_msg"));

#define STRING(s) #s
#ifdef VERSION_INFO
  m.attr("__version__") = STRING(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
