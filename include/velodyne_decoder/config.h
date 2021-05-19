// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <utility>
#include <vector>

namespace velodyne_decoder {

struct Config {
  std::string model;
  std::string calibration_file; ///< calibration file name
  float min_range = 0.1;        ///< minimum range to publish
  float max_range = 200;        ///< maximum range to publish
  int min_angle   = 0;          ///< minimum angle to publish
  int max_angle   = 36000;      ///< maximum angle to publish

  Config() = default;
  Config(std::string model, std::string calibration_file, float min_range, float max_range,
         double min_angle, double max_angle)
      : model(std::move(model)), calibration_file(std::move(calibration_file)),
        min_range(min_range), max_range(max_range), min_angle(std::lround(min_angle * 100)),
        max_angle(std::lround(max_angle * 100)) {}

  void setMinAngleDeg(double min_angle_) { min_angle = std::lround(min_angle_ * 100); }
  double getMinAngleDeg() const { return static_cast<double>(min_angle) / 100; }
  void setMaxAngleDeg(double max_angle_) { max_angle = std::lround(max_angle_ * 100); }
  double getMaxAngleDeg() const { return static_cast<double>(max_angle) / 100; }

  static const std::vector<std::string> SUPPORTED_MODELS;
};

} // namespace velodyne_decoder
