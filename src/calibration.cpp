// Copyright (c) 2012, Austin Robot Technology, Piyush Khandelwal
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/calibration.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace YAML {

namespace {
constexpr auto NUM_LASERS              = "num_lasers";
constexpr auto DISTANCE_RESOLUTION     = "distance_resolution";
constexpr auto LASERS                  = "lasers";
constexpr auto LASER_ID                = "laser_id";
constexpr auto ROT_CORRECTION          = "rot_correction";
constexpr auto VERT_CORRECTION         = "vert_correction";
constexpr auto DIST_CORRECTION         = "dist_correction";
constexpr auto DIST_CORRECTION_X       = "dist_correction_x";
constexpr auto DIST_CORRECTION_Y       = "dist_correction_y";
constexpr auto VERT_OFFSET_CORRECTION  = "vert_offset_correction";
constexpr auto HORIZ_OFFSET_CORRECTION = "horiz_offset_correction";
constexpr auto MAX_INTENSITY           = "max_intensity";
constexpr auto MIN_INTENSITY           = "min_intensity";
constexpr auto FOCAL_DISTANCE          = "focal_distance";
constexpr auto FOCAL_SLOPE             = "focal_slope";
} // namespace

template <> struct convert<velodyne_decoder::LaserCorrection> {
  static bool decode(const YAML::Node &node, velodyne_decoder::LaserCorrection &correction) {
    correction.laser_idx               = node[LASER_ID].as<int>();
    correction.rot_correction          = node[ROT_CORRECTION].as<float>();
    correction.vert_correction         = node[VERT_CORRECTION].as<float>();
    correction.dist_correction         = node[DIST_CORRECTION].as<float>(0);
    correction.dist_correction_x       = node[DIST_CORRECTION_X].as<float>(0);
    correction.dist_correction_y       = node[DIST_CORRECTION_Y].as<float>(0);
    correction.vert_offset_correction  = node[VERT_OFFSET_CORRECTION].as<float>(0);
    correction.horiz_offset_correction = node[HORIZ_OFFSET_CORRECTION].as<float>(0);
    correction.max_intensity           = std::floor(node[MAX_INTENSITY].as<float>(255));
    correction.min_intensity           = std::floor(node[MIN_INTENSITY].as<float>(0));
    correction.focal_distance          = node[FOCAL_DISTANCE].as<float>(0);
    correction.focal_slope             = node[FOCAL_SLOPE].as<float>(0);
    correction.two_pt_correction_available =
        correction.dist_correction_x != 0 && correction.dist_correction_y != 0;
    return true;
  };

  static YAML::Node encode(const velodyne_decoder::LaserCorrection &correction) {
    YAML::Node node;
    if (correction.dist_correction != 0) {
      node[DIST_CORRECTION] = correction.dist_correction;
    }
    if (correction.dist_correction_x != 0) {
      node[DIST_CORRECTION_X] = correction.dist_correction_x;
    }
    if (correction.dist_correction_y != 0) {
      node[DIST_CORRECTION_Y] = correction.dist_correction_y;
    }
    if (correction.focal_distance != 0) {
      node[FOCAL_DISTANCE] = correction.focal_distance;
    }
    if (correction.focal_slope != 0) {
      node[FOCAL_SLOPE] = correction.focal_slope;
    }
    if (correction.horiz_offset_correction != 0) {
      node[HORIZ_OFFSET_CORRECTION] = correction.horiz_offset_correction;
    }
    node[LASER_ID] = correction.laser_idx;
    if (correction.min_intensity != 0 || correction.max_intensity != 255) {
      node[MAX_INTENSITY] = correction.max_intensity;
      node[MIN_INTENSITY] = correction.min_intensity;
    }
    node[ROT_CORRECTION]  = correction.rot_correction;
    node[VERT_CORRECTION] = correction.vert_correction;
    if (correction.vert_offset_correction != 0) {
      node[VERT_OFFSET_CORRECTION] = correction.vert_offset_correction;
    }
    return node;
  }
};

template <> struct convert<velodyne_decoder::Calibration> {
  static bool decode(const YAML::Node &node, velodyne_decoder::Calibration &calibration) {
    int num_lasers = node[NUM_LASERS].as<int>();
    std::vector<velodyne_decoder::LaserCorrection> corrections(num_lasers);
    for (const auto &laser : node[LASERS]) {
      auto correction = laser.as<velodyne_decoder::LaserCorrection>();
      if (corrections.size() <= correction.laser_idx) {
        corrections.resize(correction.laser_idx + 1);
      }
      corrections[correction.laser_idx] = correction;
    }
    calibration = {corrections, node[DISTANCE_RESOLUTION].as<float>()};
    return true;
  }

  static YAML::Node encode(const velodyne_decoder::Calibration &calibration) {
    YAML::Node node;
    node[DISTANCE_RESOLUTION] = calibration.distance_resolution_m;
    for (const auto &correction : calibration.laser_corrections) {
      node[LASERS][correction.laser_idx] = correction;
    }
    node[NUM_LASERS] = calibration.laser_corrections.size();
    return node;
  }
};

} // namespace YAML

namespace velodyne_decoder {

Calibration::Calibration(const std::string &calibration_file)
    : Calibration(read(calibration_file)) {}

Calibration::Calibration(std::vector<LaserCorrection> laser_corrs, float distance_resolution_m)
    : distance_resolution_m(distance_resolution_m), laser_corrections(std::move(laser_corrs)) {
  num_lasers = (int)laser_corrections.size();
  assignRingNumbers();
}

void Calibration::assignRingNumbers() {
  // argsort by the vertical angle of the laser to assign ring numbers to lasers
  std::vector<size_t> idx(num_lasers);
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(), [this](size_t i1, size_t i2) {
    return laser_corrections[i1].vert_correction < laser_corrections[i2].vert_correction;
  });
  for (size_t i = 0; i < idx.size(); i++) {
    laser_corrections[idx[i]].laser_ring = i;
  }
}

bool Calibration::isAdvancedCalibration() const {
  // Everything besides rot_correction and vert_correction is considered "advanced",
  // since the default calibrations don't really set these (except for the oldest model, HDL-64E).
  for (const auto &corr : laser_corrections) {
    if (corr.dist_correction != 0)
      return true;
    if (corr.two_pt_correction_available)
      return true;
    if (corr.dist_correction_x != 0)
      return true;
    if (corr.dist_correction_y != 0)
      return true;
    if (corr.vert_offset_correction != 0)
      return true;
    if (corr.horiz_offset_correction != 0)
      return true;
    if (corr.max_intensity != 255)
      return true;
    if (corr.min_intensity != 0)
      return true;
    if (corr.focal_distance != 0)
      return true;
    if (corr.focal_slope != 0)
      return true;
  }
  return false;
}

Calibration Calibration::read(const std::string &calibration_file) {
  std::ifstream fin{calibration_file};
  if (!fin.is_open()) {
    throw std::runtime_error("Unable to open calibration file: " + calibration_file);
  }
  std::stringstream buffer;
  buffer << fin.rdbuf();
  fin.close();
  return fromString(buffer.str());
}

void Calibration::write(const std::string &calibration_file) const {
  std::ofstream fout{calibration_file};
  fout << toString();
  fout.close();
}

Calibration Calibration::fromString(const std::string &calibration_content) {
  return YAML::Load(calibration_content).as<Calibration>();
}

std::string Calibration::toString() const { return YAML::Dump(YAML::Node(*this)); }

} // namespace velodyne_decoder
