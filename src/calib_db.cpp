// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include "velodyne_decoder/calibration.h"

namespace velodyne_decoder {

namespace {
struct MinimalCalibData {
  float rot_corr_deg;
  float vert_corr_deg;
};
} // namespace

CalibDB::CalibDB() {
  std::map<ModelId, float> resolutions = {
      {ModelId::AlphaPrime, 0.004}, {ModelId::HDL32E, 0.002}, {ModelId::PuckHiRes, 0.002},
      {ModelId::VLP16, 0.002},      {ModelId::VLP32A, 0.004}, {ModelId::VLP32C, 0.004},
  };

  // Distance from the sensor center to the focal point where all laser beams intersect.
  std::map<ModelId, float> focal_distances = {
      {ModelId::AlphaPrime, 58.63e-3}, {ModelId::HDL32E, 28.83e-3}, {ModelId::PuckHiRes, 41.91e-3},
      {ModelId::VLP16, 41.91e-3},      {ModelId::VLP32A, 42.4e-3},  {ModelId::VLP32C, 42.4e-3},
  };

  std::map<ModelId, std::vector<float>> vertical_offsets;
  // https://velodynelidar.com/wp-content/uploads/2019/09/86-0101-REV-B1-ENVELOPEVLP-16.pdf#page=3
  // VLP-16 vertical offsets are documented, but re-deriving them from the focal distance and angles
  // produces more accurate values.
  // https://velodynelidar.com/wp-content/uploads/2019/09/86-0129-REV-A-ENVELOPEHI-RESVLP-16.pdf#page=3
  vertical_offsets[ModelId::PuckHiRes] = {
      7.4e-3, -0.9e-3, 6.5e-3, -1.8e-3, 5.5e-3, -2.7e-3, 4.6e-3, -3.7e-3,
      3.7e-3, -4.6e-3, 2.7e-3, -5.5e-3, 1.8e-3, -6.5e-3, 0.9e-3, -7.4e-3,
  };
  // https://velodynelidar.com/wp-content/uploads/2019/09/86-0106-REV-BENVELOPEHDL-32E.pdf#page=2
  vertical_offsets[ModelId::HDL32E] = {
      17.17e-3, 4.76e-3,  16.27e-3, 4.07e-3,  15.4e-3,  3.38e-3,  14.54e-3, 2.7e-3,
      13.71e-3, 2.02e-3,  12.89e-3, 1.35e-3,  12.09e-3, 0.67e-3,  11.31e-3, 0,
      10.54e-3, -0.67e-3, 9.78e-3,  -1.35e-3, 9.04e-3,  -2.02e-3, 8.3e-3,   -2.7e-3,
      7.58e-3,  -3.38e-3, 6.86e-3,  -4.07e-3, 6.15e-3,  -4.76e-3, 5.45e-3,  -5.45e-3,
  };

  std::map<ModelId, std::vector<MinimalCalibData>> raw_calib_data;
  raw_calib_data[ModelId::AlphaPrime] = {
      {-6.354, -11.742}, {-4.548, -1.99}, {-2.732, 3.4},     {-0.911, -5.29},  {0.911, -0.78},
      {2.732, 4.61},     {4.548, -4.08},  {6.354, 1.31},     {-6.354, -6.5},   {-4.548, -1.11},
      {-2.732, 4.28},    {-0.911, -4.41}, {0.911, 0.1},      {2.732, 6.48},    {4.548, -3.2},
      {6.354, 2.19},     {-6.354, -3.86}, {-4.548, 1.53},    {-2.732, -9.244}, {-0.911, -1.77},
      {0.911, 2.74},     {2.732, -5.95},  {4.548, -0.56},    {6.354, 4.83},    {-6.354, -2.98},
      {-4.548, 2.41},    {-2.732, -6.28}, {-0.911, -0.89},   {0.911, 3.62},    {2.732, -5.07},
      {4.548, 0.32},     {6.354, 7.58},   {-6.354, -0.34},   {-4.548, 5.18},   {-2.732, -3.64},
      {-0.911, 1.75},    {0.911, -25},    {2.732, -2.43},    {4.548, 2.96},    {6.354, -5.73},
      {-6.354, 0.54},    {-4.548, 9.7},   {-2.732, -2.76},   {-0.911, 2.63},   {0.911, -7.65},
      {2.732, -1.55},    {4.548, 3.84},   {6.354, -4.85},    {-6.354, 3.18},   {-4.548, -5.51},
      {-2.732, -0.12},   {-0.911, 5.73},  {0.911, -4.3},     {2.732, 1.09},    {4.548, -16.042},
      {6.354, -2.21},    {-6.354, 4.06},  {-4.548, -4.63},   {-2.732, 0.76},   {-0.911, 15},
      {0.911, -3.42},    {2.732, 1.97},   {4.548, -6.85},    {6.354, -1.33},   {-6.354, -5.62},
      {-4.548, -0.23},   {-2.732, 5.43},  {-0.911, -3.53},   {0.911, 0.98},    {2.732, -19.582},
      {4.548, -2.32},    {6.354, 3.07},   {-6.354, -4.74},   {-4.548, 0.65},   {-2.732, 11.75},
      {-0.911, -2.65},   {0.911, 1.86},   {2.732, -7.15},    {4.548, -1.44},   {6.354, 3.95},
      {-6.354, -2.1},    {-4.548, 3.29},  {-2.732, -5.4},    {-0.911, -0.01},  {0.911, 4.5},
      {2.732, -4.19},    {4.548, 1.2},    {6.354, -13.565},  {-6.354, -1.22},  {-4.548, 4.17},
      {-2.732, -4.52},   {-0.911, 0.87},  {0.911, 6.08},     {2.732, -3.31},   {4.548, 2.08},
      {6.354, -6.65},    {-6.354, 1.42},  {-4.548, -10.346}, {-2.732, -1.88},  {-0.911, 3.51},
      {0.911, -6.06},    {2.732, -0.67},  {4.548, 4.72},     {6.354, -3.97},   {-6.354, 2.3},
      {-4.548, -6.39},   {-2.732, -1},    {-0.911, 4.39},    {0.911, -5.18},   {2.732, 0.21},
      {4.548, 6.98},     {6.354, -3.09},  {-6.354, 4.98},    {-4.548, -3.75},  {-2.732, 1.64},
      {-0.911, -8.352},  {0.911, -2.54},  {2.732, 2.85},     {4.548, -5.84},   {6.354, -0.45},
      {-6.354, 8.43},    {-4.548, -2.87}, {-2.732, 2.52},    {-0.911, -6.17},  {0.911, -1.66},
      {2.732, 3.73},     {4.548, -4.96},  {6.354, 0.43},
  };
  raw_calib_data[ModelId::HDL32E] = {
      {0, -30.67}, {0, -9.33},  {0, -29.33}, {0, -8},     {0, -28},    {0, -6.67},  {0, -26.67},
      {0, -5.33},  {0, -25.33}, {0, -4},     {0, -24},    {0, -2.67},  {0, -22.67}, {0, -1.33},
      {0, -21.33}, {0, 0},      {0, -20},    {0, 1.33},   {0, -18.67}, {0, 2.67},   {0, -17.33},
      {0, 4},      {0, -16},    {0, 5.33},   {0, -14.67}, {0, 6.67},   {0, -13.33}, {0, 8},
      {0, -12},    {0, 9.33},   {0, -10.67}, {0, 10.67},
  };
  raw_calib_data[ModelId::PuckHiRes] = {
      {0, -10}, {0, 0.667}, {0, -8.667}, {0, 2},  {0, -7.333}, {0, 3.333},
      {0, -6},  {0, 4.667}, {0, -4.667}, {0, 6},  {0, -3.333}, {0, 7.333},
      {0, -2},  {0, 8.667}, {0, -0.667}, {0, 10},
  };
  raw_calib_data[ModelId::VLP16] = {
      {0, -15}, {0, 1}, {0, -13}, {0, 3},  {0, -11}, {0, 5},  {0, -9}, {0, 7},
      {0, -7},  {0, 9}, {0, -5},  {0, 11}, {0, -3},  {0, 13}, {0, -1}, {0, 15},
  };
  raw_calib_data[ModelId::VLP32A] = {
      {-1.2, -14.3},  {1.2, 0.8337}, {-1.2, 12.5}, {1.2, -0.5},  {-1.2, -10.7},  {1.2, 2.167},
      {-1.2, 8.9},    {1.2, -1.834}, {-1.2, -7},   {3.6, 1.167}, {-1.2, 6},      {3.6, -0.167},
      {-1.2, -4.9},   {3.6, 2.5},    {-1.2, 3.8},  {3.6, -1.5},  {-1.2, -2.167}, {1.2, 10.7},
      {-1.2, 1.834},  {1.2, -8.9},   {-3.6, -2.5}, {1.2, 4.9},   {-3.6, 1.5},    {1.2, -3.8},
      {-1.2, -0.834}, {1.2, 14.3},   {-1.2, 0.5},  {1.2, -12.5}, {-3.6, -1.167}, {1.2, 7},
      {-3.6, 0.167},  {1.2, -6},
  };
  raw_calib_data[ModelId::VLP32C] = {
      {-1.4, -25},    {4.2, -1},     {-1.4, -1.667}, {1.4, -15.639}, {-1.4, -11.31}, {1.4, 0},
      {-4.2, -0.667}, {1.4, -8.843}, {-1.4, -7.254}, {4.2, 0.333},   {-1.4, -0.333}, {1.4, -6.148},
      {-4.2, -5.333}, {1.4, 1.333},  {-4.2, 0.667},  {1.4, -4},      {-1.4, -4.667}, {4.2, 1.667},
      {-1.4, 1},      {4.2, -3.667}, {-4.2, -3.333}, {1.4, 3.333},   {-1.4, 2.333},  {1.4, -2.667},
      {-1.4, -3},     {1.4, 7},      {-1.4, 4.667},  {4.2, -2.333},  {-4.2, -2},     {1.4, 15},
      {-1.4, 10.333}, {1.4, -1.333},
  };

  for (const auto &[model, data] : raw_calib_data) {
    std::vector<LaserCorrection> laser_corrections(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      auto &corr           = laser_corrections[i];
      corr.rot_correction  = (float)(data[i].rot_corr_deg * M_PI / 180.0);
      corr.vert_correction = (float)(data[i].vert_corr_deg * M_PI / 180.0);
      // The vertical offset correction is not provided for some of the models,
      // but it can be approximated from the beam angle and focal distance.
      // It will still be more accurate than not correcting for the offset at all.
      corr.vert_offset_correction = focal_distances[model] * std::tan(-corr.vert_correction);
      corr.laser_idx              = i;
    }
    // Apply pre-defined vertical offset corrections, if available.
    if (vertical_offsets.find(model) != vertical_offsets.end()) {
      const auto &offsets = vertical_offsets[model];
      for (size_t i = 0; i < data.size(); i++) {
        laser_corrections[i].vert_offset_correction = offsets[i];
      }
    }
    calibrations_.emplace(model, Calibration{laser_corrections, resolutions[model]});
  }
  // VLP-32B is identical to VLP-32C
  calibrations_.emplace(ModelId::VLP32B, calibrations_[ModelId::VLP32C]);
}

Calibration CalibDB::getDefaultCalibration(ModelId model_id) const {
  auto it = calibrations_.find(model_id);
  if (it == calibrations_.end()) {
    throw std::runtime_error("No calibration found for model " + std::to_string((int)model_id));
  }
  return it->second;
}

std::vector<ModelId> CalibDB::getAvailableModels() const {
  std::vector<ModelId> models;
  for (const auto &[model, calib] : calibrations_) {
    models.push_back(model);
  }
  return models;
}

} // namespace velodyne_decoder
