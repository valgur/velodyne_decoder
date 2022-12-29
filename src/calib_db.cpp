#include <cmath>
#include <map>
#include <string>

#include "velodyne_decoder/calibration.h"

namespace velodyne_decoder {

namespace {
struct MinimalCalibData {
  float rot_corr_deg;
  float vert_corr_deg;
};
struct CompactCalibData {
  float rot_correction_deg;
  float vert_correction_deg;
  float dist_correction;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  float focal_distance;
  float focal_slope;
};
} // namespace

CalibDB::CalibDB() {
  std::map<ModelId, float> resolutions = {
      {ModelId::AlphaPrime, 0.004}, {ModelId::HDL32E, 0.002}, {ModelId::PuckHiRes, 0.002},
      {ModelId::VLP16, 0.002},      {ModelId::VLP32A, 0.004}, {ModelId::VLP32C, 0.004},
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

  std::vector<CompactCalibData> hdl_64e_data = {
      {-4.55, -6.29, 1.238, 1.281, 1.251, 0.215, 0.026, 17.0, 1.2},
      {-2.46, -5.89, 1.39, 1.408, 1.408, 0.214, -0.026, 24.0, 1.1},
      {3.3, 1.15, 1.369, 1.389, 1.373, 0.205, 0.026, 23.0, 0.9},
      {5.52, 1.52, 1.359, 1.379, 1.385, 0.205, -0.026, 24.0, 1.3},
      {-0.23, -5.61, 1.271, 1.331, 1.266, 0.214, 0.026, 24.0, 1.3},
      {2.05, -5.29, 1.425, 1.445, 1.442, 0.213, -0.026, 24.0, 1.3},
      {-1.01, -7.63, 1.316, 1.381, 1.364, 0.216, 0.026, 16.0, 0.4},
      {1.22, -7.26, 1.333, 1.352, 1.334, 0.216, -0.026, 16.5, 1.3},
      {4.05, -4.96, 1.295, 1.339, 1.306, 0.213, 0.026, 24.0, 1.3},
      {6.29, -4.57, 1.375, 1.404, 1.409, 0.212, -0.026, 21.5, 1.1},
      {3.26, -7.0, 1.238, 1.301, 1.291, 0.215, 0.026, 20.0, 0.4},
      {5.54, -6.64, 1.328, 1.363, 1.394, 0.215, -0.026, 14.0, 1.2},
      {-4.55, -2.22, 1.318, 1.348, 1.331, 0.209, 0.026, 19.5, 1.1},
      {-2.28, -1.84, 1.43, 1.434, 1.441, 0.209, -0.026, 24.0, 1.3},
      {-5.31, -4.26, 1.297, 1.337, 1.277, 0.212, 0.026, 17.0, 1.2},
      {-3.09, -3.84, 1.316, 1.331, 1.326, 0.211, -0.026, 23.0, 0.9},
      {-0.26, -1.55, 1.299, 1.306, 1.287, 0.209, 0.026, 24.0, 1.3},
      {2.01, -1.18, 1.433, 1.437, 1.45, 0.208, -0.026, 24.0, 1.2},
      {-1.04, -3.58, 1.322, 1.362, 1.327, 0.211, 0.026, 24.0, 1.3},
      {1.21, -3.2, 1.397, 1.401, 1.398, 0.211, -0.026, 24.0, 1.3},
      {4.01, -0.88, 1.295, 1.314, 1.29, 0.208, 0.026, 24.0, 1.3},
      {6.23, -0.52, 1.288, 1.316, 1.334, 0.207, -0.026, 18.5, 1.1},
      {3.23, -2.91, 1.319, 1.347, 1.308, 0.21, 0.026, 24.0, 1.3},
      {5.46, -2.53, 1.386, 1.4, 1.42, 0.21, -0.026, 24.0, 1.3},
      {-4.58, 1.86, 1.326, 1.363, 1.319, 0.204, 0.026, 22.5, 0.9},
      {-2.32, 2.19, 1.409, 1.418, 1.402, 0.204, -0.026, 24.0, 1.2},
      {-5.35, -0.15, 1.367, 1.389, 1.358, 0.207, 0.026, 24.0, 1.3},
      {-3.09, 0.23, 1.381, 1.372, 1.368, 0.206, -0.026, 21.0, 1.1},
      {-0.28, 2.51, 1.353, 1.389, 1.383, 0.203, 0.026, 16.5, 0.9},
      {1.95, 2.81, 1.305, 1.332, 1.327, 0.203, -0.026, 24.0, 0.9},
      {-1.1, 0.48, 1.373, 1.383, 1.352, 0.206, 0.026, 24.0, 1.3},
      {1.18, 0.89, 1.385, 1.403, 1.396, 0.205, -0.026, 24.0, 1.3},
      {-7.45, -21.77, 1.207, 1.244, 1.229, 0.159, 0.026, 11.0, 1.8},
      {-3.99, -21.28, 1.304, 1.361, 1.379, 0.158, -0.026, 0.25, 1.0},
      {5.01, -10.67, 1.421, 1.449, 1.476, 0.146, 0.026, 0.25, 1.1},
      {8.21, -9.99, 1.291, 1.317, 1.336, 0.145, -0.026, 7.5, 1.5},
      {-0.51, -20.91, 1.181, 1.293, 1.242, 0.158, 0.026, 8.5, 1.4},
      {2.96, -20.34, 1.233, 1.288, 1.278, 0.157, -0.026, 9.5, 1.5},
      {-1.82, -23.95, 1.378, 1.416, 1.395, 0.162, 0.026, 3.5, 1.2},
      {1.69, -23.44, 1.225, 1.257, 1.277, 0.161, -0.026, 0.25, 1.1},
      {6.43, -19.74, 1.375, 1.427, 1.438, 0.156, 0.026, 2.0, 1.1},
      {9.79, -19.14, 1.226, 1.25, 1.265, 0.156, -0.026, 9.0, 1.8},
      {5.23, -22.82, 1.353, 1.385, 1.381, 0.16, 0.026, 0.25, 1.1},
      {8.7, -22.27, 1.087, 1.131, 1.13, 0.16, -0.026, 8.5, 1.5},
      {-7.27, -15.58, 1.228, 1.292, 1.239, 0.151, 0.026, 11.0, 1.9},
      {-3.8, -15.15, 1.269, 1.32, 1.339, 0.151, -0.026, 8.0, 0.7},
      {-8.63, -18.72, 1.396, 1.442, 1.419, 0.155, 0.026, 11.0, 1.9},
      {-5.1, -18.19, 1.186, 1.253, 1.258, 0.155, -0.026, 0.25, 0.7},
      {-0.46, -14.72, 1.208, 1.247, 1.229, 0.15, 0.026, 14.0, 1.3},
      {2.86, -14.16, 1.257, 1.277, 1.306, 0.15, -0.026, 12.0, 1.6},
      {-1.75, -17.77, 1.204, 1.266, 1.249, 0.154, 0.026, 13.0, 1.7},
      {1.69, -17.3, 1.209, 1.241, 1.232, 0.153, -0.026, 18.0, 0.9},
      {6.13, -13.73, 1.174, 1.215, 1.233, 0.149, 0.026, 0.25, 0.9},
      {9.49, -13.13, 1.282, 1.27, 1.326, 0.149, -0.026, 10.0, 1.9},
      {5.01, -16.71, 1.366, 1.431, 1.416, 0.153, 0.026, 3.0, 1.0},
      {8.43, -16.2, 1.218, 1.226, 1.231, 0.152, -0.026, 11.5, 2.0},
      {-7.0, -9.37, 1.294, 1.323, 1.316, 0.144, 0.026, 8.5, 1.7},
      {-3.76, -9.15, 1.317, 1.333, 1.363, 0.144, -0.026, 0.25, 1.3},
      {-8.3, -12.37, 1.442, 1.419, 1.429, 0.148, 0.026, 10.0, 2.0},
      {-4.98, -12.05, 1.258, 1.278, 1.298, 0.147, -0.026, 0.25, 0.8},
      {-0.45, -8.65, 1.439, 1.477, 1.454, 0.144, 0.026, 5.0, 1.4},
      {2.65, -8.11, 1.298, 1.341, 1.354, 0.143, -0.026, 3.5, 1.3},
      {-1.71, -11.59, 1.392, 1.425, 1.396, 0.147, 0.026, 11.5, 1.3},
      {1.54, -11.12, 1.268, 1.299, 1.288, 0.146, -0.026, 10.0, 1.0},
  };

  for (const auto &[model, data] : raw_calib_data) {
    std::vector<LaserCorrection> laser_corrections(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      auto &corr           = laser_corrections[i];
      corr.rot_correction  = (float)(data[i].rot_corr_deg * M_PI / 180.0);
      corr.vert_correction = (float)(data[i].vert_corr_deg * M_PI / 180.0);
      corr.laser_idx       = i;
    }
    calibrations_.emplace(model, Calibration{laser_corrections, resolutions[model]});
  }
  // VLP-32B is identical to VLP-32C
  calibrations_.emplace(ModelId::VLP32B, calibrations_[ModelId::VLP32C]);

  std::vector<LaserCorrection> hdl_64e_laser_corrs(64);
  for (size_t i = 0; i < hdl_64e_data.size(); i++) {
    auto &corr                   = hdl_64e_laser_corrs[i];
    auto &data                   = hdl_64e_data[i];
    corr.rot_correction          = (float)(data.rot_correction_deg * M_PI / 180.0);
    corr.vert_correction         = (float)(data.vert_correction_deg * M_PI / 180.0);
    corr.dist_correction         = data.dist_correction;
    corr.dist_correction_x       = data.dist_correction_x;
    corr.dist_correction_y       = data.dist_correction_y;
    corr.vert_offset_correction  = data.vert_offset_correction;
    corr.horiz_offset_correction = data.horiz_offset_correction;
    corr.focal_distance          = data.focal_distance;
    corr.focal_slope             = data.focal_slope;
    corr.laser_idx               = i;
  }
  // TODO: this calibration is probably only applicable to one of the HDL-64E versions
  calibrations_.emplace(ModelId::HDL64E_S1, Calibration{hdl_64e_laser_corrs, 0.002f});
  calibrations_.emplace(ModelId::HDL64E_S2, Calibration{hdl_64e_laser_corrs, 0.002f});
  calibrations_.emplace(ModelId::HDL64E_S3, Calibration{hdl_64e_laser_corrs, 0.002f});
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
