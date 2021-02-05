#pragma once

namespace velodyne_decoder {

struct Config {
  std::string model;
  std::string calibrationFile; ///< calibration file name
  float min_range;             ///< minimum range to publish
  float max_range;             ///< maximum range to publish
  int min_angle;               ///< minimum angle to publish
  int max_angle;               ///< maximum angle to publish
};

} // namespace velodyne_decoder
