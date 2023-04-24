// Copyright (c) 2007, Austin Robot Technology, Patrick Beeson
// Copyright (c) 2009, 2010, 2012, Austin Robot Technology, Jack O'Quin
// Copyright (c) 2019, Kaarta Inc, Shawn Hanna
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/packet_decoder.h"
#include "velodyne_decoder/types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace velodyne_decoder {

namespace {
template <typename T> constexpr T SQR(T val) { return val * val; }
uint32_t wrap(int azimuth) { return (uint16_t)((azimuth + 36000) % 36000); }
uint32_t wrap(float azimuth) { return (uint16_t)(std::lround(azimuth + 36000) % 36000); }
} // namespace

PacketDecoder::PacketDecoder(const Config &config)
    : single_return_mode_info_(config.single_return_mode_info) {
  if (config.calibration.has_value()) {
    initCalibration(*config.calibration);
  }
  if (config.model.has_value()) {
    initModel(*config.model);
  }
  min_range_   = config.min_range;
  max_range_   = config.max_range;
  min_azimuth_ = std::lround(std::fmod(std::fmod(config.min_angle, 360) + 360, 360) * 100);
  max_azimuth_ = std::lround(std::fmod(std::fmod(config.max_angle, 360) + 360, 360) * 100);
  if (min_azimuth_ == max_azimuth_) {
    min_azimuth_ = 0;
    max_azimuth_ = 36000;
  }
  setupSinCosCache();
}

std::optional<ModelId> PacketDecoder::modelId() const { return model_id_; }

std::optional<DualReturnMode> PacketDecoder::returnMode() const { return return_mode_; }

void PacketDecoder::initModel(PacketModelId packet_model_id) {
  switch (packet_model_id) {
  case PacketModelId::HDL32E:
    return initModel(ModelId::HDL32E);
  case PacketModelId::VLP16:
    return initModel(ModelId::VLP16);
  case PacketModelId::VLP32AB:
    throw std::runtime_error("The model ID is ambiguous: either VLP-32A or VLP-32B based on the "
                             "data. Please specify the model explicitly.");
  case PacketModelId::VLP16HiRes:
    return initModel(ModelId::PuckHiRes);
  case PacketModelId::VLP32C:
    return initModel(ModelId::VLP32C);
  case PacketModelId::Velarray:
    throw std::runtime_error("The data contains packets from Velodyne Velarray, "
                             "which is not supported.");
  case PacketModelId::VLS128:
    return initModel(ModelId::VLS128);
  }
  throw std::runtime_error("Data from an unsupported Velodyne model. Model ID in packet: " +
                           std::to_string((int)packet_model_id));
}

void PacketDecoder::initModel(ModelId model_id) {
  if (std::find(Config::SUPPORTED_MODELS.begin(), Config::SUPPORTED_MODELS.end(), model_id) ==
      Config::SUPPORTED_MODELS.end()) {
    throw std::runtime_error("Unsupported Velodyne model ID: " + std::to_string((int)model_id));
  }
  model_id_       = model_id;
  timing_offsets_ = buildTimings(model_id);
  if (!calib_initialized_) {
    Calibration calib = CalibDB().getDefaultCalibration(model_id);
    initCalibration(calib);
  }
}

void PacketDecoder::verifyPacketModelId(PacketModelId packet_model_id, ModelId model_id) {
  PacketModelId expected;
  switch (model_id) {
  case ModelId::HDL64E_S1:
  case ModelId::HDL64E_S2:
  case ModelId::HDL64E_S3:
    return; // HDL-64E does not set the model ID in the packet
  case ModelId::HDL32E:
    expected = PacketModelId::HDL32E;
    break;
  case ModelId::VLP32A:
  case ModelId::VLP32B:
    expected = PacketModelId::VLP32AB;
    break;
  case ModelId::VLP32C:
    expected = PacketModelId::VLP32C;
    break;
  case ModelId::VLP16:
    expected = PacketModelId::VLP16;
    break;
  case ModelId::PuckHiRes:
    expected = PacketModelId::VLP16HiRes;
    break;
  case ModelId::VLS128:
    expected = PacketModelId::VLS128;
    break;
  default:
    throw std::runtime_error("Unknown model ID: " + std::to_string((int)model_id));
  }
  if (expected != packet_model_id) {
    throw std::runtime_error(
        "Expected model ID <" + std::to_string((int)expected) + "> in packet, got <" +
        std::to_string((int)packet_model_id) +
        ">. Note: if the device is HDL-64E, please set the model in Config explicitly.");
  }
}

void PacketDecoder::initCalibration(const Calibration &calibration) {
  calibration_       = calibration;
  calib_initialized_ = true;
  setupCalibrationCache(calibration_);
}

bool PacketDecoder::distanceInRange(float range) const {
  return range >= min_range_ && range <= max_range_;
}

bool PacketDecoder::azimuthInRange(uint16_t azimuth) const {
  if (min_azimuth_ <= max_azimuth_) {
    return azimuth >= min_azimuth_ && azimuth <= max_azimuth_;
  } else {
    return azimuth <= min_azimuth_ && azimuth >= max_azimuth_;
  }
}

/**
 * Build a timing table for each block/firing.
 */
std::array<std::array<float, 32>, 12> PacketDecoder::buildTimings(ModelId model) {
  std::array<std::array<float, 32>, 12> timing_offsets = {};
  // timing table calculation, based on Velodyne user manuals
  if (model == ModelId::HDL64E_S1) {
    // S1 has no details about timing info provided:
    // https://usermanual.wiki/Pdf/HDL64E20Manual.196042455/view
    // The timing info was estimated from on block azimuth values in this dataset instead:
    // http://download.ros.org/data/velodyne/class.pcap
    // The timing values are calculated on the fly in the decoding function because the timings
    // do not appear to have a fixed pattern.
  } else if (model == ModelId::HDL64E_S2) {
    // https://www.termocam.it/pdf/manuale-HDL-64E.pdf#page=38
    // Fill the timing info based on the "Laser firing time table" in the manual
    const std::array<double, 4> pattern = {0., 1.26, 2.46, 3.66};
    for (uint16_t block_idx = 0; block_idx < 12; ++block_idx) {
      for (uint16_t laser_idx = 0; laser_idx < 32; ++laser_idx) {
        double offset = 48.0 * (block_idx / 2) + 6.0 * (laser_idx / 4) + pattern[laser_idx % 4];
        offset *= 1e-6;
        timing_offsets[block_idx][laser_idx] = (float)offset;
      }
    }
  } else if (model == ModelId::HDL64E_S3) {
    // https://www.researchgate.net/profile/Joerg-Fricke/post/How_the_LiDARs_photodetector_distinguishes_lasers_returns/attachment/5fa947b8543da600017dcf9b/AS%3A955957442002980%401604929423693/download/HDL-64E_S3_UsersManual.pdf#page=48
    // Fill the timing info based on the "Laser firing time table" in the manual
    const std::array<double, 4> pattern = {0, 1.3, 2.5, 3.7};
    for (uint16_t block_idx = 0; block_idx < 12; ++block_idx) {
      for (uint16_t laser_idx = 0; laser_idx < 32; ++laser_idx) {
        double offset = 57.6 * (block_idx / 2) + 7.2 * (laser_idx / 4) + pattern[laser_idx % 4];
        offset *= 1e-6;
        timing_offsets[block_idx][laser_idx] = (float)offset;
      }
    }
  } else if (model == ModelId::HDL32E) {
    // https://velodynelidar.com/wp-content/uploads/2019/09/63-9277-Rev-D-HDL-32E-Application-Note-Packet-Structure-Timing-Definition.pdf#page=20
    // https://www.termocam.it/pdf/manuale-HDL-32E.pdf#page=12
    // 1.152 μs x 32 firings + 9.216 μs recharge time = 46.080 μs full cycle
    const double full_firing_cycle = 46.080 * 1e-6; // seconds
    const double single_firing     = 1.152 * 1e-6;  // seconds
    // compute timing offsets
    for (size_t i = 0; i < 12; ++i) {
      for (size_t j = 0; j < 32; ++j) {
        uint16_t block_index  = i;
        uint16_t firing_index = j;
        timing_offsets[i][j] =
            (float)(full_firing_cycle * block_index + single_firing * firing_index);
      }
    }
  } else if (model == ModelId::VLP16 || model == ModelId::PuckHiRes) {
    // https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf#page=50
    // Puck Hi-Res is the same as VLP-16 but with a tighter vertical angle distribution.
    // 2.304 μs x 16 firings + 18.43 μs recharge time = 55.296 μs full cycle
    const double full_firing_cycle = 55.296 * 1e-6; // seconds
    const double single_firing     = 2.304 * 1e-6;  // seconds
    const int num_lasers           = 16;
    for (size_t i = 0; i < 12; ++i) {
      for (size_t j = 0; j < 32; ++j) {
        uint16_t block_index = 2 * i + j / num_lasers;
        uint16_t point_index = j % num_lasers;
        timing_offsets[i][j] =
            (float)(full_firing_cycle * block_index + single_firing * point_index);
      }
    }
  } else if (model == ModelId::VLP32C || model == ModelId::VLP32B || model == ModelId::VLP32A) {
    // VLP-32C: https://icave2.cse.buffalo.edu/resources/sensor-modeling/VLP32CManual.pdf#page=49
    // No manual found for VLP-32A and VLP-32B.
    // VLP-32C and VLP-32B likely have the same timings since their calibrations are identical.
    // FIXME: I'm guessing the timings are the same for VLP-32A as well.
    // 2.304 μs x 16 firings + 18.43 μs recharge time = 55.296 μs full cycle
    const double full_firing_cycle = 55.296 * 1e-6; // seconds
    const double single_firing     = 2.304 * 1e-6;  // seconds
    // compute timing offsets
    for (size_t i = 0; i < 12; ++i) {
      for (size_t j = 0; j < 32; ++j) {
        uint16_t block_index  = i;
        uint16_t firing_index = j / 2;
        timing_offsets[block_index][j] =
            (float)(full_firing_cycle * block_index + single_firing * firing_index);
      }
    }
  } else if (model == ModelId::AlphaPrime) {
    // Based on the implementation in VeloView's VelodynePlugin
    // https://gitlab.kitware.com/LidarView/velodyneplugin/-/blob/releasev5.1.0/Plugin/VelodyneLidar/VelodynePacketInterpreter/vtkVelodyneLegacyPacketInterpreter.cxx#L117-142
    // Based on VeloView, the channel timings are:
    // - 2.66 usec in firmware v5.1.7.1 (Rev3 of the manual)
    // - 2.89 usec since firmware v5.2.3.0 (Rev4 of the manual).
    // The difference is automatically accounted for correctVls128Timings().
    // Channel duration. Channels correspond to a group of 8 lasers firing.
    const double single_firing = default_vls_128_channel_duration_;
    // Sequence duration. Sequence is a set of laser group firings including recharging.
    const double full_firing_cycle = 20 * single_firing;
    // ToH adjustment. Top of the Hour is aligned with the fourth firing group in a firing sequence.
    const double offset_packet_time = 7 * 1e-6;
    // Compute timing offsets
    for (size_t i = 0; i < 12; ++i) {
      for (size_t j = 0; j < 32; ++j) {
        uint16_t dsr         = 32 * (i % 4) + j;
        timing_offsets[i][j] = (float)((i / 4) * full_firing_cycle                //
                                       + (dsr / 8 + dsr / 64 * 2) * single_firing //
                                       - offset_packet_time);
      }
    }
  } else {
    throw std::runtime_error("Requested timings for an unknown model");
  }
  return timing_offsets;
}

void PacketDecoder::setupSinCosCache() {
  // Set up cached values for sin and cos of all the possible headings
  constexpr float deg2rad = M_PI / 180.f;
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation            = ROTATION_RESOLUTION * rot_index * deg2rad;
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
}

void PacketDecoder::setupCalibrationCache(const Calibration &calibration) {
  apply_advanced_calibration_ = calibration.isAdvancedCalibration();
  cos_rot_correction_.resize(calibration.num_lasers);
  sin_rot_correction_.resize(calibration.num_lasers);
  cos_vert_correction_.resize(calibration.num_lasers);
  sin_vert_correction_.resize(calibration.num_lasers);
  ring_cache_.resize(calibration.num_lasers);
  for (int i = 0; i < calibration.num_lasers; i++) {
    cos_rot_correction_[i]  = cosf(calibration.laser_corrections[i].rot_correction);
    sin_rot_correction_[i]  = sinf(calibration.laser_corrections[i].rot_correction);
    cos_vert_correction_[i] = cosf(calibration.laser_corrections[i].vert_correction);
    sin_vert_correction_[i] = sinf(calibration.laser_corrections[i].vert_correction);
    ring_cache_[i]          = calibration.laser_corrections[i].laser_ring;
  }
}

/** @brief convert raw packet to point cloud
 */
void PacketDecoder::unpack(const VelodynePacket &pkt, PointCloud &cloud, Time scan_start_time) {
  const raw_packet_t &raw_packet = *reinterpret_cast<const raw_packet_t *>(pkt.data.data());
  unpack(pkt.stamp, raw_packet, cloud, scan_start_time);
}

void PacketDecoder::unpack(Time stamp, const raw_packet_t &raw_packet, PointCloud &cloud,
                           Time scan_start_time) {
  if (model_id_.has_value()) {
    verifyPacketModelId(raw_packet.model_id, *model_id_);
  } else {
    initModel(raw_packet.model_id);
  }

  // timestamp of the first point in the packet relative to the start of the scan
  float rel_packet_stamp = (float)(stamp - scan_start_time);

  switch (*model_id_) {
  case ModelId::VLP16:
  case ModelId::PuckHiRes:
  case ModelId::HDL32E:
  case ModelId::VLP32A:
  case ModelId::VLP32B:
  case ModelId::VLP32C:
    return unpack_16_32_beam(raw_packet, rel_packet_stamp, cloud);
  case ModelId::HDL64E_S1:
    return unpack_hdl64e_s1(raw_packet, rel_packet_stamp, cloud);
  case ModelId::HDL64E_S2:
  case ModelId::HDL64E_S3:
    return unpack_hdl64e(raw_packet, rel_packet_stamp, cloud);
  case ModelId::AlphaPrime:
    return unpack_vls128(raw_packet, rel_packet_stamp, cloud);
  default:
    throw std::runtime_error("Unsupported Velodyne model ID: " + std::to_string((int)*model_id_));
  }
}

/** @brief convert raw VLP-16, HDL-32E or VLP-32A/B/C packet to point cloud
 */
void PacketDecoder::unpack_16_32_beam(const raw_packet_t &raw, float rel_packet_stamp,
                                      PointCloud &cloud) {
  // Note: for HDL-32E, this field is only set since firmware version 2.2.20.0, 2016-02-02.
  return_mode_     = raw.return_mode;
  bool dual_return = raw.return_mode == DualReturnMode::DUAL_RETURN;

  // Calculate the average rotation rate for this packet
  uint16_t azimuth_diff =
      wrap((int)raw.blocks[BLOCKS_PER_PACKET - 1].rotation - (int)raw.blocks[0].rotation);
  float duration =
      timing_offsets_[dual_return ? (BLOCKS_PER_PACKET - 1) / 2 : (BLOCKS_PER_PACKET - 1)][0] -
      timing_offsets_[0][0];
  float rotation_rate = (float)azimuth_diff / duration;

  const int block_step = dual_return ? 2 : 1;
  for (int i = 0; i < BLOCKS_PER_PACKET; i += block_step) {
    const auto &block = raw.blocks[i];

    // ignore packets with mangled or otherwise different contents
    if (block.bank_id != LaserBankId::BANK_0) {
      return; // bad packet: skip the rest
    }

    uint16_t block_azimuth = block.rotation;

    if (!dual_return) {
      // single return mode
      float t0 = timing_offsets_[i][0];
      for (int j = 0; j < SCANS_PER_BLOCK; j++) {
        const auto &measurement = block.data[j];
        if (measurement.distance == 0)
          continue;

        // correct for the laser rotation as a function of timing during the firings
        float time       = timing_offsets_[i][j];
        float dt         = time - t0;
        uint16_t azimuth = wrap(block_azimuth + rotation_rate * dt);
        if (!azimuthInRange(azimuth))
          continue;

        float full_time = rel_packet_stamp + time;
        int laser_idx   = calibration_.num_lasers == 16 && j >= 16 ? j - 16 : j;
        unpackPoint(cloud, laser_idx, azimuth, full_time, measurement, SINGLE_RETURN_FLAG);
      }
    } else {
      // dual return mode
      float t0 = timing_offsets_[i / 2][0];
      for (int j = 0; j < SCANS_PER_BLOCK; j++) {
        const auto &last      = raw.blocks[i].data[j];
        const auto &strongest = raw.blocks[i + 1].data[j];
        if (last.distance == 0 && strongest.distance == 0)
          continue;

        // correct for the laser rotation as a function of timing during the firings
        float time       = timing_offsets_[i / 2][j];
        float dt         = time - t0;
        uint16_t azimuth = wrap(block_azimuth + rotation_rate * dt);
        if (!azimuthInRange(azimuth))
          continue;

        float full_time = rel_packet_stamp + time;
        int laser_idx   = calibration_.num_lasers == 16 && j >= 16 ? j - 16 : j;
        unpackPointDual(cloud, laser_idx, azimuth, full_time, last, strongest);
      }
    }
  }
}

/** @brief convert raw HDL-64E S1 packet to point cloud
 */
void PacketDecoder::unpack_hdl64e_s1(const raw_packet_t &raw, float rel_packet_stamp,
                                     PointCloud &cloud) {
  // Calculate the time durations for each block.
  // This is estimated from on the azimuth values of the blocks in sample data.
  // The block durations appear to vary between either 25 usec or 50 usec,
  // with the longer one occurring approximately every 3.5 blocks with no clear pattern.
  return_mode_ = DualReturnMode::STRONGEST_RETURN;

  // Calculate azimuth deltas for each block.
  if (prev_packet_azimuth_ > 36000) {
    prev_packet_azimuth_ = raw.blocks.front().rotation;
  }
  std::array<int, BLOCKS_PER_PACKET> delta_az;
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    int prev    = i == 0 ? prev_packet_azimuth_ : raw.blocks[i - 1].rotation;
    delta_az[i] = wrap((int)raw.blocks[i].rotation - prev);
  }
  prev_packet_azimuth_ = raw.blocks.back().rotation;

  // Calculate the pattern of longer durations based on azimuth deltas.
  uint16_t min_delta_az = *std::min_element(delta_az.begin() + 1, delta_az.end());
  std::array<bool, BLOCKS_PER_PACKET> timing_pattern = {false};
  for (int i = 0; i < BLOCKS_PER_PACKET - 1; i++) {
    // true, if twice as long as the shortest azimuth delta
    timing_pattern[i] = delta_az[i + 1] > min_delta_az * 3 / 2;
  }
  // Would have to know the next packet's first azimuth to calculate the duration of the last block.
  // Estimate this from the previous durations instead.
  // 97% of the time, the next duration is longer if only one of the previous six ones is.
  timing_pattern[BLOCKS_PER_PACKET - 1] =
      (timing_pattern[5] + timing_pattern[6] + timing_pattern[7] + //
       timing_pattern[8] + timing_pattern[9] + timing_pattern[10]) == 1;

  constexpr float cycle_duration = 24.81236 * 1e-6; // estimated from data
  std::array<float, BLOCKS_PER_PACKET> block_durations;
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    block_durations[i] = (float)(timing_pattern[i] + 1) * cycle_duration;
    // subtract from the packet stamp since the stamp is always the packet arrival time for S1
    rel_packet_stamp -= block_durations[i];
  }

  // Calculate the average rotation rate for this packet
  float rotation_rate = 0;
  for (int i = 1; i < BLOCKS_PER_PACKET - 1; i++) {
    rotation_rate += (float)delta_az[i] / block_durations[i];
  }
  rotation_rate /= BLOCKS_PER_PACKET - 2;

  float block_time = 0;
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    const auto &block = raw.blocks[i];

    uint16_t block_azimuth = block.rotation;
    if (!azimuthInRange(block_azimuth))
      continue;

    int bank_origin = (block.bank_id == LaserBankId::BANK_0) ? 0 : 32;

    for (int j = 0; j < SCANS_PER_BLOCK; j++) {
      const auto &measurement = block.data[j];
      if (measurement.distance == 0)
        continue;

      // Assume that the firings are distributed evenly across the block,
      // which is probably not the case but should be close enough.
      float dt         = block_durations[i] * (j / 32.0f);
      uint16_t azimuth = wrap(block_azimuth + rotation_rate * dt);

      float full_time            = rel_packet_stamp + block_time + dt;
      const uint8_t laser_number = j + bank_origin;
      unpackPoint(cloud, laser_number, azimuth, full_time, measurement, SINGLE_RETURN_FLAG);
    }
    block_time += block_durations[i];
  }
}

/** @brief convert raw HDL-64E S2/S3 packet to point cloud
 */
void PacketDecoder::unpack_hdl64e(const raw_packet_t &raw, float rel_packet_stamp,
                                  PointCloud &cloud) {
  // HDL-64E does not have a separate packet field for dual return mode info.
  // Estimating this from azimuth values instead.
  bool dual_return = raw.blocks[0].rotation == raw.blocks[2].rotation;
  return_mode_     = dual_return ? DualReturnMode::DUAL_RETURN : DualReturnMode::STRONGEST_RETURN;

  // Calculate the average rotation rate for this packet
  uint16_t azimuth_diff =
      wrap((int)raw.blocks[BLOCKS_PER_PACKET - 1].rotation - (int)raw.blocks[0].rotation);
  float duration =
      timing_offsets_[(BLOCKS_PER_PACKET - 1) / (dual_return ? 2 : 1)][0] - timing_offsets_[0][0];
  float rotation_rate = (float)azimuth_diff / duration;

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    const auto &block = raw.blocks[i];

    uint16_t block_azimuth = block.rotation;
    if (!azimuthInRange(block_azimuth))
      continue;

    int bank_origin = (block.bank_id == LaserBankId::BANK_0) ? 0 : 32;

    if (!dual_return) {
      // The time of the first firing in a column
      float t0 = timing_offsets_[i / 2 * 2][0];
      for (int j = 0; j < SCANS_PER_BLOCK; j++) {
        const auto &measurement = block.data[j];
        if (measurement.distance == 0)
          continue;
        float time                 = timing_offsets_[i][j];
        float dt                   = time - t0;
        uint16_t azimuth           = wrap(block_azimuth + rotation_rate * dt);
        float full_time            = rel_packet_stamp + time;
        const uint8_t laser_number = j + bank_origin;
        unpackPoint(cloud, laser_number, azimuth, full_time, measurement, SINGLE_RETURN_FLAG);
      }
    } else {
      // dual mode blocks correspond to single mode indices with the following pattern:
      // 0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5
      if (i % 4 > 1)
        continue;
      float t0 = timing_offsets_[i / 4 * 2][0];
      for (int j = 0; j < SCANS_PER_BLOCK; j++) {
        const auto &strongest = block.data[j];
        const auto &last      = raw.blocks[i + 2].data[j];
        if (strongest.distance == 0 && last.distance == 0)
          continue;
        float time                 = timing_offsets_[i / 4 * 2 + i % 2][j];
        float dt                   = time - t0;
        uint16_t azimuth           = wrap(block_azimuth + rotation_rate * dt);
        float full_time            = rel_packet_stamp + time;
        const uint8_t laser_number = j + bank_origin;
        unpackPointDual(cloud, laser_number, azimuth, full_time, last, strongest);
      }
    }
  }
}

/** @brief convert raw VLS-128 / Alpha Prime packet to point cloud
 */
void PacketDecoder::unpack_vls128(const raw_packet_t &raw, float rel_packet_stamp,
                                  PointCloud &cloud) {
  return_mode_     = raw.return_mode;
  bool dual_return = raw.return_mode == DualReturnMode::DUAL_RETURN;

  correctVls128Timings(raw.stamp, dual_return);

  // Calculate the average rotation rate for this packet
  float rotation_rate = 0;
  if (prev_packet_azimuth_ < 36000) {
    uint16_t azimuth_diff = wrap((int)raw.blocks[0].rotation - (int)prev_packet_azimuth_);
    // Packets in dual-return mode contain only a single column, 3 in standard mode.
    float sequence_duration = timing_offsets_[4][0] - timing_offsets_[0][0];
    float packet_duration   = dual_return ? sequence_duration : 3 * sequence_duration;
    rotation_rate           = (float)azimuth_diff / packet_duration;
    if (prev_rotation_rate_ > 0 && rotation_rate > prev_rotation_rate_ * 1.8f) {
      // A packet has been dropped inbetween the current and last packet.
      // Use the previous azimuth diff instead.
      rotation_rate = prev_rotation_rate_;
    } else {
      prev_rotation_rate_ = rotation_rate;
    }
  }
  prev_packet_azimuth_ = raw.blocks[0].rotation;

  // The last 4 blocks are empty in dual-return mode.
  int num_blocks = dual_return ? 8 : 12;
  int block_step = dual_return ? 2 : 1;

  for (int block = 0; block < num_blocks; block += block_step) {
    // cache block for use
    const raw_block_t &current_block = raw.blocks[block];

    uint16_t block_azimuth = current_block.rotation;
    if (!azimuthInRange(block_azimuth))
      continue;

    int bank_origin = 0;
    // Used to detect which bank of 32 lasers is in this block
    switch (current_block.bank_id) {
    case LaserBankId::BANK_0:
      bank_origin = 0;
      break;
    case LaserBankId::BANK_1:
      bank_origin = 32;
      break;
    case LaserBankId::BANK_2:
      bank_origin = 64;
      break;
    case LaserBankId::BANK_3:
      bank_origin = 96;
      break;
    default:
      return; // bad packet: skip the rest
    }

    if (!dual_return) {
      float t0 = timing_offsets_[block / 4 * 4][0];
      for (int j = 0; j < SCANS_PER_BLOCK; j++) {
        const auto &measurement = current_block.data[j];
        if (measurement.distance == 0)
          continue;
        // correct for the laser rotation as a function of timing during the firings
        float time           = timing_offsets_[block][j];
        float dt             = time - t0;
        uint16_t azimuth     = wrap(block_azimuth + rotation_rate * dt);
        float full_time      = rel_packet_stamp + time;
        uint8_t laser_number = bank_origin + j;
        unpackPoint(cloud, laser_number, azimuth, full_time, measurement, SINGLE_RETURN_FLAG);
      }
    } else {
      float t0 = timing_offsets_[0][0];
      for (int j = 0; j < SCANS_PER_BLOCK; j++) {
        const auto &last      = raw.blocks[block].data[j];
        const auto &strongest = raw.blocks[block + 1].data[j];
        if (last.distance == 0 && strongest.distance == 0)
          continue;
        float time           = timing_offsets_[block / 2][j];
        float dt             = time - t0;
        uint16_t azimuth     = wrap(block_azimuth + rotation_rate * dt);
        float full_time      = rel_packet_stamp + time;
        uint8_t laser_number = bank_origin + j;
        unpackPointDual(cloud, laser_number, azimuth, full_time, last, strongest);
      }
    }
  }
}

void PacketDecoder::correctVls128Timings(uint32_t stamp, bool dual_return) {
  if (vls_128_channel_duration_corrected_)
    return;
  if (!prev_packet_stamp_.has_value()) {
    prev_packet_stamp_ = stamp;
    return;
  }
  uint32_t packet_duration = (stamp - prev_packet_stamp_.value()) * (dual_return ? 3 : 1);
  prev_packet_stamp_       = stamp;
  // Based on VeloView, the channel durations are:
  // 2.66 usec in firmware v5.1.7.1 - 160 us packet duration
  // 2.89 usec since firmware v5.2.3.0 (Rev4 of the manual) - 173.3 us packet duration
  // The +/- 7 us margin is a bit of an overkill, since the packet time deltas were all exactly
  // 160 us in a sample dataset with older firmware.
  if (packet_duration > 153 && packet_duration < 180) {
    double channel_duration = packet_duration < 167 ? (8. / 3.) * 1e-6 : 2.89 * 1e-6;
    float multiplier        = (float)(channel_duration / default_vls_128_channel_duration_);
    for (auto &block_timings : timing_offsets_) {
      for (auto &timing : block_timings) {
        timing *= multiplier;
      }
    }
    vls_128_channel_duration_corrected_ = true;
  }
}

void PacketDecoder::unpackPointDual(PointCloud &cloud, int laser_idx, uint16_t azimuth, float time,
                                    const raw_measurement_t last,
                                    const raw_measurement_t strongest) const {
  if (last.distance == 0 && strongest.distance == 0)
    return;
  if (last.distance == strongest.distance) {
    unpackPoint(cloud, laser_idx, azimuth, time, strongest, BOTH_RETURN_FLAG);
  } else {
    if (last.distance > 0) {
      unpackPoint(cloud, laser_idx, azimuth, time, last, LAST_RETURN_FLAG);
    }
    if (strongest.distance > 0) {
      unpackPoint(cloud, laser_idx, azimuth, time, strongest, STRONGEST_RETURN_FLAG);
    }
  }
}

/** @brief Applies calibration, converts to a Cartesian 3D point and appends to the cloud.
 */
void PacketDecoder::unpackPoint(PointCloud &cloud, int laser_idx, uint16_t azimuth, float time,
                                const raw_measurement_t measurement,
                                ReturnModeFlag return_mode_flag) const {
  float distance = measurement.distance * calibration_.distance_resolution_m;

  float cos_vert_angle     = cos_vert_correction_[laser_idx];
  float sin_vert_angle     = sin_vert_correction_[laser_idx];
  float cos_rot_correction = cos_rot_correction_[laser_idx];
  float sin_rot_correction = sin_rot_correction_[laser_idx];

  // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
  // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
  float cos_rot_angle = cos_rot_table_[azimuth] * cos_rot_correction + //
                        sin_rot_table_[azimuth] * sin_rot_correction;
  float sin_rot_angle = sin_rot_table_[azimuth] * cos_rot_correction - //
                        cos_rot_table_[azimuth] * sin_rot_correction;

  // optionally include more detailed info about the return mode on single-return mode
  if (single_return_mode_info_ && return_mode_flag == SINGLE_RETURN_FLAG) {
    return_mode_flag =
        return_mode_ == DualReturnMode::LAST_RETURN ? LAST_RETURN_FLAG : STRONGEST_RETURN_FLAG;
  }

  if (!apply_advanced_calibration_) {
    if (!distanceInRange(distance))
      return;

    float xy_distance = distance * cos_vert_angle;
    // Use the standard ROS coordinate system (x - forward, y - left, z - up)
    float x = xy_distance * cos_rot_angle;
    float y = -xy_distance * sin_rot_angle;
    float z = distance * sin_vert_angle;

    uint16_t ring = ring_cache_[laser_idx] | return_mode_flag;

    cloud.emplace_back(x, y, z, (float)measurement.intensity, ring, time);

  } else {
    const auto &corrections = calibration_.laser_corrections[laser_idx];
    distance += corrections.dist_correction;

    if (!distanceInRange(distance))
      return;

    float horiz_offset = corrections.horiz_offset_correction;
    float vert_offset  = corrections.vert_offset_correction;

    // Get 2points calibration values, Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    float distance_corr_x = 0;
    float distance_corr_y = 0;
    if (corrections.two_pt_correction_available) {
      // Compute the distance in the xy plane (w/o accounting for rotation)
      float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

      float xx = std::abs(xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle);
      float yy = std::abs(xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle);

      distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                            (xx - 2.4f) / (25.04f - 2.4f) +
                        corrections.dist_correction_x;
      distance_corr_x -= corrections.dist_correction;
      distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                            (yy - 1.93f) / (25.04f - 1.93f) +
                        corrections.dist_correction_y;
      distance_corr_y -= corrections.dist_correction;
    }

    float distance_x  = distance + distance_corr_x;
    float xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
    /// the expression with '-' is proved to be better than the one with '+'
    float x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

    float distance_y = distance + distance_corr_y;
    xy_distance      = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
    float y          = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

    // Using distance_y is not symmetric, but the velodyne manual does this.
    float z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

    /** Use standard ROS coordinate system */
    float x_coord = y;
    float y_coord = -x;
    float z_coord = z;

    /** Intensity Calculation */
    float min_intensity = corrections.min_intensity;
    float max_intensity = corrections.max_intensity;

    float focal_offset  = SQR(1 - corrections.focal_distance / 13100.f);
    float raw_intensity = measurement.intensity;
    float raw_distance  = measurement.distance;
    float intensity     = raw_intensity + 256 * corrections.focal_slope *
                                          std::abs(focal_offset - SQR(1 - raw_distance / 65535.f));
    intensity = (intensity < min_intensity) ? min_intensity : intensity;
    intensity = (intensity > max_intensity) ? max_intensity : intensity;

    uint16_t ring = corrections.laser_ring | return_mode_flag;

    cloud.emplace_back(x_coord, y_coord, z_coord, intensity, ring, time);
  }
}

} // namespace velodyne_decoder
