# Changelog

## [3.0.0] – 2024-02-25

This release introduces significant improvements to the accuracy of the decoded data and ease of use of the library.

As a major release, it includes a few breaking changes:

* All config parameters are now optional and `rpm`, `gps_time` have been dropped.
* Timestamps are now returned as a pair of `host` and `device` timestamps.
* Two new fields have been added to the returned point clouds (`return_type` and `column`).

### General changes

* Sensor `model` and `rpm` parameters are detected automatically from the data. This means that only the raw data itself
  is required to use the decoder without any additional configuration parameters.
* Added support for dual-return data. A `return_type` field has been added to the decoded points to indicate the type of
  return:  1 - strongest, 2 - last, 3 - both. This info is also included for single-return data.
* Added a `column` field to the points, which provides the index of the column of simultaneous firings relative to the
  start of the scan.
* Timestamps are now always returned as a pair of `host` and `device` timestamps. This replaces the previous behavior of
  returning only one based on a config parameter.
* Added support for parsing of telemetry packets. This provides details about PPS time synchronization status, the time
  and position of last GNSS message, internal temperatures and more.
* Precise timing info is now available for all sensor models. All of the logic and parameters related to precise timing
  info have been thoroughly reviewed to match the official specifications and have been validated against the ground truth
  dataset provided with VeloView. The existing timing parameters were not modified in the process.
* Added a correction for a systematic error in the vertical direction for up to 2 cm, dependent on the beam angle. This
  correction accounts for each laser having a slight vertical offset instead of the previous implicit assumption of them
  all being located in the optical center. Related to https://github.com/ros-drivers/velodyne/pull/518.
* Added a `cut_angle` config parameter – when working with a raw packet stream, if unset (by default), the stream is
  split into a "scan" every time at least 360 degrees have been covered. If set, the splitting always occurs at the
  specified azimuth angle instead. Note that the scan might cover less than 360 degrees in this case.
* Fixed packets at the end of a file not being decoded if they didn't form a complete scan.
* Improved the efficiency of application of calibration parameters. Most of the parameters were only applicable to the
  legacy HDL-64E model and are handled on-device in all newer models.
* The calibration parameters for all models besides HDL-64E are now stored in an embedded database. This allows for
  easier packaging as a C++ library by avoiding the need to access YAML files.
* Changes specific to HDL-64E:
  - Added functionality to extract the calibration parameters embedded in a HDL-64E data stream. This is not done
    automatically and should be run once for each sensor to create a YAML file with the calibration parameters.
  - Improvements to HDL-64E calibration formulas:
    - Corrected the focal offset coefficient being off by 100x. https://github.com/ros-drivers/velodyne/issues/499
    - Fixed two-point correction not being applied for HDL-64E, despite being recommended by the product manual. Related
      to https://github.com/ros-drivers/velodyne/commit/16be7978ee0fa78818a9ff04faf5efac83ccf378
    - Fixed two-point correction being applied outside its applicable range.
    - Horizontal and vertical offset corrections are applied at the last step only.
    - The vertical-direction correction now uses the mean of the horizontal-direction corrections as its basis instead
      of just the x-direction correction. This provides a better agreement with the VeloView ground truth dataset.
  - Note that since HDL-64E does not include model ID info in its packets, the model ID must still be provided for it.
  - Similarly, the calibration parameters for HDL-64E are not standardized and sensor-specific calibration info always
    needs to be provided.
  - The added precise timing parameters for HDL-64E S1 are only approximate, due to them not being provided by the
    product manual.
* Changes specific to Alpha Prime (VLS-128):
  - Account for the precise timing parameters varying depending on the firmware version of the sensor. The correct
    values are detected automatically from the data.
  - Increased default max range from 130 to 250.
  - Fixed `distance_resolution` calibration param being hard-coded.

### Python-specific changes

* More robust PCAP parsing. Do not raise an exception for partial packets.
* Minimum Python version is now 3.8 instead of 3.7.

### C++-specific changes

* Added support for use as a C++ library via Conan, Vcpkg, or as a standalone system library.
* `gsl::span` is used for passing data to the decoder, which allows for zero-copy decoding.
* Renamed `PointXYZIRT` -> `VelodynePoint`.
* Added `RosScanBatcher` as for use with `velodyne_driver::VelodyneScan` messages. It is only available as a header and
  not built by default to avoid ROS-specific dependencies.

## [2.3.0] – 2022-03-20

### Changed

- `VLS-128` has been replaced by `Alpha Prime` as the model identifier, but is still accepted for backwards compatibility. (@flopie2009 #1)
- Removed the broken VLS-128 calibration data file. (@flopie2009 #2)

## [2.2.0] – 2021-12-10

### Added

- Added `time_range` parameter to `read_bag()` and `read_pcap()`.

### Fixed

- `read_bag()` no longer closes opened `Bag` objects passed to it.

## [2.1.0] – 2021-12-08

### Added

- `read_bag()` now optionally also includes `frame_id`-s from the ROS messages in the returned tuples
  when `return_frame_id=True` is set.

### Fixed

- Fixed `Config("model")` constructor not working as expected due to `calibration_file` missing a default value.
- Fixed out-of-scope pointer dereferencing in the calibration parser.
- Fixed a build error on Windows due to yaml-cpp by pinning its version to 0.7.

## [2.0.0] – 2021-11-11

### Added

- Support for PCAP file decoding with the `read_pcap()` function in Python and `StreamDecoder` in C++.
- More convenient data extraction from ROS bags with `read_bag()`.
- Standard calibration info provided by Velodyne is now used by default and `config.calibration_file` can be left unset
  in most cases.
- `TIMINGS_AVAILABLE` was added for details about which models support point timing info. The current list is:
  `['HDL-32E', 'VLP-16', 'VLP-32C', 'VLS-128']`.

### Changed

- The decoded scans are now returned as a contiguous float32 NumPy arrays instead of PCL-compatible structs by default.
  You can pass `as_pcl_structs=True` to the decoding functions for the old behavior.
- Model ID naming scheme has been updated to better match the typical forms used by Velodyne. <br>
  Before: `['VLP16', '32C', '32E', 'VLS128']`. <br>
  After: `['HDL-32E', 'HDL-64E', 'HDL-64E_S2', 'HDL-64E_S3', 'VLP-16', 'VLP-32C', 'VLS-128']`.
- Conversion to NumPy arrays is now done without any unnecessary data copying.

### Fixed

- Fixed a Python 2 ROS message decoding error.

## [1.0.1] – 2021-05-19

Initial release.

[3.0.0]: https://github.com/valgur/velodyne_decoder/compare/v2.3.0...v3.0.0

[2.3.0]: https://github.com/valgur/velodyne_decoder/compare/v2.2.0...v2.3.0

[2.2.0]: https://github.com/valgur/velodyne_decoder/compare/v2.1.0...v2.2.0

[2.1.0]: https://github.com/valgur/velodyne_decoder/compare/v2.0.0...v2.1.0

[2.0.0]: https://github.com/valgur/velodyne_decoder/compare/v1.0.1...v2.0.0

[1.0.1]: https://github.com/valgur/velodyne_decoder/releases/tag/v1.0.1
