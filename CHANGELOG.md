# Changelog

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

[2.3.0]: https://github.com/valgur/velodyne_decoder/compare/v2.2.0...v2.3.0

[2.2.0]: https://github.com/valgur/velodyne_decoder/compare/v2.1.0...v2.2.0

[2.1.0]: https://github.com/valgur/velodyne_decoder/compare/v2.0.0...v2.1.0

[2.0.0]: https://github.com/valgur/velodyne_decoder/compare/v1.0.1...v2.0.0

[1.0.1]: https://github.com/valgur/velodyne_decoder/releases/tag/v1.0.1
