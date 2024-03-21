# velodyne_decoder
[![Build](https://github.com/valgur/velodyne_decoder/actions/workflows/build.yml/badge.svg?event=push)](https://github.com/valgur/velodyne_decoder/actions/workflows/build.yml) [![PyPI](https://img.shields.io/pypi/v/velodyne-decoder)](https://pypi.org/project/velodyne-decoder/) [![Conan Center](https://img.shields.io/conan/v/velodyne_decoder)](https://conan.io/center/recipes/velodyne_decoder) [![Vcpkg](https://img.shields.io/vcpkg/v/velodyne-decoder)](https://vcpkg.link/ports/velodyne-decoder) [![PyPI - Downloads](https://img.shields.io/pypi/dm/velodyne-decoder)](https://pypistats.org/packages/velodyne-decoder)

Python package and C++ library for Velodyne packet decoding. Point cloud extraction from PCAP and ROS bag files is
supported out of the box.

All non-solid-state Velodyne lidar models are fully supported. The model type and RPM are detected automatically from the data and no configuration is necessary to start using the library.

Notably, the library also includes support for dual-return data and decoding of telemetry packets. Precise timing info is available for all models.

The decoded point clouds have been validated to match the official [VeloView ground truth data](https://gitlab.kitware.com/LidarView/velodyneplugin/-/tree/master/Testing/Data?ref_type=heads) for all models.

In Python, the decoded point clouds are provided either as a structured NumPy array:

```python
array([(2.6912806, 1.1651788 , -0.47706223,  9., -0.10085896,   0, 16, 1),
       (2.3603256, 1.021404  , -1.428755  , 85., -0.10085782,   0,  1, 1),
       (2.6994078, 1.1675802 , -0.4092741 ,  3., -0.10085666,   0, 17, 1),
       ...,
       (2.8952641, 0.80728334,  0.48905915,  2.,  0.00054029, 923, 30, 1),
       (2.8683424, 0.79923725, -0.5555609 ,  2.,  0.00054144, 923, 15, 1),
       (2.908243 , 0.80980825,  0.56333727,  1.,  0.00054259, 923, 31, 1)],
      dtype={'names': ['x', 'y', 'z', 'intensity', 'time', 'column', 'ring', 'return_type'], 
             'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2', 'u1', 'u1'], 
             'offsets': [0, 4, 8, 12, 16, 20, 22, 23], 'itemsize': 32})
```

or as a contiguous array of floats (default):

```python
array([[2.691281, 1.165179, -0.477062,  9., -0.100859,   0., 16., 1.],
       [2.360326, 1.021404, -1.428755, 85., -0.100858,   0.,  1., 1.],
       [2.699408, 1.16758 , -0.409274,  3., -0.100857,   0., 17., 1.],
       ...,
       [2.895264, 0.807283,  0.489059,  2.,  0.00054 , 923., 30., 1.],
       [2.868342, 0.799237, -0.555561,  2.,  0.000541, 923., 15., 1.],
       [2.908243, 0.809808,  0.563337,  1.,  0.000543, 923., 31., 1.]], dtype=float32)
```

## Installation

Wheels are available from PyPI for Linux, MacOS and Windows. Python versions 3.7+ are supported.

```bash
pip install velodyne-decoder
```

Alternatively, you can build and install the development version from source.

```bash
sudo apt-get install cmake build-essential python3-dev
pip install git+https://github.com/valgur/velodyne_decoder.git
```

## Usage

### Decoding Velodyne data from a ROS bag

```python
import velodyne_decoder as vd

bagfile = 'xyz.bag'
lidar_topics = ['/velodyne_packets']
cloud_arrays = []
for stamp, points, topic in vd.read_bag(bagfile, topics=lidar_topics):
    cloud_arrays.append(points)
```

The `rosbag` library must be installed. If needed, you can install it without setting up the entire ROS stack with

```bash
pip install rosbag --extra-index-url https://rospypi.github.io/simple/
```

To extract all `VelodyneScan` messages in the bag you can leave the list of topics unspecified.

The header timestamp from the scan messages will be returned by default. To use the message arrival time instead
set `use_header_time=False`.

To return arrays of structs instead of the default contiguous arrays, set `as_pcl_structs=True`.

### Decoding Velodyne data from a PCAP file

```python
import velodyne_decoder as vd

pcap_file = 'vlp16.pcap'
cloud_arrays = []
for stamp, points in vd.read_pcap(pcap_file):
    cloud_arrays.append(points)
```

To return arrays of structs instead of the default contiguous arrays, set `as_pcl_structs=True`.

### Configuration

You can pass a `velodyne_decoder.Config` object to all decoder functions. The following options are available:

* `min_range` and `max_range` – only return points between these range values.
* `min_angle` and `max_angle` – only return points between these azimuth angles.
* `timestamp_first_packet` – whether the scan timestamps are set based on the first or last packet in the scan.
* `cut_angle` – when working with a raw packet stream, if unset (by default), the stream is split into a "scan" every time at least 360 degrees have been covered.
  If set, the splitting always occurs at the specified azimuth angle instead. Note that the scan might cover less than 360 degrees in this case.

Only required for data from HDL-64E sensors:
* `model` – the sensor model ID. See `velodyne_decoder.Model.__entries` for the possible values.
* `calibration_file` – beam calibration parameters in a YAML format.
  You can either extract the calibration info from a PCAP file with packets using `extract-hdl64e-calibration <pcap_file>` or
  convert a `db.xml` provided with the sensor using [gen_calibration.py](https://wiki.ros.org/velodyne_pointcloud#gen_calibration.py) from the ROS driver.

## Authors

* Martin Valgur ([@valgur](https://github.com/valgur))

The core functionality has been adapted from the ROS [velodyne driver](https://github.com/ros-drivers/velodyne).

## License

[BSD 3-Clause License](LICENSE)
