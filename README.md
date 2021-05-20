# velodyne_decoder [![PyPI](https://img.shields.io/pypi/v/velodyne-decoder)](https://pypi.org/project/velodyne-decoder/) [![Build](https://github.com/valgur/velodyne_decoder/actions/workflows/build.yml/badge.svg?event=push)](https://github.com/valgur/velodyne_decoder/actions/workflows/build.yml)

Python package and C++ library for Velodyne packet decoding. Intended as a light-weight substitute for
the [velodyne_driver](http://wiki.ros.org/velodyne_driver) in ROS with minimal external dependencies.

The resulting decoded data is provided as a structured NumPy array in Python and an array of structs in C++.

```python
array([(8.327308, -2.161341, 0.3599853, 85., 17, -0.04960084),
       (8.323784, -2.9578836, 0.27016047, 102., 15, -0.04959854),
       (8.184404, -2.845847, -0.8741639, 39., 2, -0.04959623), ...,
       (8.369528, -2.8161895, 2.307987, 17., 31, 0.00064051),
       (8.377898, -3.2570598, 1.7714221, 104., 30, 0.00064282),
       (8.358282, -2.8030438, 0.31229734, 104., 16, 0.00064282)],
      dtype={'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
             'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'], 'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32})
```

The layout of the structs and the array is identical to the `PointXYZIRT` point clouds output by the ROS driver.

## Installation

Wheels are available from PyPI for Linux, MacOS and Windows. Python versions 2.7 and 3.6+ are supported.

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
from rosbag import Bag
import velodyne_decoder as vd

config = vd.Config()
config.model = '32C'
config.calibration_file = '/opt/ros/noetic/share/velodyne_pointcloud/params/VeloView-VLP-32C.yaml'
config.min_range = 0.3
config.max_range = 130
decoder = vd.ScanDecoder(config)

bagfile = 'xyz.bag'
lidar_topics = ['/velodyne_packets']
cloud_arrays = []
with Bag(bagfile) as bag:
    for topic, scan_msg, ros_time in bag.read_messages(lidar_topics):
        cloud_arrays.append(decoder.decode_message(scan_msg))
```

`config.model` and `config.calibration_file` are required. For a list of supported model IDs see

```python
>>> velodyne_decoder.Config.SUPPORTED_MODELS
['VLP16', '32C', '32E', 'VLS128']
```

For the calibration file you can use the
one [provided with the velodyne driver](https://github.com/ros-drivers/velodyne/tree/master/velodyne_pointcloud/params)
matching your model.

Additional optional config parameters are `min_range` and `max_range` in meters and `min_angle` and `max_angle` in
degrees.

## Authors

* Martin Valgur ([@valgur](https://github.com/valgur)) – this library.

The core functionality has been adapted from the ROS [velodyne driver](https://github.com/ros-drivers/velodyne).

## License

[BSD 3-Clause License](LICENSE)
