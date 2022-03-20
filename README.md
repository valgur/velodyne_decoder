# velodyne_decoder [![PyPI](https://img.shields.io/pypi/v/velodyne-decoder)](https://pypi.org/project/velodyne-decoder/) [![Build](https://github.com/valgur/velodyne_decoder/actions/workflows/build.yml/badge.svg?event=push)](https://github.com/valgur/velodyne_decoder/actions/workflows/build.yml) [![PyPI - Downloads](https://img.shields.io/pypi/dm/velodyne-decoder)](https://pypistats.org/packages/velodyne-decoder)

Python package and C++ library for Velodyne packet decoding. Point cloud extraction from PCAP and ROS bag files is
supported out of the box.

The decoded point clouds are provided either as a structured NumPy array:

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

or as a contiguous array of floats (default):

```python
array([[8.327308, -2.161341, 0.3599853, 85., 17., -0.04960084],
       [8.323784, -2.9578836, 0.27016047, 102., 15., -0.04959854],
       [8.184404, -2.845847, -0.8741639, 39., 2., -0.04959623],
       ...,
       [8.369528, -2.8161895, 2.307987, 17., 31., 0.00064051],
       [8.377898, -3.2570598, 1.7714221, 104., 30., 0.00064282],
       [8.358282, -2.8030438, 0.31229734, 104., 16., 0.00064282]], dtype=float32)
```

The layout of the structs matches the layout of `PointXYZIRT` point cloud points output by the ROS driver.

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
import velodyne_decoder as vd

config = vd.Config(model='VLP-32C')
bagfile = 'xyz.bag'
lidar_topics = ['/velodyne_packets']
cloud_arrays = []
for stamp, points, topic in vd.read_bag(bagfile, config, lidar_topics):
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

config = vd.Config(model='VLP-16', rpm=600)
pcap_file = 'vlp16.pcap'
cloud_arrays = []
for stamp, points in vd.read_pcap(pcap_file, config):
    cloud_arrays.append(points)
```

`config.model` and `config.rpm` must be set.

To return arrays of structs instead of the default contiguous arrays, set `as_pcl_structs=True`.

### Configuration

The main parameter `config.model` must always be set. For a list of supported model IDs see

```python
>> > velodyne_decoder.Config.SUPPORTED_MODELS
['HDL-32E', 'HDL-64E', 'HDL-64E_S2', 'HDL-64E_S3', 'VLP-16', 'VLP-32C', 'Alpha Prime']
```

Note that timing info is available for only a subset of the models:

```python
>> > velodyne_decoder.Config.TIMINGS_AVAILABLE
['HDL-32E', 'VLP-16', 'VLP-32C', 'Alpha Prime']
```

Other available options are:

* `calibration_file` – the beam calibration details from Velodyne are used by default based on the model ID. If you
  however wish to use a more specific calibration, you can specify one in
  the [YAML format](https://wiki.ros.org/velodyne_pointcloud#gen_calibration.py) used by the ROS driver.
* `min_range` and `max_range` – only return points between these range values.
* `min_angle` and `max_angle` – only return points between these azimuth angles.

Options only applicable to PCAP decoding:

* `rpm` – the device rotation speed in revolutions per minute.
* `gps_time` – use the timestamp from the packet's data if true, packet's arrival time otherwise (default).
* `timestamp_first_packet` – whether the timestamps are set based on the first or last packet in the scan

## Authors

* Martin Valgur ([@valgur](https://github.com/valgur))

The core functionality has been adapted from the ROS [velodyne driver](https://github.com/ros-drivers/velodyne).

## License

[BSD 3-Clause License](LICENSE)
