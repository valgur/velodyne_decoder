from pathlib import Path

import pytest

import velodyne_decoder as vd

pcl_struct_dtype = {'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                    'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32}

calib_data_dir = Path(__file__).parent.parent / "velodyne_decoder/calibrations"


def test_config_constructor():
    vd.Config()
    vd.Config(model=vd.Model.VLP16)
    calibration = vd.Calibration.read(str(calib_data_dir / "VLP-16.yml"))
    vd.Config(model=vd.Model.VLP16, calibration=calibration)


def test_pcap_as_contiguous_array(sample_pcap_path):
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False))
    assert len(pcds) == 94
    stamp, pcd = pcds[0]
    assert stamp == 1427759049.259595
    assert pcd.shape == (27657, 6)
    assert pcd.dtype.name == "float32"


def test_pcap_as_struct_array(sample_pcap_path):
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=True))
    assert len(pcds) == 94
    stamp, pcd = pcds[0]
    assert stamp == 1427759049.259595
    assert pcd.shape == (27657,)
    assert pcd.dtype == pcl_struct_dtype


def test_pcap_time_range(sample_pcap_path):
    t0 = 1427759050
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False, time_range=(t0, None)))
    stamp, pcd = pcds[0]
    assert stamp >= t0


def test_bag_as_contiguous_array(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=False))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300, 6)
    assert pcd.dtype.name == "float32"


def test_bag_as_struct_array(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=True))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300,)
    assert pcd.dtype == pcl_struct_dtype


def test_bag_automatic_topic(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300, 6)


def test_bag_ros_time(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, use_header_time=False))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.7427142
    assert pcd.shape == (27300, 6)


def test_bag_frame_id(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, return_frame_id=True))
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300, 6)
    assert frame_id == "velodyne"


def test_bag_time_range(sample_bag_path):
    t0 = 1636622717
    pcds = list(vd.read_bag(sample_bag_path, time_range=(t0, None)))
    stamp, pcd, topic = pcds[0]
    assert stamp.to_sec() >= t0


@pytest.mark.parametrize("model_id", list(vd.Model.__members__.values()))
def test_bundled_calibrations(model_id):
    if model_id.name.startswith("HDL64E"):
        pytest.skip("Default calibrations for HDL-64E are not provided")
    vd.ScanDecoder(vd.Config(model=model_id))
