import pytest

import velodyne_decoder as vd

pcl_struct_dtype = {'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                    'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32}


@pytest.fixture
def config():
    config = vd.Config()
    config.model = "VLP-16"
    config.rpm = 600
    return config


def test_config_constructor():
    vd.Config()
    vd.Config("VLP-16", rpm=600)
    vd.Config(model="VLP-16", rpm=600)
    vd.Config("VLP-16", calibration_file="calib.yml")
    with pytest.raises(TypeError):
        vd.Config("VLP-16", "calib.yml")


def test_pcap_as_contiguous_array(sample_pcap_path, config):
    pcds = list(vd.read_pcap(sample_pcap_path, config, as_pcl_structs=False))
    assert len(pcds) == 93
    stamp, pcd = pcds[0]
    assert stamp == 1427759049.258258
    assert pcd.shape == (27282, 6)
    assert pcd.dtype.name == "float32"


def test_pcap_as_struct_array(sample_pcap_path, config):
    pcds = list(vd.read_pcap(sample_pcap_path, config, as_pcl_structs=True))
    assert len(pcds) == 93
    stamp, pcd = pcds[0]
    assert stamp == 1427759049.258258
    assert pcd.shape == (27282,)
    assert pcd.dtype == pcl_struct_dtype


def test_bag_as_contiguous_array(sample_bag_path, config):
    pcds = list(vd.read_bag(sample_bag_path, config, "/velodyne_packets", as_pcl_structs=False))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300, 6)
    assert pcd.dtype.name == "float32"


def test_bag_as_struct_array(sample_bag_path, config):
    pcds = list(vd.read_bag(sample_bag_path, config, "/velodyne_packets", as_pcl_structs=True))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300,)
    assert pcd.dtype == pcl_struct_dtype


def test_bag_automatic_topic(sample_bag_path, config):
    pcds = list(vd.read_bag(sample_bag_path, config))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300, 6)


def test_bag_ros_time(sample_bag_path, config):
    pcds = list(vd.read_bag(sample_bag_path, config, use_header_time=False))
    assert len(pcds) == 13
    stamp, pcd, topic = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.7427142
    assert pcd.shape == (27300, 6)


def test_bag_frame_id(sample_bag_path, config):
    pcds = list(vd.read_bag(sample_bag_path, config, return_frame_id=True))
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.to_sec() == 1636622716.742135
    assert pcd.shape == (27300, 6)
    assert frame_id == "velodyne"
