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


def test_pcap_time_range(sample_pcap_path, config):
    t0 = 1427759050
    pcds = list(vd.read_pcap(sample_pcap_path, config, as_pcl_structs=False, time_range=(t0, None)))
    stamp, pcd = pcds[0]
    assert stamp >= t0


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


def test_bag_time_range(sample_bag_path, config):
    t0 = 1636622717
    pcds = list(vd.read_bag(sample_bag_path, config, time_range=(t0, None)))
    stamp, pcd, topic = pcds[0]
    assert stamp.to_sec() >= t0


@pytest.mark.parametrize("model_id", vd.Config.SUPPORTED_MODELS)
def test_bundled_calibrations(model_id):
    if model_id in ["HDL-64E_S2", "HDL-64E_S3"]:
        # No default calibration has been provided for these by VeloView
        return
    vd.ScanDecoder(vd.Config(model_id))


def test_model_renaming():
    config = vd.Config("VLS-128")
    assert config.model == "Alpha Prime"
    config = vd.Config()
    config.model = "VLS-128"
    assert config.model == "Alpha Prime"
