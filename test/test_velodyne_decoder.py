# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

import math
from pathlib import Path

import pytest
import velodyne_decoder as vd
import yaml

pcl_struct_dtype = {
    "names": ["x", "y", "z", "intensity", "ring", "time", "return_type"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "<f4", "u1"],
    "offsets": [0, 4, 8, 12, 16, 20, 24],
    "itemsize": 32,
}
calib_data_dir = Path(__file__).parent.parent / "src/velodyne_decoder/calibrations"

MODELS = list(vd.Model.__members__)


def test_config_constructor():
    vd.Config()
    vd.Config(model=vd.Model.VLP16)
    calibration = vd.Calibration.read(str(calib_data_dir / "VLP-16.yml"))
    vd.Config(model=vd.Model.VLP16, calibration=calibration)


def test_pcap_as_contiguous_array(sample_pcap_path):
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False))
    assert len(pcds) == 94
    stamp, pcd = pcds[0]
    assert stamp.host == 1427759049.259595
    assert pcd.shape == (27657, 7)
    assert pcd.dtype.name == "float32"


def test_pcap_as_struct_array(sample_pcap_path):
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=True))
    assert len(pcds) == 94
    stamp, pcd = pcds[0]
    assert stamp.host == 1427759049.259595
    assert pcd.shape == (27657,)
    assert pcd.dtype == pcl_struct_dtype


def test_pcap_time_range(sample_pcap_path):
    t0 = 1427759050
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False, time_range=(t0, None)))
    stamp, pcd = pcds[0]
    assert stamp.host >= t0
    assert stamp.device >= t0


def test_bag_as_contiguous_array(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=False))
    assert len(pcds) == 13
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.host == 1636622716.742135
    assert pcd.shape == (27300, 7)
    assert pcd.dtype.name == "float32"
    assert frame_id == "velodyne"


def test_bag_as_struct_array(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=True))
    assert len(pcds) == 13
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.host == 1636622716.742135
    assert pcd.shape == (27300,)
    assert pcd.dtype == pcl_struct_dtype
    assert frame_id == "velodyne"


def test_bag_automatic_topic(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path))
    assert len(pcds) == 13
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.host == 1636622716.742135
    assert pcd.shape == (27300, 7)
    assert frame_id == "velodyne"


def test_bag_time_range(sample_bag_path):
    t0 = 1636622717
    pcds = list(vd.read_bag(sample_bag_path, time_range=(t0, None)))
    assert pcds[0].stamp.host >= t0


@pytest.mark.parametrize("model_id", MODELS)
def test_bundled_calibrations(model_id):
    model_id = vd.Model.__members__[model_id]
    calib = None
    if model_id.name.startswith("HDL64E"):
        calib_file = {
            vd.Model.HDL64E_S1: "HDL-64E_S1-utexas.yml",
            vd.Model.HDL64E_S2: "HDL-64E_S2.1-sztaki.yml",
            vd.Model.HDL64E_S3: "HDL-64E_S3-VeloView.yml",
        }[model_id]
        calib = vd.get_bundled_calibration(calib_file)
    vd.ScanDecoder(vd.Config(model=model_id, calibration=calib))


@pytest.mark.parametrize("model_id", [m for m in MODELS if not m.startswith("HDL64E")])
def test_default_calib_vertical_offsets(model_id):
    """Check that the pre-defined vertical offset calibration values are sensible."""
    model_id = vd.Model.__members__[model_id]
    calib = vd.Calibration.default_calibs[model_id]
    lasers = yaml.safe_load(calib.to_string())["lasers"]
    focal_distance = {
        vd.Model.AlphaPrime: 58.63e-3,
        vd.Model.HDL32E: 28.83e-3,
        vd.Model.PuckHiRes: 41.91e-3,
        vd.Model.VLP16: 41.91e-3,
        vd.Model.VLP32A: 42.4e-3,
        vd.Model.VLP32B: 42.4e-3,
        vd.Model.VLP32C: 42.4e-3,
    }[model_id]
    for laser in lasers:
        approx_offset = focal_distance * math.tan(-laser["vert_correction"])
        thr = 0.5e-3 if model_id == vd.Model.PuckHiRes else 0.1e-3
        assert pytest.approx(approx_offset, abs=thr) == laser["vert_offset_correction"]
