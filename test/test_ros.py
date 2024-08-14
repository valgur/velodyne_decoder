# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause
import numpy as np
import velodyne_decoder as vd


def test_bag_as_contiguous_array(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=False))
    assert len(pcds) == 13
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.host == 1636622716.742135
    assert pcd.shape == (27300, 8)
    assert pcd.dtype.name == "float32"
    assert frame_id == "velodyne"
    # Sanity check that the point clouds are actually distinct
    assert not all(len(x) == len(pcd) for _, x, _, _ in pcds)


def test_bag_as_struct_array(sample_bag_path, pcl_struct_dtype):
    pcds = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=True))
    assert len(pcds) == 13
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.host == 1636622716.742135
    assert pcd.shape == (27300,)
    assert pcd.dtype == pcl_struct_dtype
    assert frame_id == "velodyne"
    assert not all(len(x) == len(pcd) for _, x, _, _ in pcds)


def test_bag_automatic_topic(sample_bag_path):
    pcds = list(vd.read_bag(sample_bag_path))
    assert len(pcds) == 13
    stamp, pcd, topic, frame_id = pcds[0]
    assert topic == "/velodyne_packets"
    assert stamp.host == 1636622716.742135
    assert pcd.shape == (27300, 8)
    assert frame_id == "velodyne"


def test_bag_time_range(sample_bag_path):
    t0 = 1636622717
    pcds = list(vd.read_bag(sample_bag_path, time_range=(t0, None)))
    assert pcds[0].stamp.host >= t0


def _struct_pcd_to_contiguous(pcd_struct):
    shape = (len(pcd_struct), len(pcd_struct.dtype.fields))
    converted_pcd = np.empty(shape, dtype=np.float32)
    for i, field in enumerate(pcd_struct.dtype.fields):
        converted_pcd[:, i] = pcd_struct[field]
    return converted_pcd


def test_pcap_structs_match_contiguous(sample_bag_path):
    """Check that the contents of as_pcl_structs=True/False match."""
    pcds_struct = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=True))
    pcds_contiguous = list(vd.read_bag(sample_bag_path, topics="/velodyne_packets", as_pcl_structs=False))
    assert len(pcds_struct) == len(pcds_contiguous)
    for (stamp_struct, pcd_struct, _, _), (stamp_contiguous, pcd_contiguous, _, _) in zip(pcds_struct, pcds_contiguous):
        assert stamp_struct.host == stamp_contiguous.host
        assert stamp_struct.device == stamp_contiguous.device
        cast_struct_pcd = _struct_pcd_to_contiguous(pcd_struct)
        assert np.allclose(cast_struct_pcd, pcd_contiguous, rtol=1e-6)
