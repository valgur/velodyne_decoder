# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

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


def test_bag_as_struct_array(sample_bag_path, pcl_struct_dtype):
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
    assert pcd.shape == (27300, 8)
    assert frame_id == "velodyne"


def test_bag_time_range(sample_bag_path):
    t0 = 1636622717
    pcds = list(vd.read_bag(sample_bag_path, time_range=(t0, None)))
    assert pcds[0].stamp.host >= t0
