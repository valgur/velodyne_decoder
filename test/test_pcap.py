# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

import numpy as np
import velodyne_decoder as vd


def test_pcap_as_contiguous_array(sample_pcap_path):
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False))
    assert len(pcds) == 94
    stamp, pcd = pcds[0]
    assert stamp.host == 1427759049.259595
    assert pcd.shape == (27657, 8)
    assert pcd.dtype.name == "float32"
    # Check that the column index is monotonically increasing
    column_idx = pcd[:, vd.PointField.column].astype(int)
    assert np.diff(column_idx).min() == 0


def test_pcap_as_struct_array(sample_pcap_path, pcl_struct_dtype):
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=True))
    assert len(pcds) == 94
    stamp, pcd = pcds[0]
    assert stamp.host == 1427759049.259595
    assert pcd.shape == (27657,)
    assert pcd.dtype == pcl_struct_dtype
    # Check that the column index is monotonically increasing
    column_idx = pcd["column"].astype(int)
    assert np.diff(column_idx).min() == 0


def test_pcap_time_range(sample_pcap_path):
    t0 = 1427759050
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False, time_range=(t0, None)))
    stamp, pcd = pcds[0]
    assert stamp.host >= t0
    assert stamp.device >= t0
