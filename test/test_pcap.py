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
    # Sanity check that the point clouds are actually distinct
    assert not all(len(x) == len(pcd) for _, x in pcds)


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
    assert not all(len(x) == len(pcd) for _, x in pcds)


def test_pcap_time_range(sample_pcap_path):
    t0 = 1427759050
    pcds = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False, time_range=(t0, None)))
    stamp, pcd = pcds[0]
    assert stamp.host >= t0
    assert stamp.device >= t0


def _struct_pcd_to_contiguous(pcd_struct):
    shape = (len(pcd_struct), len(pcd_struct.dtype.fields))
    converted_pcd = np.empty(shape, dtype=np.float32)
    for i, field in enumerate(pcd_struct.dtype.fields):
        converted_pcd[:, i] = pcd_struct[field]
    return converted_pcd


def test_pcap_structs_match_contiguous(sample_pcap_path):
    """Check that the contents of as_pcl_structs=True/False match."""
    pcds_struct = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=True))
    pcds_contiguous = list(vd.read_pcap(sample_pcap_path, as_pcl_structs=False))
    assert len(pcds_struct) == len(pcds_contiguous)
    for (stamp_struct, pcd_struct), (stamp_contiguous, pcd_contiguous) in zip(pcds_struct, pcds_contiguous):
        assert stamp_struct.host == stamp_contiguous.host
        assert stamp_struct.device == stamp_contiguous.device
        cast_struct_pcd = _struct_pcd_to_contiguous(pcd_struct)
        assert np.allclose(cast_struct_pcd, pcd_contiguous, rtol=1e-6)
