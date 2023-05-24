# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

import sys
from collections import namedtuple

import velodyne_decoder as vd

is_py2 = sys.version_info[0] == 2


def read_pcap(
    pcap_file,
    config=None,
    as_pcl_structs=False,
    time_range=(None, None),
):
    """Decodes and yields all point clouds stored in a PCAP file.

    `model` must be set in the provided config.

    Parameters
    ----------
    pcap_file : path or file handle
    config : Config, optional
    as_pcl_structs : bool
        If False, the returned NumPy arrays will be a contiguous array of floats (default).
        If True, the returned NumPy arrays will contain PCL-compatible structs with dtype
        {'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
         'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
         'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32}
    time_range : float, float
        Optionally only return scans from given time range.

    Yields
    ------
    timestamp: float
    point_cloud : numpy.ndarray
    """
    if config is None:
        config = vd.Config()

    # Extract model and calibration for HDL-64E S2/S3 from the pcap file if not specified
    if config.model is None or config.calibration is None:
        try:
            model, calib = vd.hdl64e.detect_hdl64e(pcap_file)
        except ValueError as e:
            raise ValueError(
                "Data is from a HDL-64E, but the exact model and calibration could not be determined: "
                + str(e)
            )
        config.model = config.model or model
        config.calibration = config.calibration or calib
        if hasattr(pcap_file, "seek"):
            pcap_file.seek(0)

    decoder = vd.StreamDecoder(config)
    ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
    for host_stamp, data in vd.util.iter_pcap(pcap_file, time_range):
        if len(data) != vd.PACKET_SIZE:
            continue
        if is_py2:
            data = bytearray(data)
        result = decoder.decode(host_stamp, data, as_pcl_structs)
        if result is not None:
            yield ResultTuple(*result)
    # Decode any remaining packets
    result = decoder.finish(as_pcl_structs)
    if result is not None:
        yield ResultTuple(*result)
