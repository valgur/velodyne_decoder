# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

import struct
from contextlib import contextmanager
import warnings

import dpkt
import numpy as np


def iter_pcap(pcap_file, time_range=(None, None)):
    start_time, end_time = time_range
    with _fopen(pcap_file, "rb") as f:
        for stamp, buf in dpkt.pcap.Reader(f):
            if start_time is not None and stamp < start_time:
                continue
            if end_time is not None and stamp > end_time:
                break
            # Get the innermost layer of the packet
            # typically Ethernet -> IP -> UDP -> raw Velodyne data
            try:
                data = dpkt.ethernet.Ethernet(buf)
            except dpkt.dpkt.UnpackError:
                continue
            while hasattr(data, "data"):
                data = data.data
            yield stamp, data


def iter_bag(bag_file, topics=None, default_msg_types=None, time_range=(None, None)):
    with warnings.catch_warnings():
        # Suppress an irrelevant deprecation warning from the Cryptodome package due to 'import imp'
        warnings.filterwarnings("ignore", category=DeprecationWarning)
        from rosbag import Bag
        from rospy import Time

    if isinstance(bag_file, Bag):
        bag = bag_file
    else:
        bag = Bag(str(bag_file))

    # Find relevant topics automatically, if not specified
    if topics is None and default_msg_types is not None:
        topics = []
        for topic, info in bag.get_type_and_topic_info()[1].items():
            if info.msg_type in default_msg_types:
                topics.append(topic)
    if not topics:
        return

    start_time, end_time = time_range
    if start_time is not None and not isinstance(start_time, Time):
        start_time = Time.from_sec(start_time)
    if end_time is not None and not isinstance(end_time, Time):
        end_time = Time.from_sec(end_time)

    try:
        for topic, msg, ros_time in bag.read_messages(topics, start_time, end_time):
            yield topic, msg, ros_time
    finally:
        if not isinstance(bag_file, Bag):
            bag.close()


@contextmanager
def _fopen(filein, *args, **kwargs):
    if isinstance(filein, str):  # filename
        with open(filein, *args, **kwargs) as f:
            yield f
    elif hasattr(filein, "open"):  # pathlib.Path
        with filein.open(*args, **kwargs) as f:
            yield f
    else:  # file-like object
        yield filein


def parse_packet(data):
    stamp, dual_return_mode, model_id = struct.unpack("<LBB", data[-6:])
    block_dtype = np.dtype(
        [
            ("bank_id", "<u2"),
            ("azimuth", "<u2"),
            ("measurements", [("distance", "u2"), ("intensity", "u1")], 32),
        ],
        align=False,
    )
    blocks = np.frombuffer(data[:-6], dtype=block_dtype)
    return blocks, stamp, model_id, dual_return_mode
