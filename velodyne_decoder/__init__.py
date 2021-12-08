from collections import namedtuple
from contextlib import contextmanager

import dpkt
import sys
from velodyne_decoder_pylib import *

is_py2 = sys.version_info[0] == 2


def read_pcap(pcap_file, config, as_pcl_structs=False):
    """Decodes and yields all point clouds stored in a PCAP file.

    `model` and `rpm` parameters must be set in the provided config.

    Parameters
    ----------
    pcap_file : path or file handle
    config : Config
    as_pcl_structs : bool
        If False, the returned NumPy arrays will be a contiguous array of floats (default).
        If True, the returned NumPy arrays will contain PCL-compatible structs with dtype
        {'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
         'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
         'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32}

    Yields
    ------
    timestamp: float
    point_cloud : numpy.ndarray
    """
    decoder = StreamDecoder(config)
    ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
    with _fopen(pcap_file, "rb") as f:
        for stamp, buf in dpkt.pcap.Reader(f):
            data = dpkt.ethernet.Ethernet(buf).data.data.data
            if is_py2:
                data = bytearray(data)
            if len(data) != PACKET_SIZE:
                continue
            result = decoder.decode(stamp, data, as_pcl_structs)
            if result is not None:
                yield ResultTuple(*result)


def _get_velodyne_scan_topics(bag):
    topics = []
    for topic, info in bag.get_type_and_topic_info()[1].items():
        if info.msg_type == "velodyne_msgs/VelodyneScan":
            topics.append(topic)
    return topics


def read_bag(bag_file, config, topics=None, as_pcl_structs=False, use_header_time=True,
             return_frame_id=False):
    """Decodes and yields all point clouds stored in a ROS bag file.

    `model` parameter must be set in the provided config.

    Parameters
    ----------
    bag_file : path or file handle
    config : Config
    topics : str or list of str
    as_pcl_structs : bool
        If False, the returned NumPy arrays will be a contiguous array of floats (default).
        If True, the returned NumPy arrays will contain PCL-compatible structs with dtype
        {'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
         'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
         'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32}
    use_header_time : bool
        If True, scan_msg.header.stamp will be used for the time stamps.
        If False, the message arrival time will be used.
    return_frame_id : bool
        If True, includes the frame_id of the messages in the returned tuple. Defaults to False.

    Yields
    ------
    timestamp: ros.Time
    point_cloud : numpy.ndarray
    topic : str
    """
    from rosbag import Bag

    decoder = ScanDecoder(config)

    if isinstance(bag_file, Bag):
        bag = bag_file
    else:
        bag = Bag(str(bag_file))

    if topics is None:
        topics = _get_velodyne_scan_topics(bag)

    Result = namedtuple("ResultTuple", ("stamp", "points", "topic"))
    ResultWithFrameId = namedtuple("ResultTuple", ("stamp", "points", "topic", "frame_id"))
    try:
        for topic, scan_msg, ros_time in bag.read_messages(topics):
            if is_py2:
                for packet in scan_msg.packets:
                    packet.data = bytearray(packet.data)
            stamp = scan_msg.header.stamp if use_header_time else ros_time
            points = decoder.decode_message(scan_msg, as_pcl_structs)
            if return_frame_id:
                yield ResultWithFrameId(stamp, points, topic, scan_msg.header.frame_id)
            else:
                yield Result(stamp, points, topic)
    finally:
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
