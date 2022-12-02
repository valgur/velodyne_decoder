from collections import namedtuple
from contextlib import contextmanager

import dpkt
import sys
from velodyne_decoder_pylib import *

is_py2 = sys.version_info[0] == 2


def read_pcap(pcap_file, config, as_pcl_structs=False, time_range=(None, None)):
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
    time_range : float, float
        Optionally only return scans from given time range.

    Yields
    ------
    timestamp: float
    point_cloud : numpy.ndarray
    """
    decoder = StreamDecoder(config)
    start_time, end_time = time_range
    ResultTuple = namedtuple("StampCloudTuple", ("stamp", "points"))
    with _fopen(pcap_file, "rb") as f:
        for stamp, buf in dpkt.pcap.Reader(f):
            if (start_time is not None and stamp < start_time or
                end_time is not None and stamp > end_time):
                continue
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
             return_frame_id=False, time_range=(None, None)):
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
    time_range : float, float or rospy.Time, rospy.Time
        Optionally only return scans from given time range.

    Yields
    ------
    timestamp: rospy.Time
    point_cloud : numpy.ndarray
    topic : str
    """
    from rosbag import Bag
    from rospy import Time

    decoder = ScanDecoder(config)

    if isinstance(bag_file, Bag):
        bag = bag_file
    else:
        bag = Bag(str(bag_file))

    if topics is None:
        topics = _get_velodyne_scan_topics(bag)

    start_time, end_time = time_range
    if start_time is not None and not isinstance(start_time, Time):
        start_time = Time.from_sec(start_time)
    if end_time is not None and not isinstance(end_time, Time):
        end_time = Time.from_sec(end_time)

    Result = namedtuple("ResultTuple", ("stamp", "points", "topic"))
    ResultWithFrameId = namedtuple("ResultTuple", ("stamp", "points", "topic", "frame_id"))
    try:
        for topic, scan_msg, ros_time in bag.read_messages(topics, start_time=start_time,
                                                           end_time=end_time):
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
        if not isinstance(bag_file, Bag):
            bag.close()


def read_bag_ros1(
    bag_file,
    config,
    topics,
    as_pcl_structs=False,
    use_header_time=True,
    return_frame_id=False,
    time_range=(None, None),
):
    """Decodes and yields all point clouds stored in a ROS bag file based on the rosbags package.
    This will be possible even if the system ros version is ros2.

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
    time_range : int, int
        Optionally only return scans from given time range.
    Yields
    ------
    timestamp: int
    point_cloud : numpy.ndarray
    topic : str
    """
    import types
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import register_types, get_types_from_msg
    from rosbags.serde import deserialize_ros1
    decoder = ScanDecoder(config)
    if isinstance(bag_file, Reader):
        bag = bag_file
    else:
        bag = Reader(str(bag_file))
        bag.open()
    try:
        connections = [
            c for c in bag.connections if c.msgtype == "velodyne_msgs/msg/VelodyneScan"
        ]
        if topics is not None:
            if isinstance(topics, str):
                connections = [c for c in connections if c.topic == topics]
            elif isinstance(topics, list):
                connections = [c for c in connections if c.topic in topics]
            else:
                raise AttributeError(
                    "The type of topics ({}) is not supported!".format(type(topics))
                )
        # register custom type
        if len(connections) > 0:
            register_types(
                get_types_from_msg(connections[0].msgdef, connections[0].msgtype)
            )
        start_time, end_time = time_range
        if start_time is not None and not isinstance(start_time, int):
            raise AttributeError("Can't handle non int start_time (has to be in ns)!")
        if end_time is not None and not isinstance(end_time, int):
            raise AttributeError("Can't handle non int end_time (has to be in ns)!")
        Result = namedtuple("ResultTuple", ("stamp", "points", "topic"))
        ResultWithFrameId = namedtuple(
            "ResultTuple", ("stamp", "points", "topic", "frame_id")
        )
        for connection, ros_timestamp, rawdata in bag.messages(
            connections=connections, start=start_time, stop=end_time
        ):
            scan_msg = deserialize_ros1(rawdata, connection.msgtype)
            # create workaround to add required methods
            def to_sec(self):
                return float(self.sec) + (float(self.nanosec) / 1_000_000_000)
            def to_nsec(self):
                return int(self.sec) * 1_000_000_000 + self.nanosec
            scan_msg.header.stamp.to_sec = types.MethodType(
                to_sec, scan_msg.header.stamp
            )
            scan_msg.header.stamp.to_nsec = types.MethodType(
                to_nsec, scan_msg.header.stamp
            )
            for i in range(len(scan_msg.packets)):
                scan_msg.packets[i].stamp.to_sec = types.MethodType(
                    to_sec, scan_msg.packets[i].stamp
                )
                scan_msg.packets[i].stamp.to_nsec = types.MethodType(
                    to_nsec, scan_msg.packets[i].stamp
                )
            # parse data
            if is_py2:
                for packet in scan_msg.packets:
                    packet.data = bytearray(packet.data)
            stamp = scan_msg.header.stamp if use_header_time else ros_timestamp
            points = decoder.decode_message(scan_msg, as_pcl_structs)
            if return_frame_id:
                yield ResultWithFrameId(
                    stamp, points, connection.topic, scan_msg.header.frame_id
                )
            else:
                yield Result(stamp, points, connection.topic)
    finally:
        if not isinstance(bag_file, Reader):
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
