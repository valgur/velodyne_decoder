"""
Tools to read calibration and status info from HDL-64E S2 and S3 packet streams.

Usage example:
    pcap_file = "hdl64e_packets.pcap"
    calib = vd.hdl64e.read_calibration_from_pcap(pcap_file)
    config = vd.Config(model=vd.Model.HDL64E_S3, calibration=calib)

Saving the calibration to a file (also includes all other parsed metadata under the "status" key):
    packets = vd.hdl64e.iter_pcap_packets(pcap_file)
    metadata = vd.hdl64e.read_status_info(packets)
    calib_dict = vd.hdl64e.status_info_to_calib_dict(metadata)
    with open("calib.yml", "w") as f:
        f.write(yaml.dump(calib_dict, sort_keys=False))

Using the saved calibration:
    calib = vd.Calibration.read("calib.yml")
    config = vd.Config(model=vd.Model.HDL64E_S3, calibration=calib)

YAML output example:
  distance_resolution: 0.002
  num_lasers: 64
  lasers:
  - laser_id: 0
    ...
  - laser_id: 63
    vert_correction: -0.1534144
    rot_correction: 0.0483456
    dist_correction: 1.228
    dist_correction_x: 1.258
    dist_correction_y: 1.258
    vert_offset_correction: 0.144
    horiz_offset_correction: -0.026
    focal_distance: 0.25
    focal_slope: 0.011
    min_intensity: 40
    max_intensity: 255
  status:
    Model: S3
    Current Time: 2000-01-01 00:05:54
    GPS Status: 0
    Temperature: 20
    Firmware Version: 227
    Upper Block Noise Threshold: 24
    Lower Block Noise Threshold: 24
    Calibration Time: 2016-11-15 12:47:47
    Humidity %: 25
    RPM: 300
    FOV Min: 0
    FOV Max: 36000
    Real Life Hour: 2685
    IP Source: 192.168.3.43
    IP Destination: 192.168.3.255
    Multiple Return Status: Strongest
    Power Level Status: A0
    Lens Contaminated: true
    Unit Too Hot Internally: false
    Unit Too Cold Internally: false
    PPS Signal Present: false
    GPS Time Present: false
"""
from __future__ import print_function

import argparse
import datetime
import math
import struct
import sys
import warnings

import numpy as np
import yaml
from velodyne_decoder_pylib import Calibration, PACKET_SIZE

import velodyne_decoder.util as _util

__all__ = [
    "iter_pcap_packets",
    "iter_bag_packets",
    "read_calibration_from_pcap",
    "status_info_to_calib_dict",
    "read_status_info",
]


def crc16(data):
    xor_in = 0x0000  # initial value
    xor_out = 0x0000  # final XOR value
    poly = 0x8005  # generator polynomial (normal form)
    reg = xor_in
    for octet in data:
        for i in range(8):
            topbit = reg & 0x8000
            if octet & (0x80 >> i):
                topbit ^= 0x8000
            reg <<= 1
            if topbit:
                reg ^= poly
        reg &= 0xFFFF
    return reg ^ xor_out


def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i : i + n]


def iter_pcap_packets(pcap_path):
    """Yields raw Velodyne packets from a pcap file."""
    for stamp, data in _util.iter_pcap(pcap_path):
        if len(data) == PACKET_SIZE:
            yield data


def iter_bag_packets(bag_path, topic=None):
    """Yields raw Velodyne packets from a given topic in a bag file."""
    for topic, scan_msg, ros_time in _util.iter_bag(bag_path, [topic]):
        for packet in scan_msg.packets:
            yield packet.data


def read_status_info(raw_packets):
    cur_status = {}
    CALIB_DATA_SIZE = 1820
    values = bytearray(CALIB_DATA_SIZE)
    attempts = 0
    byte_count = 0
    for packet_data in raw_packets:
        stamp, status_type, status_value = struct.unpack("<LBB", packet_data[-6:])
        if chr(status_type) in "HMSDNYGTVW":
            cur_status[chr(status_type)] = status_value
            if chr(status_type) != "W":
                continue
        values[byte_count] = status_value
        byte_count += 1
        if byte_count == CALIB_DATA_SIZE:
            attempts += 1
            start = values.find(b"UNIT#")
            if start < 0:
                warnings.warn("Did not find a 'UNIT#' start token in the status data")
                byte_count = 0
                continue
            values = values[start:] + values[:start]
            expected_checksum = struct.unpack("<H", values[-2:])[0]
            # S3 uses a checksum, S2 sets only the expected length of data as the last bytes
            is_s3 = "W" in cur_status
            if is_s3:
                calib_data = b"".join(x[1:21] for x in chunks(values[7 : -3 * 7], 28))
                checksum = crc16(calib_data)
            else:
                checksum = CALIB_DATA_SIZE
            if expected_checksum == checksum:
                return decode_status_bytes(values, cur_status)
            else:
                warnings.warn(
                    "Checksum validation of calibration data failed: "
                    "calculated {:X} != expected {:X}.".format(checksum, expected_checksum)
                )
                byte_count = CALIB_DATA_SIZE - start if start > 0 else 0
                continue
    if attempts == 0:
        raise ValueError(
            "Not enough metadata bytes in stream: {} < {}".format(byte_count, CALIB_DATA_SIZE)
        )
    else:
        raise ValueError("No valid metadata found in stream")


def decode_status_bytes(status_bytes, cur_status):
    is_s3 = "W" in cur_status
    tail = status_bytes[-4 * 7 :]
    data = {}
    data["Model"] = "S3" if is_s3 else "S2"
    data["Current Time"] = datetime.datetime(
        2000 + cur_status["Y"],
        cur_status["N"],
        cur_status["D"],
        cur_status["H"],
        cur_status["M"],
        cur_status["S"],
    )
    data["GPS Status"] = cur_status["G"]
    data["Temperature"] = cur_status["T"]
    data["Firmware Version"] = cur_status["V"]
    data["Upper Block Noise Threshold"] = status_bytes[5]
    data["Lower Block Noise Threshold"] = status_bytes[6]
    if all(x != 0xFF for x in tail[:6]):
        data["Calibration Time"] = datetime.datetime(tail[0] + 2000, *tail[1:6])
    else:
        data["Calibration Time"] = None
    data["Humidity %"] = tail[6]
    data["RPM"] = struct.unpack_from("<h", tail, 7)[0]
    data["FOV Min"] = struct.unpack_from("<H", tail, 9)[0]
    data["FOV Max"] = struct.unpack_from("<H", tail, 11)[0]
    data["Real Life Hour"] = struct.unpack_from("<H", tail, 13)[0]
    data["IP Source"] = ".".join(map(str, struct.unpack_from("<BBBB", tail, 15)))
    data["IP Destination"] = ".".join(map(str, struct.unpack_from("<BBBB", tail, 19)))
    data["Multiple Return Status"] = {0: "Strongest", 1: "Last", 2: "Both"}.get(tail[23], tail[23])
    data["Power Level Status"] = hex(tail[25])[2:].upper()
    w = cur_status["W"] if is_s3 else tail[24]
    data["Lens Contaminated"] = bool(w & (1 << 0))
    data["Unit Too Hot Internally"] = bool(w & (1 << 1))
    data["Unit Too Cold Internally"] = bool(w & (1 << 2))
    if not is_s3:
        data["Voltage Too Low"] = bool(w & (1 << 3))
        data["Voltage Too High"] = bool(w & (1 << 4))
    data["PPS Signal Present"] = bool(w & (1 << 5))
    data["GPS Time Present"] = bool(w & (1 << 6))

    data["Calibration"] = []
    raw_lasers_data = [x[:21] for x in chunks(status_bytes[7 : -3 * 7], 28)]
    for raw_laser_data in raw_lasers_data:
        raw_values = struct.unpack("<BhhhhhhhhhBB", raw_laser_data)
        c = {}
        c["Laser ID"] = raw_values[0]
        c["Vertical Correction"] = raw_values[1] * 0.01
        c["Rotational Correction"] = raw_values[2] * 0.01
        c["Far Distance Correction"] = raw_values[3] * 0.001
        c["Distance Correction X"] = raw_values[4] * 0.001
        c["Distance Correction Y"] = raw_values[5] * 0.001
        c["Vertical Offset Correction"] = raw_values[6] * 0.001
        c["Horizontal Offset Correction"] = raw_values[7] * 0.001
        c["Focal Distance"] = raw_values[8] * 0.001
        c["Focal Slope"] = raw_values[9] * 0.001
        c["Min Intensity"] = raw_values[10]
        c["Max Intensity"] = raw_values[11]
        data["Calibration"].append(c)
    return data


def status_info_to_calib_dict(metadata):
    calib = {}
    calib["distance_resolution"] = 0.002
    calib["num_lasers"] = 64
    calib["lasers"] = []
    for c in metadata["Calibration"]:
        l = {}
        l["laser_id"] = c["Laser ID"]
        l["vert_correction"] = float(np.round(math.radians(c["Vertical Correction"]), 7))
        l["rot_correction"] = float(np.round(math.radians(c["Rotational Correction"]), 7))
        l["dist_correction"] = float(np.round(c["Far Distance Correction"], 7))
        l["dist_correction_x"] = float(np.round(c["Distance Correction X"], 7))
        l["dist_correction_y"] = float(np.round(c["Distance Correction Y"], 7))
        l["vert_offset_correction"] = float(np.round(c["Vertical Offset Correction"], 7))
        l["horiz_offset_correction"] = float(np.round(c["Horizontal Offset Correction"], 7))
        l["focal_distance"] = float(np.round(c["Focal Distance"], 7))
        l["focal_slope"] = float(np.round(c["Focal Slope"], 7))
        l["min_intensity"] = c["Min Intensity"]
        l["max_intensity"] = c["Max Intensity"]
        calib["lasers"].append(l)
    calib["status"] = {k: v for k, v in metadata.items() if k != "Calibration"}
    return calib


def read_calibration_from_pcap(pcap_file):
    packets = iter_pcap_packets(pcap_file)
    metadata = read_status_info(packets)
    calib_dict = status_info_to_calib_dict(metadata)
    if calib_dict["status"]["Calibration Time"] is None:
        raise ValueError("Calibration info not available in the metadata")
    if [l["laser_id"] for l in calib_dict["lasers"]] != list(range(64)):
        raise ValueError("Invalid calibration data")
    yaml_str = yaml.dump(calib_dict, sort_keys=False)
    return Calibration.from_string(yaml_str)


def cli(args=None):
    parser = argparse.ArgumentParser(description="Extract HDL-64E calibration from a .pcap file")
    parser.add_argument("pcap_file", help=".pcap file to read")
    args = parser.parse_args(args)
    packets = iter_pcap_packets(args.pcap_file)
    metadata = read_status_info(packets)
    calib_dict = status_info_to_calib_dict(metadata)
    if calib_dict["status"]["Calibration Time"] is None:
        print("Warning: calibration info not found in the metadata", file=sys.stderr)
    if [l["laser_id"] for l in calib_dict["lasers"]] != list(range(64)):
        print("Warning: invalid calibration data", file=sys.stderr)
    print(yaml.dump(calib_dict, sort_keys=False))
