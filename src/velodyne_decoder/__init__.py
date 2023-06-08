# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

from enum import IntEnum

# noinspection PyUnresolvedReferences
from .velodyne_decoder_pylib import (
    Config,
    ScanDecoder,
    StreamDecoder,
    Calibration,
    VelodynePacket,
    TelemetryPacket,
    PacketVector,
    Model,
    ReturnMode,
    PACKET_SIZE,
    TELEMETRY_PACKET_SIZE,
    __version__ as _pylib_version,
)

from . import hdl64e
from . import util
from .calibrations import get_bundled_calibration
from .pcap import read_pcap
from .bag import read_bag


class PointField(IntEnum):
    """Point field indices in the decoded point cloud for contiguous NumPy arrays."""
    x = 0
    y = 1
    z = 2
    intensity = 3
    time = 4
    column = 5
    ring = 6
    return_type = 7


__version__ = _pylib_version
