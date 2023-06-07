# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

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

__version__ = _pylib_version
