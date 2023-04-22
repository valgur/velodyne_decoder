#!/usr/bin/env python3
# Copyright (c) 2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause
"""
Convert a single-return VLS-128 pcap file into a mock dual-return VLS-128 pcap file.
All returns are both strongest and last.
"""
import struct
from pathlib import Path

import dpkt
from tqdm.auto import tqdm

pcap_path = Path(__file__).parent / "VLS-128_single/VLS-128_single.pcap"
output_path = Path(__file__).parent / "VLS-128_dual/VLS-128_dual.pcap"

PACKET_SIZE = 1206

FIRING_DURATION = (8 / 3) * 20 * 1e-6

with pcap_path.open("rb") as fin, output_path.open("wb") as fout:
    pcap_out = dpkt.pcap.Writer(fout)
    dummy_block = bytearray(100)
    packet_count = sum(1 for _ in dpkt.pcap.Reader(fin))
    fin.seek(0)
    for pcap_stamp, buf in tqdm(dpkt.pcap.Reader(fin), total=packet_count):
        eth = dpkt.ethernet.Ethernet(buf)
        try:
            data = dpkt.ethernet.Ethernet(buf)
        except dpkt.dpkt.UnpackError:
            continue
        while hasattr(data, "data"):
            data = data.data
        if len(data) != PACKET_SIZE:
            pcap_out.writepkt(buf, stamp)
            continue
        # Parse
        blocks = [data[i * 100 : (i + 1) * 100] for i in range(12)]
        stamp, dual_return_mode, model_id = struct.unpack("<LBB", data[-6:])
        # Convert three sequences in a single-return packet into three dual-return packets
        for seq_idx in range(3):
            new_blocks = []
            for block_idx in range(4):
                new_blocks.append(blocks[seq_idx * 4 + block_idx])
                new_blocks.append(blocks[seq_idx * 4 + block_idx])
            dummy_block[2:4] = new_blocks[0][2:4]
            for i in range(4):
                new_blocks.append(dummy_block)
            new_stamp = stamp + int(round(FIRING_DURATION * 1e6 * seq_idx))
            new_buf = b"".join(new_blocks) + struct.pack("<LBB", new_stamp, 0x39, model_id)

            # Re-frame the data and write to PCAP
            ip = eth.data
            udp = ip.data
            udp.data = new_buf
            udp.ulen = len(udp)
            ip.data = udp
            ip.len = len(ip)
            eth = dpkt.ethernet.Ethernet(data=ip)
            pcap_out.writepkt(eth, pcap_stamp + FIRING_DURATION * seq_idx)
