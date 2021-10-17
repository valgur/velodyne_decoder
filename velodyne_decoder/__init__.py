from contextlib import contextmanager

import dpkt
import velodyne_decoder_pylib as vd
from velodyne_decoder_pylib import *


def read_pcap(pcap_file, config, as_pcl_structs=False):
    decoder = vd.StreamDecoder(config)
    with _fopen(pcap_file, "rb") as f:
        for stamp, buf in dpkt.pcap.Reader(f):
            data = dpkt.ethernet.Ethernet(buf).data.data.data
            if len(data) != vd.PACKET_SIZE:
                continue
            result = decoder.decode(stamp, data, as_pcl_structs)
            if result is not None:
                yield result


@contextmanager
def _fopen(filein, *args, **kwargs):
    if isinstance(filein, str):  # filename
        with open(filein, *args, **kwargs) as f:
            yield f
    else:  # file-like object
        yield filein
