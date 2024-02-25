# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

from pathlib import Path

import pytest
import requests

# logging with rospy from rosbag hangs in Docker otherwise due to
# https://github.com/ros/ros_comm/blob/842f0f02/tools/rosgraph/src/rosgraph/roslogging.py#L64-L71
# going into an infinite loop for some reason.
import logging
logging.getLogger("rosout").setLevel(logging.CRITICAL)

root_dir = Path(__file__).parent.parent
data_dir = Path(__file__).parent / "data"
base_url = "https://github.com/valgur/velodyne_decoder/releases/download/v1.0.1/"


def fetch(name):
    url = base_url + name
    if not data_dir.exists():
        data_dir.mkdir()
    path = data_dir / name
    if not path.exists():
        with path.open("wb") as f:
            f.write(requests.get(url).content)
    return path


@pytest.fixture
def sample_pcap_path():
    return fetch("vlp16.pcap")


@pytest.fixture
def sample_bag_path():
    return fetch("vlp16.bag")


@pytest.fixture(scope="module")
def pcl_struct_dtype():
    return {
        "names": ["x", "y", "z", "intensity", "time", "column", "ring", "return_type"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "<u2", "u1", "u1"],
        "offsets": [0, 4, 8, 12, 16, 20, 22, 23],
        "itemsize": 32,
    }


@pytest.fixture(scope="module")
def calib_data_dir():
    return root_dir / "src/velodyne_decoder/calibrations"
