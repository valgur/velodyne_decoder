import tempfile
from pathlib import Path

import pytest
import requests

data_dir = Path(tempfile.tempdir) / "velodyne_decoder_data"
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
