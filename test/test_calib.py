# Copyright (c) 2021-2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

import math

import pytest
import velodyne_decoder as vd
import yaml

MODELS = list(vd.Model.__members__)


def test_config_constructor(calib_data_dir):
    vd.Config()
    vd.Config(model=vd.Model.VLP16)
    calibration = vd.Calibration.read(str(calib_data_dir / "VLP-16.yml"))
    vd.Config(model=vd.Model.VLP16, calibration=calibration)


@pytest.mark.parametrize("model_id", MODELS)
def test_bundled_calibrations(model_id):
    model_id = vd.Model.__members__[model_id]
    calib = None
    if model_id.name.startswith("HDL64E"):
        calib_file = {
            vd.Model.HDL64E_S1: "HDL-64E_S1-utexas.yml",
            vd.Model.HDL64E_S2: "HDL-64E_S2.1-sztaki.yml",
            vd.Model.HDL64E_S3: "HDL-64E_S3-VeloView.yml",
        }[model_id]
        calib = vd.get_bundled_calibration(calib_file)
    vd.ScanDecoder(vd.Config(model=model_id, calibration=calib))


@pytest.mark.parametrize("model_id", [m for m in MODELS if not m.startswith("HDL64E")])
def test_default_calib_vertical_offsets(model_id):
    """Check that the pre-defined vertical offset calibration values are sensible."""
    model_id = vd.Model.__members__[model_id]
    calib = vd.Calibration.default_calibs[model_id]
    lasers = yaml.safe_load(calib.to_string())["lasers"]
    focal_distance = {
        vd.Model.AlphaPrime: 58.63e-3,
        vd.Model.HDL32E: 28.83e-3,
        vd.Model.PuckHiRes: 41.91e-3,
        vd.Model.VLP16: 41.91e-3,
        vd.Model.VLP32A: 42.4e-3,
        vd.Model.VLP32B: 42.4e-3,
        vd.Model.VLP32C: 42.4e-3,
    }[model_id]
    for laser in lasers:
        approx_offset = focal_distance * math.tan(-laser["vert_correction"])
        thr = 0.5e-3 if model_id == vd.Model.PuckHiRes else 0.1e-3
        assert pytest.approx(approx_offset, abs=thr) == laser["vert_offset_correction"]
