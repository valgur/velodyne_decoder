# Copyright (c) 2023, Martin Valgur
# SPDX-License-Identifier: BSD-3-Clause

"""
Except for HDL-64E, the calibration YAML files are for reference only.
The equivalent data is now stored in src/calib_db.cpp instead.
"""

import importlib_resources

import velodyne_decoder as vd


def get_bundled_calibration(calib_file):
    """Return a Calibration object for the given calibration file.

    E.g. get_bundled_calibration("HDL-64E_S3-VeloView.yml").
    """
    path = importlib_resources.files(__package__) / calib_file
    with importlib_resources.as_file(path) as calib_path:
        return vd.Calibration.read(str(calib_path))
