import sys

try:
    from setuptools import find_packages
    from skbuild import setup
except ImportError:
    print(
        "The preferred way to invoke 'setup.py' is via pip, as in 'pip "
        "install .'. If you wish to run the setup script directly, you must "
        "first install the build dependencies listed in pyproject.toml!",
        file=sys.stderr,
    )
    raise

setup(
    packages=find_packages(),
    cmake_install_dir="velodyne_decoder",
)
