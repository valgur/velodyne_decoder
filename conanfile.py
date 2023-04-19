import os

from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.files import copy

required_conan_version = ">=1.52.0"


class VelodyneDecoderConan(ConanFile):
    name = "velodyne_decoder"
    version = "3.0.0"
    package_type = "library"

    description = "Decoder for raw Velodyne packet data"
    url = "https://github.com/valgur/velodyne_decoder"
    license = "BSD-3-Clause"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_python": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "with_python": True,
    }
    generators = "CMakeDeps", "CMakeToolchain"

    exports_sources = ["include/*", "src/*", "test/*", "docs/*", "cmake/*", "CMakeLists.txt"]

    def requirements(self):
        self.requires("yaml-cpp/0.7.0", headers=True, libs=True)
        if self.options.with_python:
            self.requires("pybind11/2.10.1", headers=True, libs=True)

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(
            self,
            pattern="*LICENSE",
            src=self.source_folder,
            dst=os.path.join(self.package_folder, "licenses"),
        )
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = [self.name]
