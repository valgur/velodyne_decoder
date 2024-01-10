import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, CMakeDeps, cmake_layout
from conan.tools.files import copy

required_conan_version = ">=1.52.0"


class VelodyneDecoderConan(ConanFile):
    name = "velodyne_decoder"
    description = "Decoder for raw Velodyne packet data"
    version = "3.0.0-pre"
    license = "BSD-3-Clause"
    url = "https://github.com/valgur/velodyne_decoder"

    package_type = "library"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
    }

    exports_sources = ["include/*", "src/*", "test/*", "docs/*", "cmake/*", "CMakeLists.txt"]

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("yaml-cpp/0.8.0")
        self.requires("ms-gsl/4.0.0", transitive_headers=True)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(self, "LICENSE", self.source_folder, os.path.join(self.package_folder, "licenses"))
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "velodyne_decoder")
        self.cpp_info.set_property("cmake_target_name", "velodyne_decoder::velodyne_decoder")
        self.cpp_info.set_property("pkg_config_name", "velodyne_decoder")

        self.cpp_info.libs = ["velodyne_decoder"]
