"""Setup script for Bevel engine with optional Jolt Physics C++ extension."""

import os
import subprocess
import sys
from pathlib import Path

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """A CMake-based extension module."""

    def __init__(self, name: str, sourcedir: str = "") -> None:
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """Custom build_ext command for CMake extensions."""

    def build_extension(self, ext: CMakeExtension) -> None:
        if not isinstance(ext, CMakeExtension):
            super().build_extension(ext)
            return

        # Check if CMake is available
        try:
            subprocess.check_output(["cmake", "--version"])
        except (OSError, subprocess.CalledProcessError):
            print("CMake not found. Skipping Jolt Physics bindings.")
            print("Install CMake and pybind11 to enable 3D physics support.")
            return

        # Check if pybind11 is available
        try:
            import pybind11
        except ImportError:
            print("pybind11 not found. Skipping Jolt Physics bindings.")
            print("Install pybind11 to enable 3D physics support: pip install pybind11")
            return

        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # Ensure extdir ends with a path separator
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE=Release",
        ]

        build_args = ["--config", "Release"]

        # Parallel build
        if hasattr(os, "cpu_count") and os.cpu_count() is not None:
            build_args += ["-j", str(os.cpu_count())]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        try:
            subprocess.check_call(
                ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
            )
            subprocess.check_call(
                ["cmake", "--build", "."] + build_args, cwd=self.build_temp
            )
        except subprocess.CalledProcessError as e:
            print(f"CMake build failed: {e}")
            print("Jolt Physics bindings will not be available.")
            # Mark this extension as failed so it won't be installed
            self.failed_extensions = getattr(self, 'failed_extensions', [])
            self.failed_extensions.append(ext.name)

    def build_extensions(self):
        """Build extensions, skipping those that fail."""
        # Filter out failed extensions before building
        self.failed_extensions = []
        
        # Build each extension
        for ext in self.extensions:
            try:
                self.build_extension(ext)
            except Exception as e:
                print(f"Failed to build extension {ext.name}: {e}")
                self.failed_extensions.append(ext.name)
        
        # Remove failed extensions from the list so they don't get copied
        self.extensions = [
            ext for ext in self.extensions 
            if ext.name not in self.failed_extensions
        ]


# Check if we should try to build Jolt bindings
jolt_source_dir = "bevel/physics/jolt_bindings"
ext_modules = []

if os.path.exists(jolt_source_dir):
    ext_modules.append(
        CMakeExtension("bevel.physics.jolt_bindings._jolt", jolt_source_dir)
    )

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": CMakeBuild} if ext_modules else {},
)
