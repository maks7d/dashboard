#!/usr/bin/env python3
"""Build script for STM32 project - Cross-platform (Windows/macOS/Linux)"""

import os
import sys
import subprocess
import shutil


def get_nproc():
    """Get number of available CPU cores (cross-platform)"""
    return os.cpu_count() or 4


def main():
    # Detect build type
    build_type = "Debug" if len(sys.argv) > 1 and sys.argv[1].lower() == "debug" else "Release"
    print(f"{build_type} build")

    # Detect generator: Ninja if available, otherwise Make
    generator = "Ninja"
    if shutil.which("ninja") is None:
        generator = "Unix Makefiles"
        print("Ninja not found, falling back to Make")

    nproc = get_nproc()
    print(f"Using {nproc} parallel jobs")

    # Clean and create build directory
    build_dir = "build"
    if os.path.exists(build_dir):
        shutil.rmtree(build_dir)
    os.makedirs(build_dir)

    # Run CMake configuration
    cmake_cmd = [
        "cmake",
        "-DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-G{generator}",
        ".."
    ]
    print("Configuring...")
    subprocess.run(cmake_cmd, cwd=build_dir, check=True)

    # Run build
    build_cmd = [
        "cmake",
        "--build", ".",
        "--target", "all",
        "--", f"-j{nproc}"
    ]
    print("Building...")
    subprocess.run(build_cmd, cwd=build_dir, check=True)

    print("Build completed!")


if __name__ == "__main__":
    main()
