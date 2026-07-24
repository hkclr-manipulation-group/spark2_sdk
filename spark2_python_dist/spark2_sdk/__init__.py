import os
import sys
import platform
import ctypes

# Get the absolute path of the spark2_sdk package directory
_current_dir = os.path.dirname(os.path.abspath(__file__))
_machine = platform.machine().lower()
_system = platform.system().lower()

if "x86_64" in _machine or "amd64" in _machine:
    _platform_name = "win_x64" if "windows" in _system else "linux_x86_64"
elif "arm64" in _machine or "aarch64" in _machine:
    _platform_name = "win_arm64" if "windows" in _system else "linux_arm64"
else:
    raise ImportError(f"Unsupported system architecture: {platform.machine()}")

_toolchain = ""
if "windows" in _system:
    _sys_ver = sys.version.lower()
    if "msc v.194" in _sys_ver:  # MSC V.1940+ corresponds to VS2022
        _toolchain = "msvc2022_v143"
    elif "msc v.192" in _sys_ver:  # MSC V.1920-1929 corresponds to VS2019
        _toolchain = "msvc2019_v142"
    else:
        _toolchain = "msvc2022_v143"
elif "linux" in _system:
    _toolchain = "gcc_linux"

_binary_dir = os.path.join(_current_dir, "lib", _platform_name, _toolchain)

if not os.path.isdir(_binary_dir):
    raise ImportError(
        f"Spark2 native library directory not found: {_binary_dir}\n"
        "Rebuild the C++ SDK with BUILD_PYTHON_LIB=ON, or install a matching wheel."
    )

# Make the extension module importable without PYTHONPATH hacks.
if _binary_dir not in sys.path:
    sys.path.insert(0, _binary_dir)

# Resolve libspark2_sdk next to the extension (no LD_LIBRARY_PATH required).
if "windows" in _system:
    os.add_dll_directory(_binary_dir)
    _engine = os.path.join(_binary_dir, "spark2_sdk.dll")
    if os.path.isfile(_engine):
        ctypes.WinDLL(_engine)
else:
    _engine = os.path.join(_binary_dir, "libspark2_sdk.so")
    if os.path.isfile(_engine):
        ctypes.CDLL(_engine, mode=ctypes.RTLD_GLOBAL)

# Import the wrapper classes
from .spark2 import Spark2
from .configurator import Configurator
from .kinematics import Kinematics
from .types import *
from . import types

del os, sys, platform, ctypes, _current_dir, _machine, _system, _platform_name, _toolchain, _binary_dir, _engine
if '_sys_ver' in locals():
    del _sys_ver

__all__ = ["Spark2", "Configurator", "Kinematics"] + getattr(types, "__all__", [])
