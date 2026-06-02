import os
import sys
import platform

# Get the absolute path of the spark2_sdk package directory
_current_dir = os.path.dirname(os.path.abspath(__file__))
_machine = platform.machine().lower()

if "x86_64" in _machine or "amd64" in _machine:
    _arch_folder = "x86"
elif "arm64" in _machine or "aarch64" in _machine:
    _arch_folder = "arm64"
else:
    raise ImportError(f"Unsupported system architecture: {platform.machine()}")

_binary_dir = os.path.join(_current_dir, "lib", _arch_folder)

if os.path.isdir(_binary_dir) and _binary_dir not in sys.path:
    sys.path.insert(0, _binary_dir)

# Import the wrapper classes
from .spark2 import Spark2
from .configurator import Configurator
from .kinematics import Kinematics
from .types import *
from . import types

del os, sys, platform, _current_dir, _machine, _arch_folder, _binary_dir

__all__ = ["Spark2", "Configurator", "Kinematics"] + getattr(types, "__all__", [])