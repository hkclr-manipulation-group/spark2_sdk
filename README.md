# Spark2 SDK

Spark2 SDK provides a unified API to control the Spark2 robotic arm: joint-space and Cartesian motion, teach/playback modes, configuration, and kinematics. The repository ships both a **C++20** library and a **Python 3.8+** package. The Python API is a thin wrapper over the same C++ core, exposed via pybind11 as `_spark2_sdk_cpp`.

End-user distributions are provided under `spark2_cpp_dist/` and `spark2_python_dist/`. Robot configuration is bundled in `configuration/`. The `cpp/` directory contains the source tree and build system for SDK developers.

## Repository layout

```text
spark2_sdk/
|-- README.md
|-- configuration/                Bundled robot configuration (URDF, YAML, meshes)
|   |-- config.yaml
|   `-- share/
|-- cpp/                          Source SDK and build system (developers)
|   |-- CMakeLists.txt
|   |-- include/                  Public headers
|   |-- src/                      Implementation and pybind11 bindings
|   `-- examples/                 C++ example programs (source)
|-- spark2_cpp_dist/              Pre-built C++ distribution (end users)
|   |-- CMakelists.txt
|   |-- include/                  Public headers
|   |-- lib/
|   |   `-- <platform>/           e.g. win_x64/msvc2022_v143, linux_x86_64/gcc_linux
|   |-- examples/                 C++ example programs
|   `-- spark2_sdk_cpp.md         C++ API reference
|-- spark2_python_dist/           Python distribution (end users)
|   |-- pyproject.toml
|   |-- spark2_sdk_python.md      Python API reference
|   |-- spark2_sdk/               Installable package
|   |   |-- lib/
|   |   |   `-- <platform>/<toolchain>/   Pre-built `_spark2_sdk_cpp` + runtime DLL/SO
|   |   |-- spark2.py
|   |   |-- configurator.py
|   |   |-- kinematics.py
|   |   `-- types.py
|   `-- examples/                 Python example scripts
```

## Prerequisites

**End users** (pre-built distributions)

- **C++**: CMake ≥ 3.20, C++20 compiler matching the shipped binary (e.g. MSVC 2022 on Windows)
- **Python**: Python ≥ 3.8

**Developers** (building from `cpp/`)

This SDK depends on sibling repositories under the same parent directory (e.g. `cuarm_panel_control/`):

| Dependency | Purpose |
|---|---|
| [`cuarm_robotics`](../cuarm_robotics) | Robotics runtime (`robotics_lib`, networking, Pinocchio, etc.) |
| [`cuarm_configuration`](../cuarm_configuration) | Source arm configuration used during development builds |
| [`curi_udp`](../curi_udp) | Low-latency UDP communication for real-time telemetry and state data. |
| [`curi_tcp`](../curi_tcp) | Reliable TCP streaming for commands and system configuration messages. |
| [`robot_platform_utils`](../robot_platform_utils) | General cross-platform helper functions, logging systems, and system utilities. |

- CMake ≥ 3.20, C++20 compiler
- Python ≥ 3.8 with development headers and [pybind11](https://pybind11.readthedocs.io/)

## Quick start: C++ (pre-built distribution)

From the repository root:

```bash
cd spark2_cpp_dist
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This links against the pre-built `spark2_sdk` library under `spark2_cpp_dist/lib/<platform>/` and produces example executables in `spark2_cpp_dist/build/`. On Windows, `spark2_sdk.dll` is copied next to each executable automatically.

Run an example (config path is baked in at compile time and points to the bundled `configuration/` directory):

```bash
./build/move_point        # Linux
build\Release\move_point  # Windows (MSVC)
```

## Quick start: Python (pre-built distribution)

Pre-built native modules live under `spark2_python_dist/spark2_sdk/lib/<platform>/<toolchain>/` (e.g. `win_x64/msvc2022_v143/_spark2_sdk_cpp.cp310-win_amd64.pyd`).

Install the package:

```bash
cd spark2_python_dist
pip install -e .
```

Quick import check:

```bash
python -c "from spark2_sdk import Spark2; print(Spark2)"
```

Run an example:

```bash
cd spark2_python_dist/examples
python move_point.py
```

Examples resolve configuration from the bundled `configuration/` directory at the repository root.

## Build from source (developers)

Use `cpp/` when you need to modify the SDK or rebuild binaries for a new platform, toolchain, or Python version.

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces:

- Shared library `spark2_sdk` in `cpp/build/`
- Example executables in `cpp/build/`
- Python native module `_spark2_sdk_cpp` and runtime library under `spark2_python_dist/spark2_sdk/lib/<platform>/<toolchain>/`
- On successful build, auto-deploys headers, examples, and binaries to `spark2_cpp_dist/`, and syncs configuration into `configuration/`

Use the same Python interpreter you plan to run against; the extension name includes the Python version (e.g. `cp310-win_amd64`).

Then install the Python package:

```bash
cd spark2_python_dist
pip install -e .
```

## Configuration

Examples and the SDK expect a configuration prefix directory. The bundled layout is:

```text
configuration/
|-- config.yaml
`-- share/
    |-- config/assembly/no_gripper.yaml
    |-- config/components/arm_v1.yaml
    |-- no_gripper/arm_v1.urdf
    `-- meshes/arm/
```

At runtime you can:

- Rely on the default used by examples: the `configuration/` directory at the repository root
- Set `CONFIG_PREFIX_PATH` to that directory, or
- Pass a prefix to the constructor: `Spark2(config_prefix_path="/path/to/configuration")`

C++ distribution examples receive `CONFIG_PREFIX_PATH` at compile time via CMake. Python examples use `get_config_prefix_path()` in `spark2_python_dist/examples/utils.py`.

## Examples

| Example | C++ source (`cpp/examples/`) | C++ dist (`spark2_cpp_dist/examples/`) | Python (`spark2_python_dist/examples/`) |
|---|---|---|---|
| Single-point motion | `move_point.cpp` | `move_point.cpp` | `move_point.py` |
| Path motion | `move_path.cpp` | `move_path.cpp` | `move_path.py` |
| Teach and Playback | `teach_and_playback.cpp` | `teach_and_playback.cpp` | `teach_and_playback.py` |
| Configuration | `set_configuration.cpp` | `set_configuration.cpp` | `set_configuration.py` |
| Kinematics | `calculate_kinematics.cpp` | `calculate_kinematics.cpp` | `calculate_kinematics.py` |

**Python**

```bash
cd spark2_python_dist/examples
python move_point.py
```

**C++ (distribution)**

```bash
cd spark2_cpp_dist/build
./move_point
```

**C++ (source build)**

```bash
cd cpp/build
./move_point
```

## API documentation

- C++: [`spark2_cpp_dist/spark2_sdk_cpp.md`](spark2_cpp_dist/spark2_sdk_cpp.md)
- Python: [`spark2_python_dist/spark2_sdk_python.md`](spark2_python_dist/spark2_sdk_python.md)

## Typical workflow summary

**End user**

1. Clone or download this repository.
2. **C++**: configure and build in `spark2_cpp_dist/`, then run examples from `spark2_cpp_dist/build/`.
3. **Python**: `pip install -e spark2_python_dist` and run scripts from `spark2_python_dist/examples/`.
4. Point the SDK at the bundled `configuration/` directory (examples do this automatically).

**Developer**

1. Ensure sibling dependencies are available next to this repo.
2. Build in `cpp/`; outputs are synced to `spark2_cpp_dist/`, `spark2_python_dist/spark2_sdk/lib/`, and `configuration/`.
3. Rebuild in `cpp/` when changing Python version, platform, or toolchain.
4. `pip install -e spark2_python_dist` and run examples or your own application.
