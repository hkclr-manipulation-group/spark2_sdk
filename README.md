# Spark2 SDK

Spark2 SDK provides a unified API to control the Spark2 robotic arm: joint-space and Cartesian motion, teach/playback modes, configuration, and kinematics. The repository ships both a **C++20** library and a **Python 3.8+** package. The Python API is a thin wrapper over the same C++ core, exposed via pybind11 as `_spark2_sdk_cpp`.

## Repository layout

```text
spark2_sdk/
|-- README.md
|-- cpp/                          C++ SDK and build system
|   |-- CMakeLists.txt
|   |-- include/                  Public headers
|   |-- src/                      Implementation and pybind11 bindings
|   |-- examples/                 C++ example programs
|   `-- spark2_sdk_cpp.md         C++ API reference
|-- python/                       Python package
|   |-- pyproject.toml
|   |-- spark2_sdk_python.md      Python API reference
|   |-- spark2_sdk/               Installable package
|   |   |-- lib/
|   |   |   |-- x86/              Pre-built `_spark2_sdk_cpp*.so` (x86_64)
|   |   |   `-- arm64/            Pre-built `_spark2_sdk_cpp*.so` (aarch64)
|   |   |-- spark2.py
|   |   |-- configurator.py
|   |   |-- kinematics.py
|   |   `-- types.py
|   `-- examples/                 Python example scripts
```

## Prerequisites

This SDK depends on sibling repositories under the same parent directory (e.g. `cuarm_panel_control/`):

| Dependency | Purpose |
|---|---|
| [`cuarm_rt_control`](../cuarm_rt_control) | Robotics runtime (`robotics_lib`, networking, Pinocchio, etc.) |
| [`cuarm_configuration`](../cuarm_configuration) | Arm configuration files (`arm_v1`, limits, URDF, etc.) |

**C++ build**

- CMake ≥ 3.20, C++20 compiler
- When using pre-built `cuarm_rt_control` artifacts: Pinocchio, yaml-cpp, Eigen3, Boost

**Python build** (only when building the native extension)

- Python ≥ 3.8 with development headers
- [pybind11](https://pybind11.readthedocs.io/) installed for that interpreter (`pip install pybind11`)

Build `cuarm_rt_control` first if its libraries are not already present under `cuarm_rt_control/build/`. The Spark2 CMake project will compile from source as a fallback when pre-built libraries are missing.

## Build: C++ SDK

From the repository root:

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces:

- Static library `libspark2_sdk.a` in `cpp/build/`
- Example executables in `cpp/build/` (e.g. `move_point`, `move_path`, `teach_mode`)

By default, `BUILD_PYTHON_LIB` is **OFF** — a plain C++ build does not produce the pybind11 module. See [Build: Python native extension](#build-python-native-extension) when you need Python support.

Run a C++ example (config path is baked in at compile time relative to the parent workspace):

```bash
./build/move_point
```

## Build: Python native extension

The Python package loads `_spark2_sdk_cpp` from `python/spark2_sdk/lib/<arch>/`. That shared object is **not** built by `pip install` alone; it must come from the C++ CMake build.

**Check whether a `.so` already exists for your architecture:**

```bash
# x86_64
ls python/spark2_sdk/lib/x86/_spark2_sdk_cpp*.so

# aarch64
ls python/spark2_sdk/lib/arm64/_spark2_sdk_cpp*.so
```

If no matching `.so` is present, build it from `cpp/` with `BUILD_PYTHON_LIB=ON`:

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_LIB=ON
cmake --build build -j
```

If you already configured the build directory with the default (`OFF`), re-run `cmake` with `-DBUILD_PYTHON_LIB=ON` before building.

CMake writes the module to:

- `python/spark2_sdk/lib/x86/_spark2_sdk_cpp*.so` on x86_64
- `python/spark2_sdk/lib/arm64/_spark2_sdk_cpp*.so` on aarch64

Use the same Python interpreter you plan to run against; the extension name includes the Python version (e.g. `cpython-312-x86_64-linux-gnu`).

## Install: Python package

After the native `.so` is in place:

```bash
cd python
pip install -e .
```

Quick import check:

```bash
python3 -c "from spark2_sdk import Spark2; print(Spark2)"
```

## Configuration

Examples and the SDK expect configuration under `cuarm_configuration/arm_v1`. CMake sets a default prefix at build time; at runtime you can:

- Set `CONFIG_PREFIX_PATH` to the `cuarm_configuration` directory (the SDK appends `/arm_v1`), or
- Pass a prefix to the constructor: `Spark2(config_prefix_path="/path/to/cuarm_configuration/arm_v1")`

Python helpers in `spark2_sdk.utils` can auto-discover `cuarm_configuration` by walking up from the package directory.

## Examples

| Example | C++ (`cpp/examples/`) | Python (`python/examples/`) |
|---|---|---|
| Single-point motion | `move_point.cpp` | `move_point.py` |
| Path motion | `move_path.cpp` | `move_path.py` |
| Teach and Playback | `teach_and_playback.cpp` | `teach_and_playback.py` |
| Configuration | `set_configuration.cpp` | `set_configuration.py` |
| Kinematics | `calculate_kinematics.cpp` | `calculate_kinematics.py` |

**Python**

```bash
cd python/examples
python3 move_point.py
```

**C++**

```bash
cd cpp/build
./move_point
```

## API documentation

- C++: [`cpp/spark2_sdk_cpp.md`](cpp/spark2_sdk_cpp.md)
- Python: [`python/spark2_sdk_python.md`](python/spark2_sdk_python.md)

## Typical workflow summary

1. Ensure `cuarm_rt_control` (and optionally pre-built libs) and `cuarm_configuration` are available next to this repo.
2. For C++ only: build in `cpp/` (default). For Python: build with `-DBUILD_PYTHON_LIB=ON` or use a pre-built `_spark2_sdk_cpp*.so`.
3. Confirm `_spark2_sdk_cpp*.so` exists under `python/spark2_sdk/lib/<arch>/`; rebuild with `BUILD_PYTHON_LIB=ON` if you change Python version or architecture.
4. `pip install -e python` and run examples or your own application.
