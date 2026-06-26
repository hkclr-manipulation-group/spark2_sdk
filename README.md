
# Spark2 SDK

Spark2 SDK provides a unified API to control the Spark2 robotic arm: joint-space and Cartesian motion, teach/playback modes, configuration, and kinematics. The repository ships both a **C++20** library and a **Python 3.8+** package. 

End-user distributions are provided under `spark2_cpp_dist/` and `spark2_python_dist/`. Robot configuration is bundled in `configuration/`. The `cpp/` directory contains the source tree and build system exclusively for SDK internal core developers.

## Repository layout

```text
spark2_sdk/
|-- README.md
|-- configuration/                Bundled robot configuration (URDF, YAML, meshes)
|-- cpp/                          Source SDK and build system (developers only)
|-- spark2_cpp_dist/              Pre-built C++ distribution (end users)
|-- spark2_python_dist/           Python distribution (end users)
```

## Prerequisites

**End users** (pre-built distributions)

- **C++**: CMake ≥ 3.20, C++20 compiler matching the shipped binary (e.g. MSVC 2022 on Windows)
- **Python**: Python ≥ 3.8

---

## 🚀 Quick start: C++ (pre-built distribution)

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

---

## 🐍 Quick start: Python (pre-built distribution)

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
cd examples
python move_point.py
```

---

## Configuration

The bundled layout is:

```text
configuration/
|-- config.yaml
`-- share/
```

At runtime you can:
- Rely on the default used by examples: the `configuration/` directory at the repository root
- Set `CONFIG_PREFIX_PATH` to that directory, or
- Pass a prefix to the constructor: `Spark2(config_prefix_path="/path/to/configuration")`

---

## Examples & Documentation

- **C++ Examples:** Built directly from `spark2_cpp_dist/examples/`
- **Python Examples:** Run straight from `spark2_python_dist/examples/`
- **C++ API Reference Manual:** [`spark2_cpp_dist/spark2_sdk_cpp.md`](spark2_cpp_dist/spark2_sdk_cpp.md)
- **Python API Reference Manual:** [`spark2_python_dist/spark2_sdk_python.md`](spark2_python_dist/spark2_sdk_python.md)
