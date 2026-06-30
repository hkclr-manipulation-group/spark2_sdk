# Spark2 SDK Internal Developer Guide

This workspace handles compiling the source tree, managing underlying robotics frameworks, and rebuilding distribution packages for new platforms, toolchains, or Python environments.

---

## 🛠 Prerequisites (Developers)
This SDK requires its sibling modules to reside inside the same unified parent workspace directory (**`cuarm_panel_control/`**). If you are setting up a clean workspace, clone or place these repositories directly alongside `spark2_sdk` to satisfy the relative path configurations:

```text
cuarm_panel_control/
├── cuarm_rt_control/              # Real-time control stack reference framework
│   └── cuarm_robotics/            # Internal core physics & communication algorithms
├── cuarm_configuration/           # Arm geometries, calibrations, and kinematic definitions
├── curi_udp/                      # Low-latency real-time telemetry protocols
├── curi_tcp/                      # Reliable task streaming and state machines
├── robot_platform_utils/          # System trackers, logging, and OS helpers
└── spark2_sdk/                    # This repository
    └── cpp/                       # C++ source compilation entry point
```

| Dependency | Purpose |
|---|---|
| [`cuarm_robotics`](../cuarm_robotics) | Robotics runtime (`robotics_lib`, networking, Pinocchio, etc.) |
| [`cuarm_configuration`](../cuarm_configuration) | Source arm configuration used during development builds |
| [`curi_udp`](../curi_udp) | Low-latency UDP communication for real-time telemetry and state data. |
| [`curi_tcp`](../curi_tcp) | Reliable TCP streaming for commands and system configuration messages. |
| [`robot_platform_utils`](../robot_platform_utils) | General cross-platform helper functions, logging systems, and system utilities. |

- CMake ≥ 3.20, C++20 compiler
- Python ≥ 3.8 with development headers and [pybind11](https://readthedocs.io)

### Third-Party Dependencies

This SDK can be compiled against either **custom static dependencies** (for standalone distribution) or standard **dynamic shared libraries** (such as a Conda environment development tree).

👉 **If you intend to build standalone binaries using static dependencies, refer to the companion guide: [`BUILD_DEPENDENCIES.md`](BUILD_DEPENDENCIES.md) for step-by-step instructions on building those assets.**

👉 **If you are building against dynamic libraries, ensure your local workspace matches the environment specifications documented inside the [`cuarm_robotics/README.md`](../cuarm_rt_control/cuarm_robotics/README.md) reference framework.**

---

## 🔨 Build from Source (Developers)

### 🐧 Linux (Ubuntu x86_64)

Select **one** of the following compilation strategies depending on your local toolchain environment:

#### Option 1: Link Against Static Dependencies (Standalone Bundle)
Ensure that your `$PREFIX` environment variable is set to the folder where you compiled your custom static third-party dependencies (e.g., `$HOME/pkgs/install`).

```bash
cd cpp
mkdir build && cd build
cmake .. -G "Ninja" \
  -DCMAKE_PREFIX_PATH="\$PREFIX" \
  -DROBOTICS_STATIC=ON
cmake --build . --parallel
```

#### Option 2: Link Against Dynamic Dependencies (Conda / Shared Libraries)
Use this option when compiling against a pre-packaged environment (like Anaconda or Miniconda) where dependencies are built as shared system objects (`.so`). 

```bash
cd cpp
mkdir build && cd build
cmake -DROBOTICS_STATIC=OFF ..
cmake --build . --parallel
```

---

### 🖥️ Windows (x64 MSVC)

Select **one** of the following compilation strategies depending on your local toolchain environment:

#### Option 1: Link Against Static Dependencies (Default/Standalone Bundle)
Ensure that your `%PREFIX%` environment variable is set to the folder where you compiled your static third-party dependencies (e.g., `D:\pkgs\install`).

```bat
cd cpp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="%PREFIX%" -DROBOTICS_STATIC=ON ..
cmake --build . --config Release --parallel
```

#### Option 2: Link Against Dynamic Dependencies (Conda / Shared Libraries)
Use this option when compiling against a pre-packaged environment (like Anaconda or Miniconda) where dependencies such as Pinocchio are built as shared DLLs. Set `-DROBOTICS_STATIC=OFF` to drop static compiler macro flags and force standard dynamic DLL linkage layouts. 

*Replace `<CONDA_ENV_PATH>` with your literal absolute conda path (e.g., `D:/miniconda3/envs/pinocchio_env`).*

```bat
cd cpp
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="<CONDA_ENV_PATH>;<CONDA_ENV_PATH>/Library" -DROBOTICS_STATIC=OFF ..
cmake --build . --config Release --parallel
```

---

## 📦 Output Artifacts & Deployment

A successful build automatically produces and distributes the following targets:

- **Shared Library:** Built as `spark2_sdk` inside your local `cpp/build/` directory.
- **Example Executables:** Native binaries created inside `cpp/build/`.
- **Python Native Module:** Generates the raw `_spark2_sdk_cpp` extension binary under `spark2_python_dist/spark2_sdk/lib/<platform>/<toolchain>/`.
- **Auto-Deployment Core:** Post-build hooks instantly copy fresh interface headers, runtime examples, and compiled libraries directly into `spark2_cpp_dist/`, while keeping the global `configuration/` layout synchronized.
