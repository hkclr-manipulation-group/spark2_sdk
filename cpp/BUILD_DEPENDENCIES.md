# Building Core Third-Party Dependencies Statically

This internal guide details how to compile the required static third-party binaries (Pinocchio, Boost, urdfdom) used to back the Spark2 C++ SDK.

<details>
<summary><b style="font-size: 1.5em; cursor: pointer;">🖥️ Windows (x64 MSVC) Compilation Track</b></summary>

This guide walks you through compiling a 100% self-contained static C++ build of Pinocchio and its critical dependencies on Windows using the **Static Runtime (`/MT`)**. 

---

### 🛠 Prerequisites

1. **Visual Studio 2022 (or Build Tools)** with the "Desktop development with C++" workload installed.
2. **CMake** and **Ninja** added to your system PATH.
3. ⚠️ **Crucial:** Always execute all terminal commands inside the **x64 Native Tools Command Prompt for VS 2022**. Regular CMD or PowerShell may fail.

---

### 📁 Step 1: Initialize Workspace Environment

Set your custom folder path as an environment variable so you can copy and paste the commands seamlessly. 

```cmd
:: 🛑 EXAMPLE PATH: Replace "C:\(\path\to\workspace\)" with your actual target directory
set WORKSPACE=C:\(\path\to\workspace\)
set PREFIX=%WORKSPACE%\install

mkdir "%WORKSPACE%"
mkdir "%PREFIX%"
```

---

### 📦 Step 2: Build Header-Only & Common Dependencies

#### 1. Eigen (Header-Only)
```cmd
cd /d "%WORKSPACE%"
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="%PREFIX%"
cmake --install .
```

#### 2. console_bridge
```cmd
cd /d "%WORKSPACE%"
git clone https://github.com/ros/console_bridge.git
cd console_bridge && mkdir build && cd build
cmake .. -G "Ninja" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_SHARED_LIBS=OFF ^
  -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
  -DCMAKE_INSTALL_PREFIX="%PREFIX%"
ninja && ninja install
```

#### 3. urdfdom_headers (Header-Only)
```cmd
cd /d "%WORKSPACE%"
git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers && mkdir build && cd build
cmake .. -G "Ninja" -DCMAKE_INSTALL_PREFIX="%PREFIX%"
ninja install
```

#### 4. TinyXML2
```cmd
cd /d "%WORKSPACE%"
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && mkdir build && cd build
cmake .. -G "Ninja" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_SHARED_LIBS=OFF ^
  -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
  -DCMAKE_INSTALL_PREFIX="%PREFIX%"
ninja && ninja install
```

#### 5. yaml-cpp
```cmd
cd /d "%WORKSPACE%"
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake .. -G "Ninja" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_SHARED_LIBS=OFF ^
  -DYAML_CPP_BUILD_TESTS=OFF ^
  -DYAML_CPP_BUILD_TOOLS=OFF ^
  -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
  -DCMAKE_INSTALL_PREFIX="%PREFIX%"
ninja && ninja install
```

---

### ✏️ Step 3: Patch and Build Static urdfdom

Because `urdfdom` forces a dynamic configuration by default on Windows, you must manually alter its build scripts before compiling.

1. Clone the repository:
   ```cmd
   cd /d "%WORKSPACE%"
   git clone https://github.com/ros/urdfdom.git
   ```
2. 🛑 **MANUAL STEP:** In Windows File Explorer, navigate inside your workspace folder and open `urdfdom\urdf_parser\CMakeLists.txt` in a text editor.
3. Locate all instances of `SHARED` inside the `add_library` commands (usually 3 places: `urdfdom_model`, `urdfdom_world`, and `urdfdom_sensor`) and change them to `STATIC`.
4. Return to your command prompt and compile:
   ```cmd
   cd /d "%WORKSPACE%\urdfdom" && mkdir build && cd build
   cmake .. -G "Ninja" ^
     -DCMAKE_BUILD_TYPE=Release ^
     -DBUILD_SHARED_LIBS=OFF ^
     -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
     -DCMAKE_CXX_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG -DURDFDOM_STATIC" ^
     -DCMAKE_C_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG -DURDFDOM_STATIC" ^
     -Dconsole_bridge_DIR="%PREFIX%\lib\console_bridge\cmake" ^
     -DCMAKE_PREFIX_PATH="%PREFIX%" ^
     -DCMAKE_INSTALL_PREFIX="%PREFIX%"
   ninja && ninja install
   ```

---

### 🌐 Step 4: Download and Build Static Boost

1. 🛑 **MANUAL STEP:** Open your web browser, download the latest Boost source `.zip` archive from [boost.org](https://www.boost.org/releases/latest/), and extract it directly into your workspace folder so the path looks like `%WORKSPACE%\boost`.
2. 🛑 **MANUAL STEP:** Open the folder `%WORKSPACE%\boost` in Windows File Explorer and **double-click `bootstrap.bat`**. Wait a few seconds for the command window to automatically process and close. You will see a new `b2.exe` file appear in the directory.
3. Open your **x64 Native Tools Command Prompt** window and run the compiler engine to build the static libraries:
   ```cmd
   cd /d "%WORKSPACE%\boost"
   b2 variant=release link=static runtime-link=static threading=multi toolset=msvc --with-filesystem --with-serialization --prefix="%PREFIX%" install
   ```
4. 🛑 **MANUAL STEP:** Boost automatically nests its headers inside a versioned directory. You must flatten it so CMake can locate it natively. Run these two clean-up lines in your terminal:
   ```cmd
   move "%PREFIX%\include\boost-1_91\boost" "%PREFIX%\include\boost"
   rmdir "%PREFIX%\include\boost-1_91"
   ```
   *(Note: Change `boost-1_91` in the folder path above if you downloaded a different version of Boost).*

---

### 🚀 Step 5: Patch and Build Pinocchio
:: Note: -D_WIN32_WINNT=0x0A00 restricts the compilation target to Windows 10 & 11 only.

1. Clone Pinocchio along with its submodules:
   ```cmd
   cd /d "%WORKSPACE%"
   git clone --recursive https://github.com/stack-of-tasks/pinocchio
   ```
2. 🛑 **MANUAL STEP 1:** Open `pinocchio\src\CMakeLists.txt`. Find the line `set(LIBRARY_TYPE SHARED)` (around line 140) and change it to `set(LIBRARY_TYPE STATIC)` to force the library targets to compile as static archives instead of DLLs.
3. 🛑 **MANUAL STEP 2:** Open `%WORKSPACE%\pinocchio\CMakeLists.txt` in a text editor. Move to the dependency validation area (around lines 315-390). Add the explicit upstream package lookups directly **before** the `urdfdom` dependency blocks:
   ```cmake
   find_package(console_bridge REQUIRED)
   find_package(tinyxml2 REQUIRED)
   
   add_project_dependency(urdfdom_headers REQUIRED)
   add_project_dependency(urdfdom REQUIRED)
   ```
4. Clear your build target namespace and run the final compilation (note the inclusion of `-DBUILD_TESTING=OFF` to prevent Boost unit test macro clashes and `-D_WIN32_WINNT=0x0A00` to target Windows 10/11):
   ```cmd
   cd /d "%WORKSPACE%\pinocchio" && mkdir build && cd build
   cmake .. -G "Ninja" ^
     -DCMAKE_BUILD_TYPE=Release ^
     -DBUILD_SHARED_LIBS=OFF ^
     -DBUILD_TESTING=OFF ^
     -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
     -DCMAKE_CXX_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG -D_WIN32_WINNT=0x0A00 -DURDFDOM_STATIC" ^
     -DCMAKE_C_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG -D_WIN32_WINNT=0x0A00 -DURDFDOM_STATIC" ^
     -DBUILD_WITH_CASADI_SUPPORT=OFF ^
     -DBUILD_WITH_COLLISION_SUPPORT=OFF ^
     -DBUILD_PYTHON_INTERFACE=OFF ^
     -DBoost_USE_STATIC_LIBS=ON ^
     -DBoost_USE_STATIC_RUNTIME=ON ^
     -DBOOST_ROOT="%PREFIX%" ^
     -DCMAKE_PREFIX_PATH="%PREFIX%" ^
     -DCMAKE_INSTALL_PREFIX="%PREFIX%"
   ninja && ninja install
   ```
---

### 🎉 Verification

Once completed, check your local deployment directory inside your workspace at **`%PREFIX%\lib\`**. You should find **`pinocchio_default.lib`** and **`pinocchio_parsers.lib`** as standalone, high-capacity static binaries ready for integration with zero runtime `.dll` requirements!

</details>

<details>
<summary><b style="font-size: 1.5em; cursor: pointer;">🐧 Linux (Ubuntu x86_64) Compilation Track</b></summary>

Unlike Windows, Linux natively supports mixing dynamic system dependencies with a customized static target package. You do not need to build common utilities like Boost or TinyXML2 from source; you can pull their pre-compiled static `.a` variants directly through your package manager. 

However, robotics-specific libraries (Pinocchio, urdfdom, console_bridge) must be compiled from source. The critical rule on Linux is that **every static library must be built with Position Independent Code (`-fPIC`) enabled**, otherwise the linker will fail when trying to bundle them into your final dynamic `robot_sdk.so` library.

---

### 📁 Step 1: Initialize Workspace Environment

Set your custom folder path as an environment variable so you can copy and paste the commands seamlessly.

```bash
# 🛑 EXAMPLE PATH: Replace "\$HOME/pkgs" with your actual target directory
export WORKSPACE=\$HOME/pkgs
export PREFIX=\$WORKSPACE/install

mkdir -p "\$WORKSPACE"
mkdir -p "\$PREFIX"
```

---

### 📦 Step 2: Install System Static Packages & Dependencies

Install the core compilation tools alongside system packages that provide their static `.a` assets natively out of the box:

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  libeigen3-dev \
  libtinyxml2-dev \
  libyaml-cpp-dev \
  libboost-filesystem-dev \
  libboost-serialization-dev \
  liburdfdom-headers-dev
```

---

### 🔨 Step 3: Build Custom Third-Party Static Libraries

#### 1. console_bridge
```bash
cd "\$WORKSPACE"
git clone https://github.com/ros/console_bridge.git
cd console_bridge && mkdir build && cd build

cmake .. -G "Ninja" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX="\$PREFIX"

ninja && ninja install
```

#### 2. urdfdom (With Code Modification)
Because `urdfdom` forces a dynamic configuration by default, you must alter its build scripts before compiling.

```bash
cd "\$WORKSPACE"
git clone https://github.com/ros/urdfdom.git

# 🛑 AUTOMATED PATCH: Replaces 'SHARED' with 'STATIC' inside the CMake files
sed -i 's/ SHARED / STATIC /g' urdfdom/urdf_parser/CMakeLists.txt

cd urdfdom && mkdir build && cd build
cmake .. -G "Ninja" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX="\$PREFIX" \
  -Dconsole_bridge_DIR="\$PREFIX/lib/console_bridge/cmake" \
  -DCMAKE_CXX_FLAGS="-DURDFDOM_STATIC"

ninja && ninja install
```

#### 3. Pinocchio (With CMake Patch)
```bash
cd "\$WORKSPACE"
git clone --recursive https://github.com

# 🛑 AUTOMATED PATCH: Injects explicit upstream packages before the urdfdom block
sed -i '/add_project_dependency(urdfdom_headers REQUIRED)/i find_package(console_bridge REQUIRED)\nfind_package(tinyxml2 REQUIRED)' pinocchio/CMakeLists.txt

cd pinocchio && mkdir build && cd build
cmake .. -G "Ninja" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX="\$PREFIX" \
  -DCMAKE_PREFIX_PATH="\$PREFIX" \
  -DBUILD_WITH_CASADI_SUPPORT=OFF \
  -DBUILD_WITH_COLLISION_SUPPORT=OFF \
  -DBUILD_PYTHON_INTERFACE=OFF

ninja && ninja install
```

---

### 🎉 Verification

Once the compilation finishes, run the `file` utility tool on the target binary assets inside your installation prefix to confirm they are genuine static archives:

```bash
file "\$PREFIX/lib/libpinocchio_default.a"
file "\$PREFIX/lib/libpinocchio_parsers.a"
```

**Expected Output:** Terminal output for both files must explicitly contain the statement: 
`current ar archive`

</details>