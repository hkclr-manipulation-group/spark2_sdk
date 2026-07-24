cmake_minimum_required(VERSION 3.20)
project(spark2_sdk)

option(ROBOTICS_STATIC "Build using static variations of dependencies" OFF)
if(MSVC)
    if(ROBOTICS_STATIC)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")    # /MT
    else()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL") # /MD
    endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

###################################OS Settings###########################################
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    if(WIN32)
        set(PLATFORM_NAME "win_arm64")
    else()
        set(PLATFORM_NAME "linux_arm64")
    endif()
else() 
    if(WIN32)
        set(PLATFORM_NAME "win_x64")
    else()
        set(PLATFORM_NAME "linux_x86_64")
    endif()
endif()

if(MSVC)
    if(MSVC_VERSION VERSION_GREATER_EQUAL 1940)
        set(TOOLCHAIN_VARIANT "msvc2022_v143")
    elseif(MSVC_VERSION VERSION_GREATER_EQUAL 1920)
        set(TOOLCHAIN_VARIANT "msvc2019_v142")
    else()
        set(TOOLCHAIN_VARIANT "msvc_unknown")
    endif()
else()
    set(TOOLCHAIN_VARIANT "gcc_linux")
endif()
set(PYTHON_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../spark2_python_dist/spark2_sdk/lib/${PLATFORM_NAME}/${TOOLCHAIN_VARIANT}")
set(CPP_INSTALL_PATH "lib/${PLATFORM_NAME}/${TOOLCHAIN_VARIANT}")

###################################Build Switches###########################################
option(BUILD_PYTHON_LIB "Build Python library" ON)

###################################Repo layout##############################################
# cpp/ is PROJECT_SOURCE_DIR; parent is the SDK repo root (configuration/, examples dist).
# Dependencies may be either:
#   A) Nested under SDK_ROOT (standalone spark2_sdk package), or
#   B) Siblings of spark2_sdk (workspace: spark2_control/cuarm_panel_control).
cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH SDK_ROOT)
cmake_path(GET SDK_ROOT PARENT_PATH WORKSPACE_ROOT)
set(CONFIG_PATH "${SDK_ROOT}/configuration")

if(EXISTS "${SDK_ROOT}/cuarm_rt_control/cuarm_robotics")
    set(RT_CONTROL_DIR "${SDK_ROOT}/cuarm_rt_control")
elseif(EXISTS "${WORKSPACE_ROOT}/cuarm_rt_control/cuarm_robotics")
    set(RT_CONTROL_DIR "${WORKSPACE_ROOT}/cuarm_rt_control")
else()
    message(FATAL_ERROR
        "Cannot find cuarm_rt_control/cuarm_robotics under:\n"
        "  ${SDK_ROOT}\n"
        "  ${WORKSPACE_ROOT}\n"
        "Place it as a sibling of spark2_sdk or nested under the SDK root.")
endif()
set(CUARM_ROBOTICS_DIR "${RT_CONTROL_DIR}/cuarm_robotics")

# robot_platform_utils expects curi_udp/curi_tcp as siblings of its parent directory.
if(EXISTS "${SDK_ROOT}/robot_platform_utils")
    set(_PLATFORM_UTILS_PARENT "${SDK_ROOT}")
elseif(EXISTS "${WORKSPACE_ROOT}/robot_platform_utils")
    set(_PLATFORM_UTILS_PARENT "${WORKSPACE_ROOT}")
else()
    message(FATAL_ERROR
        "Cannot find robot_platform_utils under ${SDK_ROOT} or ${WORKSPACE_ROOT}")
endif()
set(ROBOT_PLATFORM_UTILS_DIR "${_PLATFORM_UTILS_PARENT}" CACHE PATH "Path containing robot_platform_utils/" FORCE)
set(ROBOT_PLATFORM_UTILS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Binary dir for robot_platform_utils" FORCE)
message(STATUS "SDK_ROOT: ${SDK_ROOT}")
message(STATUS "RT_CONTROL_DIR: ${RT_CONTROL_DIR}")
message(STATUS "ROBOT_PLATFORM_UTILS_DIR: ${ROBOT_PLATFORM_UTILS_DIR}")

#Find the pre-compiled robotics library first
set(LIB_SEARCH_DIR ${RT_CONTROL_DIR}/build)
find_library(ROBOTICS_LIB_FILE
    NAMES robotics_lib
    PATHS ${LIB_SEARCH_DIR}/cuarm_robotics
    NO_DEFAULT_PATH
)

# If NOT found, fallback to adding the source directory
if(NOT ROBOTICS_LIB_FILE)
    message(STATUS "Compiled library not found in ${LIB_SEARCH_DIR}. Falling back to source compilation...")
    if(CMAKE_PREFIX_PATH)
        cmake_path(CONVERT "${CMAKE_PREFIX_PATH}" TO_CMAKE_PATH_LIST CLEAN_PREFIX_PATH)
        set(CMAKE_PREFIX_PATH "${CLEAN_PREFIX_PATH}")
    endif()

    add_subdirectory(${CUARM_ROBOTICS_DIR} ${CMAKE_BINARY_DIR}/cuarm_robotics)
else()
    message(STATUS "Found pre-compiled robotics: ${ROBOTICS_LIB_FILE}")
    find_library(UTILS_FILE
        NAMES robot_platform_utils
        PATHS ${LIB_SEARCH_DIR}/robot_platform_utils
        NO_DEFAULT_PATH
    )

    find_library(UDP_FILS
        NAMES curi_udp
        PATHS ${LIB_SEARCH_DIR}/curi_udp
        NO_DEFAULT_PATH
    )

    find_library(TCP_FILE
        NAMES curi_tcp
        PATHS ${LIB_SEARCH_DIR}/curi_tcp
        NO_DEFAULT_PATH
    )
    
    find_package(pinocchio REQUIRED) 
    find_package(yaml-cpp REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(Boost REQUIRED)
    find_package(OpenSSL REQUIRED)
    if(ROBOTICS_STATIC)
        find_package(console_bridge REQUIRED)
        find_package(tinyxml2 REQUIRED)
    endif()
    add_library(cuarm_robotics::robotics STATIC IMPORTED GLOBAL)
    set_target_properties(cuarm_robotics::robotics PROPERTIES
        IMPORTED_LOCATION "${ROBOTICS_LIB_FILE}"
        INTERFACE_INCLUDE_DIRECTORIES "${RT_CONTROL_DIR};${CUARM_ROBOTICS_DIR}/src;${ROBOT_PLATFORM_UTILS_DIR}/robot_platform_utils/cpp/include;${ROBOT_PLATFORM_UTILS_DIR};${EIGEN3_INCLUDE_DIRS};${Boost_INCLUDE_DIRS};${PINOCCHIO_INCLUDE_DIRS}"
    )

    set_property(TARGET cuarm_robotics::robotics PROPERTY INTERFACE_COMPILE_DEFINITIONS
        "EIGEN_QUATERNIONBASE_PLUGIN=\"${CUARM_ROBOTICS_DIR}/src/3rd_party/Eigen_QuaternionBase_Plugin.hpp\""
        "EIGEN_MATRIX_PLUGIN=\"${CUARM_ROBOTICS_DIR}/src/3rd_party/Matrix_initializer_list.h\""
    )
    target_link_libraries(cuarm_robotics::robotics INTERFACE 
        "${UTILS_FILE}"
        "${UDP_FILS}"
        "${TCP_FILE}"
        yaml-cpp::yaml-cpp
        OpenSSL::Crypto
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        pinocchio::pinocchio
    )
endif()

###################################Generate Library######################################
add_library(spark2_sdk SHARED
    ${PROJECT_SOURCE_DIR}/src/spark2.cpp
    ${PROJECT_SOURCE_DIR}/src/configurator.cpp
    ${PROJECT_SOURCE_DIR}/src/kinematics.cpp
)

set_target_properties(spark2_sdk PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

target_include_directories(spark2_sdk PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
)

# Workspace root so includes like "robot_platform_utils/cpp/include/..." resolve;
# also expose cuarm_rt_control for robotics headers used by kinematics.
target_include_directories(spark2_sdk PRIVATE
    ${ROBOT_PLATFORM_UTILS_DIR}
    ${RT_CONTROL_DIR}
)

target_link_libraries(spark2_sdk PRIVATE
    cuarm_robotics::robotics
)

if(WIN32)
    target_link_libraries(spark2_sdk PRIVATE ws2_32 crypt32)
endif()

###################################Generate Python Library######################################
if(BUILD_PYTHON_LIB)
    message(STATUS "---------------Generate Python Library----------------------")
    # --- Use python3 in the same environment ---
    set(Python3_FIND_STRATEGY LOCATION)
    set(Python3_FIND_REGISTRY NEVER) 
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

    # Locate pybind11 directly using the discovered Python executable
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    cmake_path(CONVERT "${pybind11_DIR}" TO_CMAKE_PATH_LIST pybind11_DIR)
    find_package(pybind11 REQUIRED CONFIG PATHS ${pybind11_DIR})

    pybind11_add_module(_spark2_sdk_cpp
        ${PROJECT_SOURCE_DIR}/src/pybind11/bindings.cpp
    )

    target_link_libraries(_spark2_sdk_cpp PRIVATE spark2_sdk)
    target_include_directories(_spark2_sdk_cpp PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )
    set_target_properties(_spark2_sdk_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PYTHON_LIB_PATH}"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PYTHON_LIB_PATH}"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${PYTHON_LIB_PATH}"
        LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${PYTHON_LIB_PATH}"
        LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PYTHON_LIB_PATH}"
        # Find sibling libspark2_sdk.so / spark2_sdk.dll without LD_LIBRARY_PATH.
        BUILD_RPATH "\$ORIGIN"
        INSTALL_RPATH "\$ORIGIN"
        BUILD_WITH_INSTALL_RPATH TRUE
        SKIP_BUILD_RPATH FALSE
    )

    add_custom_command(TARGET _spark2_sdk_cpp POST_BUILD
        # Copies spark2_sdk (.dll on Win, .so on Linux) next to your python module (.pyd/.so)
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:spark2_sdk>"          
            "$<TARGET_FILE_DIR:_spark2_sdk_cpp>" 
        COMMENT "[PYTHON BINDINGS] Deploying runtime shared library engine directly into Python tree..."
    )
endif()

###################################Generate Executable######################################
file(GLOB EXAMPLE_SRCS "${PROJECT_SOURCE_DIR}/examples/*.cpp")
foreach(EXAMPLE_SRC ${EXAMPLE_SRCS})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SRC} NAME_WE)
    set(EXECUTABLE_NAME ${EXAMPLE_NAME})
    message(STATUS "Add example executable: ${EXECUTABLE_NAME}")

    add_executable(${EXECUTABLE_NAME}
        ${EXAMPLE_SRC}
    )
    target_link_libraries(${EXECUTABLE_NAME}
        spark2_sdk
    )
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC 
        CONFIG_PREFIX_PATH="${CONFIG_PATH}"
    )

    if(WIN32)
        add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_RUNTIME_DLLS:${EXECUTABLE_NAME}>"          
                "$<TARGET_FILE_DIR:${EXECUTABLE_NAME}>" 
            COMMAND_EXPAND_LISTS
            COMMENT "Build-time: Deploying Conda DLLs for ${EXECUTABLE_NAME}..."
        )
    endif()
endforeach()


##########################################Install############################################
set(DIST_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/../spark2_cpp_dist")
set(INSTALL_BIN_DIR "${DIST_OUTPUT_DIR}/${CPP_INSTALL_PATH}")

if(WIN32)
    install(TARGETS spark2_sdk
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
        ARCHIVE DESTINATION "${INSTALL_BIN_DIR}"
    )    
else()
    install(TARGETS spark2_sdk
        LIBRARY DESTINATION "${INSTALL_BIN_DIR}"
    )
endif()

# Public SDK API headers shipped in spark2_cpp_dist.
install(FILES
    "${PROJECT_SOURCE_DIR}/include/spark2.h"
    "${PROJECT_SOURCE_DIR}/include/types.h"
    "${PROJECT_SOURCE_DIR}/include/kinematics.h"
    "${PROJECT_SOURCE_DIR}/include/configurator.h"
    DESTINATION "${DIST_OUTPUT_DIR}/include"
)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/examples/"
    DESTINATION "${DIST_OUTPUT_DIR}/examples"
    FILES_MATCHING PATTERN "*.cpp"
)

install(DIRECTORY "${CONFIG_PATH}/"
    DESTINATION "${PROJECT_SOURCE_DIR}/../configuration"
)

########################################## Auto-Deploy Sync ############################################
add_custom_command(TARGET spark2_sdk POST_BUILD
    COMMAND ${CMAKE_COMMAND} --install "${CMAKE_BINARY_DIR}" --config $<CONFIG>
    COMMENT "[AUTO-DEPLOY] Syncing binaries to python/ modules and spark2_sdk_dist/ packages..."
)
