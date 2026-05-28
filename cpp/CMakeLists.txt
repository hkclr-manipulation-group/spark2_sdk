cmake_minimum_required(VERSION 3.20)
project(lite_arm_sdk)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

###################################OS Settings###########################################
add_definitions(-DARCHITECTURE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(STATUS "Building for Linux-ARM architecture")
    set(ARCHITECTURE "arm64")
    add_definitions(-DARCHITECTURE_ARM64=1)
else() 
    message(STATUS "Building for Linux-x86 architecture")
    set(ARCHITECTURE "x86")
    add_definitions(-DARCHITECTURE_X86=1)
endif()

###################################Add subdirectory#################################################
cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR)
cmake_path(GET PARENT_DIR PARENT_PATH GRANDPARENT_DIR )
set (CONFIG_PREFIX_PATH ${GRANDPARENT_DIR}/cuarm_configuration)

#Find the pre-compiled robotics library first
set(LIB_SEARCH_DIR ${GRANDPARENT_DIR}/cuarm_rt_control/build)
find_library(ROBOTICS_LIB_FILE
    NAMES robotics_lib
    PATHS ${LIB_SEARCH_DIR}/cuarm_robotics
    NO_DEFAULT_PATH
)

# If NOT found, fallback to adding the source directory
if(NOT ROBOTICS_LIB_FILE)
    message(STATUS "Compiled library not found in ${LIB_SEARCH_DIR}. Falling back to source compilation...")
    
    set(ROBOT_PLATFORM_UTILS_DIR "${GRANDPARENT_DIR}" CACHE PATH "Path to the utils repository")
    set(ROBOT_PLATFORM_UTILS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Path to the utils repository binary directory")
    add_subdirectory(${GRANDPARENT_DIR}/cuarm_rt_control/cuarm_robotics ${CMAKE_BINARY_DIR}/cuarm_robotics)
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
    add_library(cuarm_robotics::robotics STATIC IMPORTED GLOBAL)
    set_target_properties(cuarm_robotics::robotics PROPERTIES
        IMPORTED_LOCATION "${ROBOTICS_LIB_FILE}"
        INTERFACE_INCLUDE_DIRECTORIES "${GRANDPARENT_DIR}/cuarm_rt_control;${GRANDPARENT_DIR}/cuarm_rt_control/cuarm_robotics/src;${GRANDPARENT_DIR}/robot_platform_utils/cpp/include;${EIGEN3_INCLUDE_DIRS};${Boost_INCLUDE_DIRS};${PINOCCHIO_INCLUDE_DIRS}"
    )

    set_property(TARGET cuarm_robotics::robotics PROPERTY INTERFACE_COMPILE_DEFINITIONS
        "EIGEN_QUATERNIONBASE_PLUGIN=\"${GRANDPARENT_DIR}/cuarm_rt_control/cuarm_robotics/src/3rd_party/Eigen_QuaternionBase_Plugin.hpp\""
        "EIGEN_MATRIX_PLUGIN=\"${GRANDPARENT_DIR}/cuarm_rt_control/cuarm_robotics/src/3rd_party/Matrix_initializer_list.h\""
    )
    target_link_libraries(cuarm_robotics::robotics INTERFACE 
        "${UTILS_FILE}"
        "${UDP_FILS}"
        "${TCP_FILE}"
        yaml-cpp::yaml-cpp
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PINOCCHIO_LIBRARIES}
    )
endif()

###################################Generate Executable######################################
add_library(lite_arm_sdk
    ${PROJECT_SOURCE_DIR}/src/spark.cpp
    ${PROJECT_SOURCE_DIR}/src/configurator.cpp
    ${PROJECT_SOURCE_DIR}/src/kinematics.cpp
)

target_include_directories(lite_arm_sdk PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${GRANDPARENT_DIR}
    ${GRANDPARENT_DIR}/cuarm_rt_control
)

target_link_libraries(lite_arm_sdk
    pthread
    cuarm_robotics::robotics
)

file(GLOB EXAMPLE_SRCS "${PROJECT_SOURCE_DIR}/examples/*.cpp")
foreach(EXAMPLE_SRC ${EXAMPLE_SRCS})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SRC} NAME_WE)
    set(EXECUTABLE_NAME ${EXAMPLE_NAME})
    message(STATUS "Add example executable: ${EXECUTABLE_NAME}")

    add_executable(${EXECUTABLE_NAME}
        ${EXAMPLE_SRC}
    )

    target_link_libraries(${EXECUTABLE_NAME}
        lite_arm_sdk
    )

    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC 
        CONFIG_PREFIX_PATH="${CONFIG_PREFIX_PATH}/arm_v1"
    )
endforeach()
