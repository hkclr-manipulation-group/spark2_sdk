cmake_minimum_required(VERSION 3.20)
project(spark2_cpp_dist)

# Enforce modern C++ parameters matching your core runtime library engine
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################### Dynamic Platform Mapping ###################################
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
message(STATUS "[SDK CLIENT] Detected Target Architecture Platform Profile: ${PLATFORM_NAME}")

# Resolve the absolute folder path containing your pre-compiled binaries matching your new layout
set(SDK_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/${PLATFORM_NAME}/${TOOLCHAIN_VARIANT}")

################################### Register Imported Target ###################################
# Declare your pre-compiled library target interface abstraction layer
add_library(spark2_sdk SHARED IMPORTED GLOBAL)

if(WIN32)
    set_target_properties(spark2_sdk PROPERTIES
        # Windows requires mapping the compile-time import link stub (.lib)
        IMPORTED_IMPLIB "${SDK_LIB_DIR}/spark2_sdk.lib"
        
        # Windows requires mapping the physical runtime shared engine (.dll)
        IMPORTED_LOCATION "${SDK_LIB_DIR}/spark2_sdk.dll"
        
        # Pull definitions out of the centralized distributed include folder
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include"
    )
else()
    set_target_properties(spark2_sdk PROPERTIES
        # Linux requires mapping the standalone dynamic shared object (.so) directly
        IMPORTED_LOCATION "${SDK_LIB_DIR}/libspark2_sdk.so"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include"
    )
endif()

################################### Generate User Executables ###################################
# Automatically scan, map, and generate build targets for all distributed example templates
file(GLOB USER_EXAMPLES "${CMAKE_CURRENT_LIST_DIR}/examples/*.cpp")

cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR)
set (CONFIG_PREFIX_PATH ${PARENT_DIR}/configuration)

foreach(EXAMPLE_SRC ${USER_EXAMPLES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SRC} NAME_WE)
    message(STATUS "[SDK CLIENT] Configuring distributed example compilation target: ${EXAMPLE_NAME}")

    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SRC})
    target_link_libraries(${EXAMPLE_NAME} PRIVATE spark2_sdk)
    target_compile_definitions(${EXAMPLE_NAME} PUBLIC 
        CONFIG_PREFIX_PATH="${CONFIG_PREFIX_PATH}"
    )

    # Windows Runtime Automated Task: Copy spark2_sdk.dll straight next to the user's compiled .exe
    if(WIN32)
        add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:spark2_sdk>"
                "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>"
            COMMENT "[DEPLOY] Copying spark2_sdk.dll into target executable execution directory..."
        )
    endif()
endforeach()
