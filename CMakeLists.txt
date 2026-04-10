cmake_minimum_required(VERSION 3.15)
project(Warped)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default build type if none provided
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# ------------------------------------------------------------------
# Renderer backend selection
# ------------------------------------------------------------------
if(NOT DEFINED WARPED_SOKOL_BACKEND OR WARPED_SOKOL_BACKEND STREQUAL "")
    if(APPLE)
        set(WARPED_SOKOL_BACKEND "METAL" CACHE STRING "Sokol renderer backend" FORCE)
    elseif(WIN32)
        set(WARPED_SOKOL_BACKEND "D3D11" CACHE STRING "Sokol renderer backend" FORCE)
    elseif(UNIX)
        set(WARPED_SOKOL_BACKEND "GLCORE" CACHE STRING "Sokol renderer backend" FORCE)
    else()
        message(FATAL_ERROR "Unsupported platform for Sokol backend selection: ${CMAKE_SYSTEM_NAME}")
    endif()
endif()

set_property(CACHE WARPED_SOKOL_BACKEND PROPERTY STRINGS GLCORE METAL D3D11)
string(TOUPPER "${WARPED_SOKOL_BACKEND}" WARPED_SOKOL_BACKEND)

if(NOT WARPED_SOKOL_BACKEND MATCHES "^(GLCORE|METAL|D3D11)$")
    message(FATAL_ERROR "Invalid WARPED_SOKOL_BACKEND='${WARPED_SOKOL_BACKEND}'. Expected GLCORE, METAL or D3D11.")
endif()

if(APPLE AND NOT WARPED_SOKOL_BACKEND STREQUAL "METAL")
    message(FATAL_ERROR "macOS builds must use WARPED_SOKOL_BACKEND=METAL.")
elseif(WIN32 AND NOT WARPED_SOKOL_BACKEND STREQUAL "D3D11")
    message(FATAL_ERROR "Windows builds must use WARPED_SOKOL_BACKEND=D3D11.")
elseif(UNIX AND NOT APPLE AND NOT WARPED_SOKOL_BACKEND STREQUAL "GLCORE")
    message(FATAL_ERROR "Linux builds must use WARPED_SOKOL_BACKEND=GLCORE.")
endif()

message(STATUS "Warped renderer backend: ${WARPED_SOKOL_BACKEND}")

# ------------------------------------------------------------------
# Enforce Clang + libc++ on macOS and Linux
# ------------------------------------------------------------------
if(APPLE OR (UNIX AND NOT WIN32))
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR
            "This project requires Clang/Clang++ on ${CMAKE_SYSTEM_NAME}. "
            "Re-configure with: -D CMAKE_C_COMPILER=clang -D CMAKE_CXX_COMPILER=clang++")
    endif()

    add_compile_options(-stdlib=libc++)
    add_link_options(-stdlib=libc++)

    # On Linux, link libc++ runtime explicitly
    if(UNIX AND NOT APPLE)
        set(WARPED_LIBCXX_EXTRA_LIBS c++ c++abi)
    endif()
endif()

# ------------------------------------------------------------------
# Sanitizers (Clang or GCC) — off by default; ASan's shadow-memory
# bookkeeping and quarantine sweeps cause runtime hitching, so only
# enable on demand:  cmake -D WARPED_ENABLE_ASAN=ON ..
# ------------------------------------------------------------------
option(WARPED_ENABLE_ASAN "Build with AddressSanitizer" OFF)
if (WARPED_ENABLE_ASAN AND (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang")))
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

# Output dirs
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ----------------------------
# Map compiler (offline CLI) — built standalone, no app/runtime renderer/Jolt.
# Uses sokol_gfx directly for the compute lightmap baker.
#   cmake -B build_mc -DBUILD_MAP_COMPILER=ON && cmake --build build_mc
#   ./build_mc/bin/compile_map assets/maps/test.map test
# ----------------------------
option(BUILD_MAP_COMPILER "Build only the compile_map CLI tool" OFF)

if(BUILD_MAP_COMPILER)
    if(APPLE)
        enable_language(OBJC)
        enable_language(OBJCXX)
        set_source_files_properties(src/compiler/sokol_compute_impl.c
            PROPERTIES
                LANGUAGE OBJC
                COMPILE_FLAGS "-fobjc-arc -x objective-c")
    endif()

    add_executable(compile_map
        src/compiler/compile_map.cpp
        src/compiler/lightmap.cpp
        src/compiler/lightmap_trace.cpp
        src/compiler/structural_bsp.cpp
        src/compiler/lightmap_compute.cpp
        src/compiler/sokol_compute_impl.c
        src/utils/asset_pack.cpp
        src/utils/map_parser.cpp
        src/utils/parameters.cpp
        src/physx/collision_data.cpp
    )
    if(APPLE)
        target_sources(compile_map PRIVATE src/compiler/lightmap_compute_metal.mm)
    endif()
    target_compile_definitions(compile_map PRIVATE WARPED_COMPILER_BUILD)
    if(WARPED_SOKOL_BACKEND STREQUAL "METAL")
        target_compile_definitions(compile_map PRIVATE WARPED_SOKOL_BACKEND_METAL)
    elseif(WARPED_SOKOL_BACKEND STREQUAL "D3D11")
        target_compile_definitions(compile_map PRIVATE WARPED_SOKOL_BACKEND_D3D11)
    elseif(WARPED_SOKOL_BACKEND STREQUAL "GLCORE")
        target_compile_definitions(compile_map PRIVATE WARPED_SOKOL_BACKEND_GLCORE)
    endif()
    target_include_directories(compile_map PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/lib/sokol
        ${PROJECT_SOURCE_DIR}/lib/stb
        ${PROJECT_SOURCE_DIR}/lib/rres/src
    )

    set(COMPILER_PLATFORM_LIBS)
    if(APPLE)
        find_library(FOUNDATION_LIBRARY     Foundation)
        find_library(METAL_LIBRARY          Metal)
        find_library(QUARTZCORE_LIBRARY     QuartzCore)
        list(APPEND COMPILER_PLATFORM_LIBS
            ${FOUNDATION_LIBRARY}
            ${METAL_LIBRARY}
            ${QUARTZCORE_LIBRARY}
        )
    elseif(WIN32)
        list(APPEND COMPILER_PLATFORM_LIBS d3d11 dxgi dxguid)
    elseif(UNIX)
        list(APPEND COMPILER_PLATFORM_LIBS X11 GL dl pthread m)
    endif()

    target_link_libraries(compile_map PRIVATE
        ${COMPILER_PLATFORM_LIBS}
        ${WARPED_LIBCXX_EXTRA_LIBS}
    )
    return()   # don't configure the engine target
endif()

# ----------------------------
# JoltPhysics
# ----------------------------
set(BUILD_TESTS OFF CACHE BOOL "Disable building tests" FORCE)
set(BUILD_VIEWER OFF CACHE BOOL "Disable building viewer" FORCE)
add_subdirectory(lib/JoltPhysics/Build)

# ----------------------------
# Sources
# ----------------------------
# C++ sources - everything under src/ except the sokol impl TU which is C/ObjC.
file(GLOB_RECURSE WARPED_CXX_SOURCES
    CONFIGURE_DEPENDS
    src/*.cpp
)
list(FILTER WARPED_CXX_SOURCES EXCLUDE REGEX "platform/sokol_impl")
list(FILTER WARPED_CXX_SOURCES EXCLUDE REGEX "compiler/")

# Sokol implementation TU - compiled as Obj-C on Apple, plain C elsewhere.
set(SOKOL_IMPL_SRC src/platform/sokol_impl.c)
if (APPLE)
    set_source_files_properties(${SOKOL_IMPL_SRC}
        PROPERTIES
            LANGUAGE OBJC
            COMPILE_FLAGS "-fobjc-arc -x objective-c")
endif()

add_executable(WarpedGame
    ${WARPED_CXX_SOURCES}
    ${SOKOL_IMPL_SRC}
)

if(WARPED_SOKOL_BACKEND STREQUAL "METAL")
    target_compile_definitions(WarpedGame PRIVATE WARPED_SOKOL_BACKEND_METAL)
elseif(WARPED_SOKOL_BACKEND STREQUAL "D3D11")
    target_compile_definitions(WarpedGame PRIVATE WARPED_SOKOL_BACKEND_D3D11)
elseif(WARPED_SOKOL_BACKEND STREQUAL "GLCORE")
    target_compile_definitions(WarpedGame PRIVATE WARPED_SOKOL_BACKEND_GLCORE)
endif()

target_include_directories(WarpedGame PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/lib/clay
    ${PROJECT_SOURCE_DIR}/lib/sokol
    ${PROJECT_SOURCE_DIR}/lib/sokol/util
    ${PROJECT_SOURCE_DIR}/lib/sokol/tests/ext
    ${PROJECT_SOURCE_DIR}/lib/stb
    ${PROJECT_SOURCE_DIR}/lib/rres/src
)

# ----------------------------
# Platform libs (accumulate, then link once)
# ----------------------------
set(PLATFORM_LIBS)
if (APPLE)
    enable_language(OBJC)
    find_library(COCOA_LIBRARY          Cocoa)
    find_library(METAL_LIBRARY          Metal)
    find_library(QUARTZCORE_LIBRARY     QuartzCore)
    find_library(UNIFORM_TYPE_IDENT_LIB UniformTypeIdentifiers)
    find_library(IOKIT_LIBRARY          IOKit)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(COREGRAPHICS_LIBRARY   CoreGraphics)
    find_library(COREVIDEO_LIBRARY      CoreVideo)
    list(APPEND PLATFORM_LIBS
        ${COCOA_LIBRARY}
        ${METAL_LIBRARY}
        ${QUARTZCORE_LIBRARY}
        ${UNIFORM_TYPE_IDENT_LIB}
        ${IOKIT_LIBRARY}
        ${COREFOUNDATION_LIBRARY}
        ${COREGRAPHICS_LIBRARY}
        ${COREVIDEO_LIBRARY}
    )
elseif (UNIX)
    list(APPEND PLATFORM_LIBS X11 Xi Xcursor GL dl pthread m)
elseif (WIN32)
    list(APPEND PLATFORM_LIBS d3d11 dxgi gdi32)
endif()

# ----------------------------
# Single linking call (keyword signature)
# ----------------------------
target_link_libraries(WarpedGame PRIVATE
    Jolt
    ${PLATFORM_LIBS}
    ${WARPED_LIBCXX_EXTRA_LIBS}
)
