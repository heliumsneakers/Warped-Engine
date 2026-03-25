cmake_minimum_required(VERSION 3.15)
project(Warped)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default build type if none provided
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

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
# Map compiler (offline CLI) — built standalone, no sokol/Jolt.
#   cmake -B build_mc -DBUILD_MAP_COMPILER=ON && cmake --build build_mc
#   ./build_mc/bin/compile_map assets/maps/test.map test
# ----------------------------
option(BUILD_MAP_COMPILER "Build only the compile_map CLI tool" OFF)

if(BUILD_MAP_COMPILER)
    add_executable(compile_map
        src/compiler/compile_map.cpp
        src/compiler/lightmap.cpp
        src/utils/map_parser.cpp
        src/utils/parameters.cpp
        src/physx/collision_data.cpp
    )
    target_compile_definitions(compile_map PRIVATE WARPED_COMPILER_BUILD)
    target_include_directories(compile_map PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/lib/stb
    )
    target_link_libraries(compile_map PRIVATE ${WARPED_LIBCXX_EXTRA_LIBS})
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

target_include_directories(WarpedGame PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/lib/sokol
    ${PROJECT_SOURCE_DIR}/lib/sokol/util
    ${PROJECT_SOURCE_DIR}/lib/stb
)

# ----------------------------
# Platform libs (accumulate, then link once)
# ----------------------------
set(PLATFORM_LIBS)
if (APPLE)
    enable_language(OBJC)
    find_library(COCOA_LIBRARY          Cocoa)
    find_library(QUARTZCORE_LIBRARY     QuartzCore)
    find_library(UNIFORM_TYPE_IDENT_LIB UniformTypeIdentifiers)
    find_library(IOKIT_LIBRARY          IOKit)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(COREGRAPHICS_LIBRARY   CoreGraphics)
    find_library(COREVIDEO_LIBRARY      CoreVideo)
    find_library(OPENGL_LIBRARY         OpenGL)
    list(APPEND PLATFORM_LIBS
        ${COCOA_LIBRARY}
        ${QUARTZCORE_LIBRARY}
        ${UNIFORM_TYPE_IDENT_LIB}
        ${IOKIT_LIBRARY}
        ${COREFOUNDATION_LIBRARY}
        ${COREGRAPHICS_LIBRARY}
        ${COREVIDEO_LIBRARY}
        ${OPENGL_LIBRARY}
    )
elseif (UNIX)
    list(APPEND PLATFORM_LIBS X11 Xi Xcursor GL dl pthread m)
elseif (WIN32)
    list(APPEND PLATFORM_LIBS opengl32 gdi32)
endif()

# ----------------------------
# Single linking call (keyword signature)
# ----------------------------
target_link_libraries(WarpedGame PRIVATE
    Jolt
    ${PLATFORM_LIBS}
    ${WARPED_LIBCXX_EXTRA_LIBS}
)
