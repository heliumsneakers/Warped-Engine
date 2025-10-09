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
# Sanitizers (Clang or GCC)
# ------------------------------------------------------------------
if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

# Output dirs
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include dirs
include_directories(${PROJECT_SOURCE_DIR}/src/include)

# ----------------------------
# raylib
# ----------------------------
set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET)
if (NOT raylib_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED)
    set(FETCHCONTENT_QUIET NO)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(raylib)
  endif()
endif()

# ----------------------------
# raygui
# ----------------------------
set(RAYGUI_SRC ${PROJECT_SOURCE_DIR}/lib/raygui/src)
include_directories(${RAYGUI_SRC})

# ----------------------------
# JoltPhysics
# ----------------------------
set(BUILD_TESTS OFF CACHE BOOL "Disable building tests" FORCE)
set(BUILD_VIEWER OFF CACHE BOOL "Disable building viewer" FORCE)
add_subdirectory(lib/JoltPhysics/Build)

# ----------------------------
# Sources
# ----------------------------
file(GLOB_RECURSE CUSTOM_SOURCES
    src/*.cpp
    src/*.c
    lib/raygui/src/*.c
    lib/raygui/src/*.cpp
)

add_executable(WarpedGame ${CUSTOM_SOURCES})

target_include_directories(WarpedGame PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${RAYGUI_SRC}
)

# ----------------------------
# Platform libs (accumulate, then link once)
# ----------------------------
set(PLATFORM_LIBS)
if (APPLE)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(COREGRAPHICS_LIBRARY CoreGraphics)
    find_library(COREVIDEO_LIBRARY CoreVideo)
    find_library(OPENGL_LIBRARY OpenGL)
    list(APPEND PLATFORM_LIBS
        ${COCOA_LIBRARY}
        ${IOKIT_LIBRARY}
        ${COREFOUNDATION_LIBRARY}
        ${COREGRAPHICS_LIBRARY}
        ${COREVIDEO_LIBRARY}
        ${OPENGL_LIBRARY}
    )
elseif (UNIX)
    list(APPEND PLATFORM_LIBS GL pthread dl m)
elseif (WIN32)
    list(APPEND PLATFORM_LIBS opengl32)
endif()

# ----------------------------
# Single linking call (keyword signature)
# ----------------------------
target_link_libraries(WarpedGame PRIVATE
    Jolt
    raylib
    ${PLATFORM_LIBS}
    ${WARPED_LIBCXX_EXTRA_LIBS}
)

# ----------------------------
# Defines
# ----------------------------
target_compile_definitions(WarpedGame PRIVATE RAYGUI_IMPLEMENTATION)
