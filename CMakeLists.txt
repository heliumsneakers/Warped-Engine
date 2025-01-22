cmake_minimum_required(VERSION 3.15)
project(Warped)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json (optional, useful for IDEs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# If using GCC or Clang, you can globally add ASan flags.
if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif ()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories for your project
include_directories(${PROJECT_SOURCE_DIR}/src/include)

# ----------------------------
# Manage raylib Dependency
# ----------------------------
set(RAYLIB_VERSION 5.5)  # Specify the desired raylib version

find_package(raylib ${RAYLIB_VERSION} QUIET) # Attempt to find installed raylib

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
    FetchContent_MakeAvailable(raylib)
    # Disable building raylib examples
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  endif()
endif()

# ----------------------------
# Manage raygui Dependency
# ----------------------------
# Assuming raygui is located at lib/raygui and is a single-header library
# If raygui has source files, you can add them similarly
set(RAYGUI_SRC ${PROJECT_SOURCE_DIR}/lib/raygui/src)

# Include raygui headers
include_directories(${RAYGUI_SRC}) # Ensure raygui.h is in the include path

# ----------------------------
# Manage JoltPhysics Dependency
# ----------------------------
# Add JoltPhysics as a subdirectory
# Ensure that JoltPhysics' CMakeLists.txt is configured to exclude tests and viewer
set(BUILD_TESTS OFF CACHE BOOL "Disable building tests" FORCE)
set(BUILD_VIEWER OFF CACHE BOOL "Disable building viewer" FORCE)
add_subdirectory(lib/JoltPhysics/Build)

# ----------------------------
# Collect Your Project's Source Files
# ----------------------------
# It's better to list source files explicitly for better control
# However, for brevity, we'll use GLOB_RECURSE here

file(GLOB_RECURSE CUSTOM_SOURCES
    src/*.cpp
    src/*.c
    lib/raygui/src/*.c
    lib/raygui/src/*.cpp 
)

# ----------------------------
# Create the Executable
# ----------------------------
add_executable(WarpedGame ${CUSTOM_SOURCES})

# ----------------------------
# Include Directories for Dependencies
# ----------------------------
target_include_directories(WarpedGame PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${RAYGUI_SRC}
    # raylib's include directory is handled by find_package or FetchContent
)

# ----------------------------
# Link Libraries
# ----------------------------
target_link_libraries(WarpedGame
    Jolt   # Link against the JoltPhysics library target
    raylib        # Link against raylib; handles dependencies if using find_package or FetchContent
)

# ----------------------------
# Platform-specific Frameworks
# ----------------------------
if (APPLE)
    # Find required macOS frameworks
    find_library(COCOA_LIBRARY Cocoa)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(COREGRAPHICS_LIBRARY CoreGraphics)
    find_library(COREVIDEO_LIBRARY CoreVideo)
    find_library(OPENGL_LIBRARY OpenGL)

    # Link against the frameworks
    target_link_libraries(WarpedGame
        ${COCOA_LIBRARY}
        ${IOKIT_LIBRARY}
        ${COREFOUNDATION_LIBRARY}
        ${COREGRAPHICS_LIBRARY}
        ${COREVIDEO_LIBRARY}
        ${OPENGL_LIBRARY}
    )
elseif (UNIX)
    target_link_libraries(WarpedGame GL pthread dl m)
elseif (WIN32)
    target_link_libraries(WarpedGame opengl32)
endif()

# ----------------------------
# Define Preprocessor Macros
# ----------------------------
target_compile_definitions(WarpedGame PRIVATE RAYGUI_IMPLEMENTATION)
