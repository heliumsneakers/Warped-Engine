# Warped Engine
Warped Engine is heavily inspired by Valve and Id Software's workflows. It is a project aiming to build a lightweight structure with minimal dependencies that can run on multiple platforms.

Id Tech 2 and GoldSrc are awesome pieces of software and I'd like to build something similar but in a more modern context.

The initial spark for this project was from a 72 hour game jam, where I wrote the beginnings of the engine. This initial version included: map parser (without textures), player movement, and UI.

https://heliumsneakers.itch.io/mini-quake-demo

The original project was compiled with emscripten for web, but I've reverted to local builds to accommodate the physics library, the web build version of this project is split into a separate project folder locally. I plan on uploading it in the future.

 The engine is structured around **4 libraries, and 1 external map editor**.
- **Sokol**: Graphics library used for rendering, I plan to add full compatibility with Metal, DX11, and Vulkan.
- **Jolt Physics**: The physics library used in the project.
- **Embree**: Used for accelerating the CPU and Compute shader lightmap bake using SIMD and BVH trees.
- **RRes**: Binary format used for packing assets for individual maps. 
- **Trench Broom**: Quake map editor for creating convex geometry (maps) and entities. The format used in this project is Valve 220.

## Engine Structure

The engine specifies entity definitions and configurations in the warped.fgd file (Forge Game Data), this file is used by trenchbroom during map editing and contains all of our game information.

When building a finalized map there is an included map compiler that creates a .bsp file compiled with a pre-processing step for baked lighting and shadow maps.

All geometry from the map file is parsed and uploaded to the GPU for rendering and sent to the physics system to build the collision mesh.

> [!NOTE]
> I'd like to thank **Stefan Hajnoczi** for his work in the 2001 paper ".MAP files, file format description, algorithms, and code" which can be found here:
>
> https://github.com/stefanha/map-files/blob/master/MAPFiles.pdf

## Prerequisites

- CMake (version 3.10 or higher)
- A C++ compiler (e.g., GCC, Clang, MSVC)

## Building the Project
1. Clone
``` bash
git clone https://github.com/heliumsneakers/Warped-Engine.git
cd Warped
```

2. Submodules
``` bash
git submodule init
git submodule update
```

**Before building the project ensure that Jolt is built for your platform:**
```
https://github.com/jrouwe/JoltPhysics
```

3. Building the engine
```bash 
mkdir build
cd build
cmake .. 
make
```

4. Building the .bsp compiler
```bash
cmake -B build_mc -DBUILD_MAP_COMPILER=ON && cmake --build build_mc
```

## Map compiler usage

When using the map compiler you should follow this structure:

```bash
./compile_map <PATH_TO_MAP_FILE>.map <COMPILED_MAP_DESTINATION>.bsp  
```

For example:

On Linux and MacOS
``` bash
./compile_map ../../assets/maps/example.map ../../assets/maps/example.bsp
```

## Lightmap regression harness

The CPU reference baker can be regression-tested against a small fixture set:

```bash
python3 tools/lightmap_regression.py record
python3 tools/lightmap_regression.py verify
```

By default this bakes:

- `assets/maps/reg_direct_point.map`
- `assets/maps/reg_extra.map`
- `assets/maps/reg_range.map`
- `assets/maps/reg_maxlight.map`
- `assets/maps/reg_gamma.map`
- `assets/maps/reg_soften.map`
- `assets/maps/reg_surface_direct.map`
- `assets/maps/reg_surface_bounce.map`
- `assets/maps/reg_sun_sky.map`

through `build_mc/bin/compile_map`, forcing the CPU path with
`WARPED_LIGHTMAP_FORCE_CPU=1`, then compares the baked `LUMP_LIGHTMAP` pages
against `tools/lightmap_regression_baseline.json`.
