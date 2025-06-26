# Warped Engine
Warped Engine is heavily inspired by Valve and Id Software's workflows. It is a project aiming to build a lightweight structure with minimal dependencies that can run on multiple platforms.

Id Tech 2 and GoldSrc are awesome pieces of software and I'd like to build something similar but in a more modern context.

The initial spark for this project was from a 72 hour game jam, where I wrote the beginnings of the engine. This initial version included: map parser (without textures), player movement, and UI.

https://heliumsneakers.itch.io/mini-quake-demo

The original project was compiled with emscripten for web, but I've reverted to local builds to accommodate the physics library, the web build version of this project is split into a separate project folder locally. I plan on uploading it in the future.

 The engine is structured around **3 libraries, and 1 external map editor**.
- **Raylib**: Graphics library used for rendering and mathematics.
- **RayGui**: GUI library used for the user interface, and in game hot reloading of changes.
- **Jolt Physics**: The physics library used in the project for collisions and physics interactions, as well as the physics server.
- **Trench Broom**: Quake map editor for creating convex geometry (maps) and entities. The format used in this project is Valve 220.

The engine specifies entity definitions, and configurations in a .fgd file (Forge Game Data), and has the data parsed in engine for runtime assembly of the geometry and entities.

I'd like to thank **Stefan Hajnoczi** for his work in the 2001 paper ".MAP files, file format description, algorithms, and code" which can be found here:

 https://github.com/stefanha/map-files/blob/master/MAPFiles.pdf

The current version of the engine is stripped from the gameplay features of the web build for development purposes, they will be re-implemented over the next few iterations.

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
**NOTE: Before building the project ensure that Jolt and Raylib are built:**
```
https://github.com/jrouwe/JoltPhysics
```
```
https://github.com/raysan5/raylib
```
3. Build
```bash 
mkdir build
cd build
cmake .. 
make
```
