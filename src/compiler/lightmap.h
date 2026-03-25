// lightmap.h  —  offline lightmap baker for compile_map.
#pragma once
#include "../utils/map_parser.h"
#include <vector>
#include <cstdint>

// One entry per input polygon: the lightmap UVs for each of its vertices,
// in the same order as MapPolygon::verts.  Normalised to [0..1] in atlas space.
struct PolyLightmapUV {
    std::vector<Vector2> uv;
};

struct LightmapAtlas {
    int width  = 0;
    int height = 0;
    std::vector<uint8_t> pixels;        // RGBA8, width*height*4
    std::vector<PolyLightmapUV> polyUV; // parallel to input polygon array
};

// Allocate per-face UV rects, pack into an atlas and bake Lambert lighting
// from the supplied point lights.  `polys` must already be in world-space
// GL coords (i.e. output of BuildMapPolygons).
LightmapAtlas BakeLightmap(const std::vector<MapPolygon>& polys,
                           const std::vector<PointLight>& lights);
