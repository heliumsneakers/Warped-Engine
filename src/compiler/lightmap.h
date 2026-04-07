// lightmap.h  —  offline lightmap baker for compile_map.
#pragma once
#include "../utils/map_parser.h"
#include <vector>
#include <cstdint>

struct LightmapPage {
    int                  width  = 0;
    int                  height = 0;
    std::vector<float>   pixels; // linear RGBA, width*height*4
};

// Renderable surface patch used for lightmapped geometry emission. A single
// source polygon may produce multiple patches when it spans multiple lightmap
// pages, but the source polygon list remains the authoritative geometry input.
struct LightmapPatch {
    MapPolygon            poly;
    std::vector<Vector2>  uv; // parallel to poly.verts
    uint32_t              page = 0;
    uint32_t              sourcePolyIndex = 0;
};

struct LightmapAtlas {
    std::vector<LightmapPage> pages;
    std::vector<LightmapPatch> patches;
};

// Allocate per-face UV rects, split oversized faces into lightmap-only bake
// patches, pack them into multiple lightmap pages and bake Lambert lighting
// from the supplied point lights. `polys` remains the authoritative source
// polygon list for world geometry/collision; any patch subdivision returned in
// LightmapAtlas::patches is only for lightmapped render emission.
LightmapAtlas BakeLightmap(const std::vector<MapPolygon>& polys,
                           const std::vector<MapPolygon>& occluderPolys,
                           const std::vector<PointLight>& lights,
                           const std::vector<SurfaceLightTemplate>& surfaceLights,
                           const LightBakeSettings& settings);
