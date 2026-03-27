#pragma once

#include "../math/wmath.h"
#include "../utils/map_parser.h"

#include <cstdint>
#include <string>
#include <vector>

static constexpr int LIGHTMAP_COMPUTE_MAX_POLY_VERTS = 32;

struct LightmapComputeFaceRect {
    int w = 0;
    int h = 0;
    int x = 0;
    int y = 0;
    float luxelSize = 1.0f;
    float minU = 0.0f;
    float minV = 0.0f;
    Vector3 origin{};
    Vector3 axisU{};
    Vector3 axisV{};
    Vector3 normal{};
    int polyCount = 0;
    float polyVerts[LIGHTMAP_COMPUTE_MAX_POLY_VERTS][4] = {};
};

struct LightmapComputeOccluderTri {
    Vector3 a{};
    Vector3 b{};
    Vector3 c{};
    AABB bounds{};
    int occluderGroup = -1;
};

bool BakeLightmapCompute(const std::vector<LightmapComputeFaceRect>& rects,
                         const std::vector<LightmapComputeOccluderTri>& occluders,
                         const std::vector<PointLight>& lights,
                         int atlasWidth,
                         int atlasHeight,
                         std::vector<uint8_t>& outPixels,
                         std::string* error);
