#pragma once

#include "../math/wmath.h"
#include "../utils/map_parser.h"

#include <cstdint>
#include <string>
#include <vector>

static constexpr int LIGHTMAP_COMPUTE_MAX_POLY_VERTS = 32;
static constexpr int LIGHTMAP_COMPUTE_MAX_PHONG_NEIGHBORS = 32;

struct LightmapComputeRepairSourceNeighbor {
    int sourcePolyIndex = -1;
    int edgeIndex = -1;
    float _pad[2] = {};
};

struct LightmapComputeRepairSourcePoly {
    Vector3 planePoint{};
    float _pad0 = 0.0f;
    Vector3 axisU{};
    float _pad1 = 0.0f;
    Vector3 axisV{};
    float _pad2 = 0.0f;
    Vector3 normal{};
    int polyCount = 0;
    int firstNeighbor = 0;
    int neighborCount = 0;
    float _pad3[2] = {};
    float polyVerts[LIGHTMAP_COMPUTE_MAX_POLY_VERTS][4] = {};
};

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
    int sourcePolyIndex = -1;
    float polyVerts[LIGHTMAP_COMPUTE_MAX_POLY_VERTS][4] = {};
    Vector3 phongBaseNormal{};
    float phongBaseAreaWeight = 1.0f;
    int phongNeighborCount = 0;
    float _pad0[2] = {};
    float phongNeighborEdgeA[LIGHTMAP_COMPUTE_MAX_PHONG_NEIGHBORS][4] = {};
    float phongNeighborEdgeB[LIGHTMAP_COMPUTE_MAX_PHONG_NEIGHBORS][4] = {};
    float phongNeighborNormalWeight[LIGHTMAP_COMPUTE_MAX_PHONG_NEIGHBORS][4] = {};
};

struct LightmapComputeOccluderTri {
    Vector3 a{};
    Vector3 b{};
    Vector3 c{};
    AABB bounds{};
    int occluderGroup = -1;
    int sourcePolyIndex = -1;
};

struct LightmapComputeBrushSolid {
    int firstPlane = 0;
    int planeCount = 0;
    float _pad[2] = {};
};

struct LightmapComputeSolidPlane {
    Vector3 point{};
    float _pad0 = 0.0f;
    Vector3 normal{};
    float _pad1 = 0.0f;
};

bool BakeLightmapCompute(const std::vector<LightmapComputeFaceRect>& rects,
                         const std::vector<LightmapComputeOccluderTri>& occluders,
                         const std::vector<LightmapComputeBrushSolid>& brushSolids,
                         const std::vector<LightmapComputeSolidPlane>& solidPlanes,
                         const std::vector<LightmapComputeRepairSourcePoly>& repairSourcePolys,
                         const std::vector<LightmapComputeRepairSourceNeighbor>& repairSourceNeighbors,
                         const std::vector<PointLight>& lights,
                         const LightBakeSettings& settings,
                         float skyTraceDistance,
                         int atlasWidth,
                         int atlasHeight,
                         std::vector<float>& outPixels,
                         std::string* error);
