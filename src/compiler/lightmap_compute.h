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

struct LightmapComputePhongNeighbor {
    int sourcePolyIndex = -1;
    float _pad0[3] = {};
    Vector3 edgeA{};
    float _pad1 = 0.0f;
    Vector3 edgeB{};
    float _pad2 = 0.0f;
    Vector3 normal{};
    float areaWeight = 1.0f;
};

struct LightmapComputePhongSourcePoly {
    Vector3 normal{};
    float areaWeight = 1.0f;
    int enabled = 0;
    int firstNeighbor = 0;
    int neighborCount = 0;
    float _pad0 = 0.0f;
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

struct LightmapComputeSurfaceEmitter {
    PointLight baseLight{};
    Vector3 surfaceNormal{};
    float sampleIntensityScale = 1.0f;
    float attenuationScale = 1.0f;
    float transportScale = 1.0f;
    float hotspotClamp = 16.0f;
    int firstSamplePoint = 0;
    int samplePointCount = 0;
    uint8_t omnidirectional = 0;
    uint8_t rescale = 0;
    uint8_t _pad0[2] = {};
};

struct LightmapComputeSurfaceEmitterSample {
    Vector3 point{};
    float _pad0 = 0.0f;
};

struct LightmapComputeRectSurfaceEmitterRange {
    int firstEmitterIndex = 0;
    int emitterCount = 0;
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
                         const std::vector<LightmapComputePhongSourcePoly>& phongSourcePolys,
                         const std::vector<LightmapComputePhongNeighbor>& phongNeighbors,
                         const std::vector<PointLight>& lights,
                         const std::vector<LightmapComputeSurfaceEmitter>& surfaceEmitters,
                         const std::vector<LightmapComputeSurfaceEmitterSample>& surfaceEmitterSamples,
                         const std::vector<LightmapComputeRectSurfaceEmitterRange>& rectSurfaceEmitterRanges,
                         const std::vector<uint32_t>& rectSurfaceEmitterIndices,
                         const LightBakeSettings& settings,
                         float skyTraceDistance,
                         int atlasWidth,
                         int atlasHeight,
                         bool oversampledOutput,
                         std::vector<float>& outPixels,
                         std::string* error);
