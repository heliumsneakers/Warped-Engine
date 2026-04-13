#pragma once

#include "../math/wmath.h"
#include "lightmap_compute.h"

#include <cstdint>
#include <memory>
#include <vector>

enum LightmapTraceHitKind : uint8_t {
    LIGHTMAP_TRACE_HIT_NONE = 0,
    LIGHTMAP_TRACE_HIT_SOLID,
    LIGHTMAP_TRACE_HIT_SKY,
    LIGHTMAP_TRACE_HIT_FILTERED,
};

enum LightmapTraceTriFlags : uint32_t {
    LIGHTMAP_TRACE_TRI_NONE     = 0u,
    LIGHTMAP_TRACE_TRI_SKY      = 1u << 0,
    LIGHTMAP_TRACE_TRI_FILTERED = 1u << 1,
};

struct LightmapTraceTri {
    Vector3 a{};
    Vector3 b{};
    Vector3 c{};
    AABB bounds{};
    int occluderGroup = -1;
    int sourcePolyIndex = -1;
    uint32_t flags = LIGHTMAP_TRACE_TRI_NONE;
    int materialId = -1;
};

class LightmapTraceAcceleration;

struct LightmapTraceScene {
    std::vector<LightmapTraceTri> tris;
    std::shared_ptr<LightmapTraceAcceleration> acceleration;
};

struct LightmapTraceQuery {
    float minHitT = 0.0f;
    float maxHitT = 0.0f;
    int ignoreOccluderGroup = -1;
    int ignoreSourcePolyIndex = -1;
};

struct LightmapTraceHit {
    LightmapTraceHitKind kind = LIGHTMAP_TRACE_HIT_NONE;
    float distance = 0.0f;
    int triIndex = -1;
    int occluderGroup = -1;
    int sourcePolyIndex = -1;
    uint32_t flags = LIGHTMAP_TRACE_TRI_NONE;
    int materialId = -1;
};

LightmapTraceHit LightmapTraceClosestHit(const LightmapTraceScene& scene,
                                         const Vector3& rayOrigin,
                                         const Vector3& rayDirection,
                                         const LightmapTraceQuery& query);

bool LightmapTraceOccluded(const LightmapTraceScene& scene,
                           const Vector3& rayOrigin,
                           const Vector3& rayDirection,
                           const LightmapTraceQuery& query);

float LightmapTraceClosestHitDistance(const LightmapTraceScene& scene,
                                      const Vector3& rayOrigin,
                                      const Vector3& rayDirection,
                                      const LightmapTraceQuery& query);

bool LightmapTraceBuildAcceleration(LightmapTraceScene* scene);

std::vector<LightmapComputeOccluderTri> BuildLightmapComputeOccluders(const LightmapTraceScene& scene);
