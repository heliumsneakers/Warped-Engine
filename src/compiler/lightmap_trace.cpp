#include "lightmap_trace.h"

#include <algorithm>
#include <cmath>

namespace {

static bool RayTriDistance(const Vector3& rayOrigin,
                           const Vector3& rayDirection,
                           const LightmapTraceTri& tri,
                           float tmin,
                           float tmax,
                           float* outDistance = nullptr)
{
    constexpr float kRayEpsilon = 1e-4f;
    const float baryEpsilon = kRayEpsilon;
    const Vector3 e1 = Vector3Subtract(tri.b, tri.a);
    const Vector3 e2 = Vector3Subtract(tri.c, tri.a);
    const Vector3 p = Vector3CrossProduct(rayDirection, e2);
    const float det = Vector3DotProduct(e1, p);
    if (fabsf(det) < kRayEpsilon) {
        return false;
    }

    const float invDet = 1.0f / det;
    const Vector3 s = Vector3Subtract(rayOrigin, tri.a);
    const float u = Vector3DotProduct(s, p) * invDet;
    if (u < -baryEpsilon || u > 1.0f + baryEpsilon) {
        return false;
    }

    const Vector3 q = Vector3CrossProduct(s, e1);
    const float v = Vector3DotProduct(rayDirection, q) * invDet;
    if (v < -baryEpsilon || (u + v) > 1.0f + baryEpsilon) {
        return false;
    }

    const float t = Vector3DotProduct(e2, q) * invDet;
    if (t <= tmin || t >= tmax) {
        return false;
    }

    if (outDistance) {
        *outDistance = t;
    }
    return true;
}

static bool RayAABB(const Vector3& rayOrigin,
                    const Vector3& rayInvDirection,
                    const AABB& bounds,
                    float tmax)
{
    float t1;
    float t2;
    float nearT = 0.0f;
    float farT = tmax;

    t1 = (bounds.min.x - rayOrigin.x) * rayInvDirection.x;
    t2 = (bounds.max.x - rayOrigin.x) * rayInvDirection.x;
    nearT = std::max(nearT, std::min(t1, t2));
    farT = std::min(farT, std::max(t1, t2));

    t1 = (bounds.min.y - rayOrigin.y) * rayInvDirection.y;
    t2 = (bounds.max.y - rayOrigin.y) * rayInvDirection.y;
    nearT = std::max(nearT, std::min(t1, t2));
    farT = std::min(farT, std::max(t1, t2));

    t1 = (bounds.min.z - rayOrigin.z) * rayInvDirection.z;
    t2 = (bounds.max.z - rayOrigin.z) * rayInvDirection.z;
    nearT = std::max(nearT, std::min(t1, t2));
    farT = std::min(farT, std::max(t1, t2));

    return farT >= nearT;
}

static Vector3 SafeInverseDirection(const Vector3& rayDirection) {
    constexpr float kRayEpsilon = 1e-4f;
    return {
        1.0f / (fabsf(rayDirection.x) > kRayEpsilon ? rayDirection.x : kRayEpsilon),
        1.0f / (fabsf(rayDirection.y) > kRayEpsilon ? rayDirection.y : kRayEpsilon),
        1.0f / (fabsf(rayDirection.z) > kRayEpsilon ? rayDirection.z : kRayEpsilon)
    };
}

static LightmapTraceHitKind TraceHitKindForFlags(uint32_t flags) {
    if ((flags & LIGHTMAP_TRACE_TRI_SKY) != 0u) {
        return LIGHTMAP_TRACE_HIT_SKY;
    }
    if ((flags & LIGHTMAP_TRACE_TRI_FILTERED) != 0u) {
        return LIGHTMAP_TRACE_HIT_FILTERED;
    }
    return LIGHTMAP_TRACE_HIT_SOLID;
}

} // namespace

LightmapTraceHit LightmapTraceClosestHit(const LightmapTraceScene& scene,
                                         const Vector3& rayOrigin,
                                         const Vector3& rayDirection,
                                         const LightmapTraceQuery& query)
{
    const Vector3 invDirection = SafeInverseDirection(rayDirection);

    LightmapTraceHit hit{};
    float closestDistance = query.maxHitT;
    for (size_t triIndex = 0; triIndex < scene.tris.size(); ++triIndex) {
        const LightmapTraceTri& tri = scene.tris[triIndex];
        if ((query.ignoreOccluderGroup >= 0) && (tri.occluderGroup == query.ignoreOccluderGroup)) {
            continue;
        }
        if ((query.ignoreSourcePolyIndex >= 0) && (tri.sourcePolyIndex == query.ignoreSourcePolyIndex)) {
            continue;
        }
        if (!RayAABB(rayOrigin, invDirection, tri.bounds, closestDistance)) {
            continue;
        }

        float triDistance = 0.0f;
        if (!RayTriDistance(rayOrigin, rayDirection, tri, query.minHitT, closestDistance, &triDistance)) {
            continue;
        }

        closestDistance = triDistance;
        hit.kind = TraceHitKindForFlags(tri.flags);
        hit.distance = triDistance;
        hit.triIndex = (int)triIndex;
        hit.occluderGroup = tri.occluderGroup;
        hit.sourcePolyIndex = tri.sourcePolyIndex;
        hit.flags = tri.flags;
        hit.materialId = tri.materialId;
    }

    return hit;
}

bool LightmapTraceOccluded(const LightmapTraceScene& scene,
                           const Vector3& rayOrigin,
                           const Vector3& rayDirection,
                           const LightmapTraceQuery& query)
{
    return LightmapTraceClosestHit(scene, rayOrigin, rayDirection, query).kind != LIGHTMAP_TRACE_HIT_NONE;
}

float LightmapTraceClosestHitDistance(const LightmapTraceScene& scene,
                                      const Vector3& rayOrigin,
                                      const Vector3& rayDirection,
                                      const LightmapTraceQuery& query)
{
    const LightmapTraceHit hit = LightmapTraceClosestHit(scene, rayOrigin, rayDirection, query);
    return (hit.kind == LIGHTMAP_TRACE_HIT_NONE) ? query.maxHitT : hit.distance;
}

std::vector<LightmapComputeOccluderTri> BuildLightmapComputeOccluders(const LightmapTraceScene& scene)
{
    std::vector<LightmapComputeOccluderTri> occluders;
    occluders.reserve(scene.tris.size());
    for (const LightmapTraceTri& tri : scene.tris) {
        LightmapComputeOccluderTri gpuTri{};
        gpuTri.a = tri.a;
        gpuTri.b = tri.b;
        gpuTri.c = tri.c;
        gpuTri.bounds = tri.bounds;
        gpuTri.occluderGroup = tri.occluderGroup;
        gpuTri.sourcePolyIndex = tri.sourcePolyIndex;
        occluders.push_back(gpuTri);
    }
    return occluders;
}
