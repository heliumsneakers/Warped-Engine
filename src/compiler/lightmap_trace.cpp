#include "lightmap_trace.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#ifdef WARPED_LIGHTMAP_USE_EMBREE
#include <embree4/rtcore.h>
#include <embree4/rtcore_ray.h>
#endif

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

#ifdef WARPED_LIGHTMAP_USE_EMBREE

struct EmbreeTraceVertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 0.0f;
};

struct EmbreeTraceTriangle {
    uint32_t v0 = 0;
    uint32_t v1 = 0;
    uint32_t v2 = 0;
};

struct EmbreeTraceQueryContext : public RTCRayQueryContext {
    const LightmapTraceScene* scene = nullptr;
    const LightmapTraceQuery* query = nullptr;
};

static bool EmbreeDisabledFromEnv()
{
    const char* value = std::getenv("WARPED_LIGHTMAP_DISABLE_EMBREE");
    return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
}

static void EmbreeErrorCallback(void*, RTCError code, const char* message)
{
    printf("[Lightmap] Embree error %d: %s\n", (int)code, message ? message : "(no message)");
}

static bool EmbreeQueryIgnoresPrimitive(const LightmapTraceScene& scene,
                                        const LightmapTraceQuery& query,
                                        uint32_t primID)
{
    if (primID >= scene.tris.size()) {
        return true;
    }

    const LightmapTraceTri& tri = scene.tris[primID];
    if ((query.ignoreOccluderGroup >= 0) && (tri.occluderGroup == query.ignoreOccluderGroup)) {
        return true;
    }
    if ((query.ignoreSourcePolyIndex >= 0) && (tri.sourcePolyIndex == query.ignoreSourcePolyIndex)) {
        return true;
    }
    return false;
}

static void EmbreeTraceFilterFunction(const RTCFilterFunctionNArguments* args)
{
    EmbreeTraceQueryContext* context = static_cast<EmbreeTraceQueryContext*>(args->context);
    if (!context || !context->scene || !context->query) {
        return;
    }

    constexpr int kValid = -1;
    constexpr int kInvalid = 0;
    int* valid = args->valid;
    const unsigned int rayCount = args->N;
    RTCHitN* hit = args->hit;

    for (unsigned int i = 0; i < rayCount; ++i) {
        if (valid[i] != kValid) {
            continue;
        }

        const uint32_t primID = RTCHitN_primID(hit, rayCount, i);
        if (EmbreeQueryIgnoresPrimitive(*context->scene, *context->query, primID)) {
            valid[i] = kInvalid;
        }
    }
}

static bool EmbreeQueryNeedsFilter(const LightmapTraceQuery& query)
{
    return query.ignoreOccluderGroup >= 0 || query.ignoreSourcePolyIndex >= 0;
}

static void InitEmbreeRay(RTCRay* ray,
                          const Vector3& rayOrigin,
                          const Vector3& rayDirection,
                          const LightmapTraceQuery& query)
{
    ray->org_x = rayOrigin.x;
    ray->org_y = rayOrigin.y;
    ray->org_z = rayOrigin.z;
    ray->tnear = query.minHitT;
    ray->dir_x = rayDirection.x;
    ray->dir_y = rayDirection.y;
    ray->dir_z = rayDirection.z;
    ray->time = 0.0f;
    ray->tfar = query.maxHitT;
    ray->mask = 1u;
    ray->id = 0u;
    ray->flags = 0u;
}

static void ConfigureEmbreeIntersectArguments(RTCIntersectArguments* args,
                                              EmbreeTraceQueryContext* context,
                                              const LightmapTraceScene& scene,
                                              const LightmapTraceQuery& query)
{
    rtcInitIntersectArguments(args);
    if (!EmbreeQueryNeedsFilter(query)) {
        return;
    }

    rtcInitRayQueryContext(context);
    context->scene = &scene;
    context->query = &query;
    args->context = context;
    args->filter = EmbreeTraceFilterFunction;
    args->flags = (RTCRayQueryFlags)(args->flags | RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER);
}

static void ConfigureEmbreeOccludedArguments(RTCOccludedArguments* args,
                                             EmbreeTraceQueryContext* context,
                                             const LightmapTraceScene& scene,
                                             const LightmapTraceQuery& query)
{
    rtcInitOccludedArguments(args);
    if (!EmbreeQueryNeedsFilter(query)) {
        return;
    }

    rtcInitRayQueryContext(context);
    context->scene = &scene;
    context->query = &query;
    args->context = context;
    args->filter = EmbreeTraceFilterFunction;
    args->flags = (RTCRayQueryFlags)(args->flags | RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER);
}

#endif

} // namespace

#ifdef WARPED_LIGHTMAP_USE_EMBREE

// One Embree triangle geometry mirrors the scalar occluder list. Embree primID
// maps directly back to LightmapTraceScene::tris, while argument filters handle
// per-ray self-occluder ignores without rebuilding the BVH.
class LightmapTraceAcceleration {
public:
    LightmapTraceAcceleration() = default;
    LightmapTraceAcceleration(const LightmapTraceAcceleration&) = delete;
    LightmapTraceAcceleration& operator=(const LightmapTraceAcceleration&) = delete;

    ~LightmapTraceAcceleration()
    {
        if (scene) {
            rtcReleaseScene(scene);
            scene = nullptr;
        }
        if (device) {
            rtcReleaseDevice(device);
            device = nullptr;
        }
    }

    bool Build(const LightmapTraceScene& source)
    {
        if (source.tris.empty()) {
            return false;
        }
        if (source.tris.size() > ((size_t)UINT32_MAX / 3u)) {
            return false;
        }

        device = rtcNewDevice(nullptr);
        if (!device) {
            return false;
        }
        rtcSetDeviceErrorFunction(device, EmbreeErrorCallback, nullptr);

        scene = rtcNewScene(device);
        if (!scene) {
            return false;
        }
        rtcSetSceneFlags(scene, RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS);
        rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_HIGH);

        RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
        if (!geometry) {
            return false;
        }
        rtcSetGeometryMask(geometry, 1u);
        rtcSetGeometryBuildQuality(geometry, RTC_BUILD_QUALITY_HIGH);
        rtcSetGeometryTimeStepCount(geometry, 1u);

        const size_t triCount = source.tris.size();
        EmbreeTraceVertex* vertices = (EmbreeTraceVertex*)rtcSetNewGeometryBuffer(
            geometry,
            RTC_BUFFER_TYPE_VERTEX,
            0,
            RTC_FORMAT_FLOAT3,
            sizeof(EmbreeTraceVertex),
            triCount * 3);
        EmbreeTraceTriangle* triangles = (EmbreeTraceTriangle*)rtcSetNewGeometryBuffer(
            geometry,
            RTC_BUFFER_TYPE_INDEX,
            0,
            RTC_FORMAT_UINT3,
            sizeof(EmbreeTraceTriangle),
            triCount);

        if (!vertices || !triangles) {
            rtcReleaseGeometry(geometry);
            return false;
        }

        for (size_t i = 0; i < triCount; ++i) {
            const LightmapTraceTri& tri = source.tris[i];
            const uint32_t baseVertex = (uint32_t)(i * 3);
            vertices[baseVertex + 0] = { tri.a.x, tri.a.y, tri.a.z, 0.0f };
            vertices[baseVertex + 1] = { tri.b.x, tri.b.y, tri.b.z, 0.0f };
            vertices[baseVertex + 2] = { tri.c.x, tri.c.y, tri.c.z, 0.0f };
            triangles[i] = { baseVertex + 0, baseVertex + 1, baseVertex + 2 };
        }

        rtcCommitGeometry(geometry);
        geometryId = rtcAttachGeometry(scene, geometry);
        rtcReleaseGeometry(geometry);

        if (geometryId == RTC_INVALID_GEOMETRY_ID) {
            return false;
        }

        rtcCommitScene(scene);
        const RTCError error = rtcGetDeviceError(device);
        return error == RTC_ERROR_NONE;
    }

    LightmapTraceHit ClosestHit(const LightmapTraceScene& source,
                                const Vector3& rayOrigin,
                                const Vector3& rayDirection,
                                const LightmapTraceQuery& query) const
    {
        if (!scene || query.maxHitT <= query.minHitT) {
            return {};
        }

        RTCRayHit rayHit{};
        InitEmbreeRay(&rayHit.ray, rayOrigin, rayDirection, query);
        rayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rayHit.hit.primID = RTC_INVALID_GEOMETRY_ID;
        rayHit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        EmbreeTraceQueryContext context{};
        RTCIntersectArguments args{};
        ConfigureEmbreeIntersectArguments(&args, &context, source, query);
        rtcIntersect1(scene, &rayHit, &args);

        if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
            rayHit.hit.primID == RTC_INVALID_GEOMETRY_ID ||
            rayHit.hit.primID >= source.tris.size()) {
            return {};
        }

        const uint32_t primID = rayHit.hit.primID;
        const LightmapTraceTri& tri = source.tris[primID];

        LightmapTraceHit hit{};
        hit.kind = TraceHitKindForFlags(tri.flags);
        hit.distance = rayHit.ray.tfar;
        hit.triIndex = (int)primID;
        hit.occluderGroup = tri.occluderGroup;
        hit.sourcePolyIndex = tri.sourcePolyIndex;
        hit.flags = tri.flags;
        hit.materialId = tri.materialId;
        return hit;
    }

    bool Occluded(const LightmapTraceScene& source,
                  const Vector3& rayOrigin,
                  const Vector3& rayDirection,
                  const LightmapTraceQuery& query) const
    {
        if (!scene || query.maxHitT <= query.minHitT) {
            return false;
        }

        RTCRay ray{};
        InitEmbreeRay(&ray, rayOrigin, rayDirection, query);

        EmbreeTraceQueryContext context{};
        RTCOccludedArguments args{};
        ConfigureEmbreeOccludedArguments(&args, &context, source, query);
        rtcOccluded1(scene, &ray, &args);
        return ray.tfar < 0.0f;
    }

private:
    RTCDevice device = nullptr;
    RTCScene scene = nullptr;
    unsigned int geometryId = RTC_INVALID_GEOMETRY_ID;
};

#endif

LightmapTraceHit LightmapTraceClosestHit(const LightmapTraceScene& scene,
                                         const Vector3& rayOrigin,
                                         const Vector3& rayDirection,
                                         const LightmapTraceQuery& query)
{
#ifdef WARPED_LIGHTMAP_USE_EMBREE
    if (scene.acceleration) {
        return scene.acceleration->ClosestHit(scene, rayOrigin, rayDirection, query);
    }
#endif

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
#ifdef WARPED_LIGHTMAP_USE_EMBREE
    if (scene.acceleration) {
        return scene.acceleration->Occluded(scene, rayOrigin, rayDirection, query);
    }
#endif

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

bool LightmapTraceBuildAcceleration(LightmapTraceScene* scene)
{
    if (!scene) {
        return false;
    }

    scene->acceleration.reset();

#ifdef WARPED_LIGHTMAP_USE_EMBREE
    if (EmbreeDisabledFromEnv() || scene->tris.empty()) {
        return false;
    }

    std::shared_ptr<LightmapTraceAcceleration> acceleration = std::make_shared<LightmapTraceAcceleration>();
    if (!acceleration->Build(*scene)) {
        printf("[Lightmap] Embree CPU trace acceleration unavailable; using scalar triangle trace.\n");
        return false;
    }

    scene->acceleration = std::move(acceleration);
    printf("[Lightmap] Embree CPU trace acceleration enabled (%zu triangles).\n", scene->tris.size());
    return true;
#else
    return false;
#endif
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
