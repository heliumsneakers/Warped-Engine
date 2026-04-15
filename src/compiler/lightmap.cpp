// lightmap.cpp  —  offline lightmap baker with optional GPU compute path.
//
// Oversized faces are subdivided before baking so we can preserve 1:1 luxel
// density while packing into multiple smaller lightmap pages instead of one
// extremely tall atlas.

#include "lightmap.h"
#include "lightmap_compute.h"
#include "lightmap_trace.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <unordered_map>

// --------------------------------------------------------------------------
//  Tunables
// --------------------------------------------------------------------------
static constexpr int   LM_PAD             = 2;      // border luxels (filled by dilate)
static constexpr int   LIGHTMAP_PAGE_SIZE = 1024;
static constexpr int   FACE_MAX_LUXELS    = 127;    // Source-style per-face luxel cap (interior only)
static constexpr float SHADOW_BIAS        = 0.03125f;
static constexpr float RAY_TRACE_TMIN     = 1e-4f;
static constexpr float SOLID_REPAIR_EPSILON = 0.05f;
static constexpr float SAMPLE_REPAIR_JITTER = 0.5f;
static constexpr float DARK_LUXEL_THRESHOLD = 8.0f / 255.0f;
static constexpr float OCCLUSION_NEAR_TMIN = RAY_TRACE_TMIN;
static constexpr float EDGE_SEAM_GUARD_LUXELS = 0.75f;
// Indirect bounce is emitted from lit patch surfaces. Keep the first radiosity
// pass conservative so it lifts occluded regions without flattening direct
// shadow contrast into an overcast look.
static constexpr float INDIRECT_BOUNCE_REFLECTANCE = 0.18f;
static constexpr float INDIRECT_BOUNCE_EMITTER_THRESHOLD = 1.0f / 255.0f;
static constexpr float INDIRECT_BOUNCE_SURFACE_OFFSET = 1.0f;
static constexpr float DIRECT_SURFACE_HOTSPOT_CLAMP = 16.0f;
static constexpr float INDIRECT_SURFACE_HOTSPOT_CLAMP = 128.0f;
static constexpr float SURFACE_EMITTER_TRACE_THRESHOLD = 1.0f / 255.0f;
static constexpr float SURFACE_EMITTER_CULL_THRESHOLD = 1.0f / 255.0f;
static constexpr float SURFACE_EMITTER_CULL_RADIUS_MIN = 32.0f;
static constexpr float SURFACE_EMITTER_CULL_RADIUS_MAX = 8192.0f;
static constexpr float LIGHT_ANGLE_EPSILON = 0.01f;
// Runtime super-sampling grid size set at the top of BakeLightmap from
// settings.extraSamples (which parses the worldspawn `_extra_samples` key).
// Valid values: 1 = off (1 sample per luxel), 2 = 2x2, 4 = 4x4 (historical
// default). Shared by BakeLightmapCPUPage, BuildCoverageMask, and every
// function that compares luxel coverage against g_aaGrid * g_aaGrid.
static int g_aaGrid = 4;
// Pass count for StabilizeEdgeTexels — kept independent of g_aaGrid so that
// edge-propagation still works when extraSamples = 0 (g_aaGrid = 1).
static constexpr int   STABILIZE_EDGE_PASSES = 4;
static constexpr int   DILATE_PASSES      = 4;
static constexpr int   ERICW_SUNSAMPLES   = 400;    // matches ericw-tools default sunsamples
static constexpr int   DIRT_NUM_ANGLE_STEPS = 16;
static constexpr int   DIRT_NUM_ELEVATION_STEPS = 3;
static constexpr int   DIRT_RAY_COUNT = DIRT_NUM_ANGLE_STEPS * DIRT_NUM_ELEVATION_STEPS;

static bool ForceLightmapCPUFromEnv()
{
    const char* value = std::getenv("WARPED_LIGHTMAP_FORCE_CPU");
    return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
}

// --------------------------------------------------------------------------
//  Planar basis
// --------------------------------------------------------------------------
static void FaceBasis(const Vector3& n, Vector3& u, Vector3& v) {
    Vector3 ref = (fabsf(n.y) < 0.9f) ? (Vector3){0,1,0} : (Vector3){1,0,0};
    u = Vector3Normalize(Vector3CrossProduct(n, ref));
    v = Vector3CrossProduct(n, u);
}

struct LocalPolyVert {
    Vector3 world{};
    float   u = 0.0f;
    float   v = 0.0f;
};

struct BakePatch {
    MapPolygon poly{};
    uint32_t sourcePolyIndex = 0;
};

struct SurfaceLightEmitter {
    PointLight baseLight{};
    Vector3 surfaceNormal{};
    std::vector<Vector3> samplePoints;
    AABB bounds{};
    float sampleIntensityScale = 1.0f;
    float attenuationScale = 1.0f;
    float transportScale = 1.0f;
    float hotspotClamp = DIRECT_SURFACE_HOTSPOT_CLAMP;
    float surfaceArea = 0.0f;
    int bounceDepth = 0;
    uint8_t omnidirectional = 0;
    uint8_t rescale = 0;
};

static int ComputeInteriorLuxelSpan(float extentWorld, float luxelSize) {
    const float safeLuxelSize = std::max(0.125f, luxelSize);
    const float extentLuxels = std::max(0.0f, extentWorld) / safeLuxelSize;
    return std::max(2, (int)ceilf(std::max(0.0f, extentLuxels - 1e-4f)));
}

static Vector3 LerpVec3(const Vector3& a, const Vector3& b, float t) {
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

static float EvaluateLightAttenuation(const PointLight& light, float dist) {
    const float safeRadius = std::max(1e-3f, light.intensity);
    const float linear = std::max(0.0f, 1.0f - dist / safeRadius);
    const float normalizedDist = dist / std::max(1e-3f, safeRadius * 0.5f);

    switch (light.attenuationMode) {
        case POINT_LIGHT_ATTEN_LINEAR:
            return linear;
        case POINT_LIGHT_ATTEN_INVERSE:
            return linear / (1.0f + normalizedDist);
        case POINT_LIGHT_ATTEN_INVERSE_SQUARE:
            return linear / (1.0f + normalizedDist * normalizedDist);
        case POINT_LIGHT_ATTEN_NONE:
            return 1.0f;
        case POINT_LIGHT_ATTEN_LOCAL_MINLIGHT:
            return std::max(0.35f, linear);
        case POINT_LIGHT_ATTEN_INVERSE_SQUARE_B:
            return linear / (1.0f + normalizedDist * normalizedDist * 0.5f);
        case POINT_LIGHT_ATTEN_QUADRATIC:
        default:
            return linear * linear;
    }
}

static float EvaluateIncidenceScale(const PointLight& light, float ndl) {
    const float clampedLambert = std::max(0.0f, ndl);
    const float t = std::clamp(light.angleScale, 0.0f, 1.0f);
    return (1.0f - t) + t * clampedLambert;
}

static float EvaluateAngleScale(float angleScale, float ndl) {
    const float clampedLambert = std::max(0.0f, ndl);
    const float t = std::clamp(angleScale, 0.0f, 1.0f);
    return (1.0f - t) + t * clampedLambert;
}

static float EvaluateSpotlightFactor(const PointLight& light, const Vector3& dirToLight) {
    if (light.spotOuterCos <= -1.5f || Vector3LengthSq(light.spotDirection) <= 1e-6f) {
        return 1.0f;
    }

    const Vector3 lightToSurface = Vector3Scale(dirToLight, -1.0f);
    const float spotCos = Vector3DotProduct(Vector3Normalize(light.spotDirection), lightToSurface);
    if (spotCos <= light.spotOuterCos) {
        return 0.0f;
    }
    if (light.spotInnerCos <= light.spotOuterCos) {
        return 1.0f;
    }
    return std::clamp((spotCos - light.spotOuterCos) / (light.spotInnerCos - light.spotOuterCos), 0.0f, 1.0f);
}

static bool IsParallelLight(const PointLight& light) {
    return light.parallel != 0 && Vector3LengthSq(light.parallelDirection) > 1e-6f;
}

static bool LightUsesDirt(const PointLight& light, const LightBakeSettings& settings) {
    const int dirtSetting = (light.dirt == -2) ? settings.dirt : light.dirt;
    return dirtSetting == 1;
}

static float EffectiveLightDirtScale(const PointLight& light, const LightBakeSettings& settings) {
    return (light.dirtScale > 0.0f) ? light.dirtScale : settings.dirtScale;
}

static float EffectiveLightDirtGain(const PointLight& light, const LightBakeSettings& settings) {
    return (light.dirtGain > 0.0f) ? light.dirtGain : settings.dirtGain;
}

static float DistPointSegment3D(const Vector3& p, const Vector3& a, const Vector3& b) {
    const Vector3 ab = Vector3Subtract(b, a);
    const float abLenSq = Vector3LengthSq(ab);
    if (abLenSq <= 1e-8f) {
        return sqrtf(Vector3LengthSq(Vector3Subtract(p, a)));
    }
    const float t = std::clamp(Vector3DotProduct(Vector3Subtract(p, a), ab) / abLenSq, 0.0f, 1.0f);
    const Vector3 q = Vector3Add(a, Vector3Scale(ab, t));
    return sqrtf(Vector3LengthSq(Vector3Subtract(p, q)));
}

struct PhongNeighbor {
    uint32_t sourcePolyIndex = 0;
    Vector3 edgeA{};
    Vector3 edgeB{};
    Vector3 normal{};
    float areaWeight = 1.0f;
};

struct PhongSourcePoly {
    bool enabled = false;
    Vector3 normal{};
    float areaWeight = 1.0f;
    std::vector<PhongNeighbor> neighbors;
};

static constexpr int SAMPLE_REPAIR_RECURSION_MAX = 3;

struct RepairSourceNeighbor {
    uint32_t sourcePolyIndex = 0;
    int edgeIndex = -1;
};

struct RepairSourcePoly {
    Vector3 planePoint{};
    Vector3 normal{};
    Vector3 axisU{};
    Vector3 axisV{};
    std::vector<Vector2> poly2d;
    std::vector<RepairSourceNeighbor> neighbors;
};

static uint32_t HashLightSeed(const Vector3& position, int extra) {
    auto h = [](float v) -> uint32_t {
        return (uint32_t)std::lround(v * 1000.0f);
    };
    uint32_t seed = 2166136261u;
    seed = (seed ^ h(position.x)) * 16777619u;
    seed = (seed ^ h(position.y)) * 16777619u;
    seed = (seed ^ h(position.z)) * 16777619u;
    seed = (seed ^ (uint32_t)extra) * 16777619u;
    return seed ? seed : 1u;
}

static float RandomFloat01(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return (float)(state & 0x00FFFFFFu) / (float)0x01000000u;
}

static Vector3 RandomPointInUnitSphere(uint32_t& state) {
    for (int attempt = 0; attempt < 16; ++attempt) {
        const Vector3 p = {
            RandomFloat01(state) * 2.0f - 1.0f,
            RandomFloat01(state) * 2.0f - 1.0f,
            RandomFloat01(state) * 2.0f - 1.0f
        };
        if (Vector3LengthSq(p) <= 1.0f) {
            return p;
        }
    }
    return {0.0f, 0.0f, 0.0f};
}

static Vector3 TransformToTangentSpace(const Vector3& normal,
                                       const Vector3& tangent,
                                       const Vector3& bitangent,
                                       const Vector3& local)
{
    return Vector3Add(Vector3Add(Vector3Scale(bitangent, local.x),
                                 Vector3Scale(tangent, local.y)),
                      Vector3Scale(normal, local.z));
}

static Vector3 BuildEricwDirtVector(const LightBakeSettings& settings, int sampleIndex, uint32_t& seed) {
    const float dirtAngle = std::clamp(settings.dirtAngle, 1.0f, 90.0f);
    if (settings.dirtMode == 1) {
        const float angle = RandomFloat01(seed) * PI * 2.0f;
        const float elevation = RandomFloat01(seed) * dirtAngle * DEG2RAD;
        const float sinElevation = sinf(elevation);
        return {
            cosf(angle) * sinElevation,
            sinf(angle) * sinElevation,
            cosf(elevation)
        };
    }

    const float angleStep = (PI * 2.0f) / (float)DIRT_NUM_ANGLE_STEPS;
    const float elevationStep = (dirtAngle * DEG2RAD) / (float)DIRT_NUM_ELEVATION_STEPS;
    const int angleIndex = sampleIndex / DIRT_NUM_ELEVATION_STEPS;
    const int elevationIndex = sampleIndex % DIRT_NUM_ELEVATION_STEPS;
    const float angle = (float)angleIndex * angleStep;
    const float elevation = elevationStep * (0.5f + (float)elevationIndex);
    const float sinElevation = sinf(elevation);
    return {
        sinElevation * cosf(angle),
        sinElevation * sinf(angle),
        cosf(elevation)
    };
}

static void AppendDeviatedLights(const PointLight& baseLight,
                                 float deviance,
                                 int samples,
                                 int seedSalt,
                                 std::vector<PointLight>& out)
{
    if (deviance <= 0.0f || samples <= 1) {
        out.push_back(baseLight);
        return;
    }

    const int safeSamples = std::max(1, samples);
    const Vector3 sampleColor = Vector3Scale(baseLight.color, 1.0f / (float)safeSamples);
    uint32_t seed = HashLightSeed(baseLight.position, seedSalt);
    for (int i = 0; i < safeSamples; ++i) {
        PointLight split = baseLight;
        split.position = Vector3Add(baseLight.position, Vector3Scale(RandomPointInUnitSphere(seed), deviance));
        split.color = sampleColor;
        out.push_back(split);
    }
}

static AABB ComputeBoundsFromPoints(const std::vector<Vector3>& points) {
    AABB bounds = AABBInvalid();
    for (const Vector3& point : points) {
        AABBExtend(&bounds, point);
    }
    if (bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z) {
        bounds.min = Vector3Zero();
        bounds.max = Vector3Zero();
    }
    return bounds;
}

static Vector3 AveragePoints(const std::vector<Vector3>& points) {
    if (points.empty()) {
        return Vector3Zero();
    }

    Vector3 sum = Vector3Zero();
    for (const Vector3& point : points) {
        sum = Vector3Add(sum, point);
    }
    return Vector3Scale(sum, 1.0f / (float)points.size());
}

static float ComputeEmitterSampleIntensityScale(float surfaceArea,
                                                size_t sampleCount)
{
    const float safeArea = std::max(1.0f, surfaceArea);
    const float safeSampleCount = (float)std::max<size_t>(1, sampleCount);
    return safeArea / safeSampleCount;
}

static float EstimateSurfaceEmitterCullRadius(const Vector3& color,
                                              float surfaceArea,
                                              float transportScale,
                                              float attenuationScale,
                                              float hotspotClamp)
{
    const float safeArea = std::max(1.0f, surfaceArea);
    const float safeScale = std::max(0.0f, transportScale);
    const float peakColor = std::max(color.x, std::max(color.y, color.z));
    if (peakColor <= 1e-6f || safeScale <= 0.0f) {
        return SURFACE_EMITTER_CULL_RADIUS_MIN;
    }
    if (attenuationScale <= 1e-6f) {
        return SURFACE_EMITTER_CULL_RADIUS_MAX;
    }

    const float rawRadius = sqrtf((peakColor * safeArea * safeScale) /
                                  std::max(1e-6f, SURFACE_EMITTER_CULL_THRESHOLD * attenuationScale * attenuationScale));
    return std::clamp(std::max(rawRadius, hotspotClamp),
                      SURFACE_EMITTER_CULL_RADIUS_MIN,
                      SURFACE_EMITTER_CULL_RADIUS_MAX);
}

static float EvaluateSurfaceEmitterDistanceFalloff(const SurfaceLightEmitter& emitter,
                                                   float dist)
{
    const float scaledDist = std::max(emitter.hotspotClamp,
                                      std::max(0.01f, emitter.attenuationScale * dist));
    return (emitter.transportScale * emitter.sampleIntensityScale) / (scaledDist * scaledDist);
}

static Vector3 GrayBounceColor() {
    return {127.0f / 255.0f, 127.0f / 255.0f, 127.0f / 255.0f};
}

static Vector3 ComputeTextureBounceColor(const std::string& texture,
                                         const std::unordered_map<std::string, Vector3>& textureBounceColors,
                                         float bounceColorScale)
{
    const Vector3 gray = GrayBounceColor();
    auto it = textureBounceColors.find(texture);
    if (it == textureBounceColors.end()) {
        return gray;
    }
    return LerpVec3(gray, it->second, std::clamp(bounceColorScale, 0.0f, 1.0f));
}

static void AppendDeviatedSurfaceEmitters(const SurfaceLightEmitter& baseEmitter,
                                          float deviance,
                                          int samples,
                                          int seedSalt,
                                          std::vector<SurfaceLightEmitter>& out)
{
    if (deviance <= 0.0f || samples <= 1) {
        out.push_back(baseEmitter);
        return;
    }

    const int safeSamples = std::max(1, samples);
    const Vector3 sampleColor = Vector3Scale(baseEmitter.baseLight.color, 1.0f / (float)safeSamples);
    uint32_t seed = HashLightSeed(baseEmitter.baseLight.position, seedSalt);
    for (int i = 0; i < safeSamples; ++i) {
        SurfaceLightEmitter split = baseEmitter;
        const Vector3 jitter = Vector3Scale(RandomPointInUnitSphere(seed), deviance);
        for (Vector3& samplePoint : split.samplePoints) {
            samplePoint = Vector3Add(samplePoint, jitter);
        }
        split.baseLight.color = sampleColor;
        split.baseLight.position = Vector3Add(baseEmitter.baseLight.position, jitter);
        split.bounds = ComputeBoundsFromPoints(split.samplePoints);
        out.push_back(std::move(split));
    }
}

static void RemoveConsecutiveDuplicates(std::vector<LocalPolyVert>& poly) {
    if (poly.empty()) {
        return;
    }

    std::vector<LocalPolyVert> deduped;
    deduped.reserve(poly.size());
    deduped.push_back(poly[0]);
    for (size_t i = 1; i < poly.size(); ++i) {
        const LocalPolyVert& a = deduped.back();
        const LocalPolyVert& b = poly[i];
        if (fabsf(a.u - b.u) < 1e-4f &&
            fabsf(a.v - b.v) < 1e-4f &&
            Vector3LengthSq(Vector3Subtract(a.world, b.world)) < 1e-6f) {
            continue;
        }
        deduped.push_back(b);
    }

    if (deduped.size() >= 2) {
        const LocalPolyVert& a = deduped.front();
        const LocalPolyVert& b = deduped.back();
        if (fabsf(a.u - b.u) < 1e-4f &&
            fabsf(a.v - b.v) < 1e-4f &&
            Vector3LengthSq(Vector3Subtract(a.world, b.world)) < 1e-6f) {
            deduped.pop_back();
        }
    }

    poly.swap(deduped);
}

static float SignedArea2D(const std::vector<LocalPolyVert>& poly) {
    if (poly.size() < 3) {
        return 0.0f;
    }
    float area = 0.0f;
    for (size_t i = 0; i < poly.size(); ++i) {
        const size_t j = (i + 1) % poly.size();
        area += poly[i].u * poly[j].v - poly[j].u * poly[i].v;
    }
    return area * 0.5f;
}

static Vector3 ComputePolygonNormal(const std::vector<Vector3>& verts) {
    for (size_t i = 1; i + 1 < verts.size(); ++i) {
        Vector3 n = CalculateNormal(verts[0], verts[i], verts[i + 1]);
        if (Vector3LengthSq(n) > 1e-8f) {
            return n;
        }
    }
    return Vector3Zero();
}

static float PolygonArea(const std::vector<Vector3>& verts) {
    if (verts.size() < 3) {
        return 0.0f;
    }

    float area = 0.0f;
    for (size_t i = 1; i + 1 < verts.size(); ++i) {
        const Vector3 e0 = Vector3Subtract(verts[i], verts[0]);
        const Vector3 e1 = Vector3Subtract(verts[i + 1], verts[0]);
        area += 0.5f * Vector3Length(Vector3CrossProduct(e0, e1));
    }
    return area;
}

static Vector3 PolygonCentroid(const std::vector<Vector3>& verts) {
    Vector3 centroid = Vector3Zero();
    if (verts.empty()) {
        return centroid;
    }
    for (const Vector3& v : verts) {
        centroid = Vector3Add(centroid, v);
    }
    return Vector3Scale(centroid, 1.0f / (float)verts.size());
}

static bool PointsEqualEps(const Vector3& a, const Vector3& b, float eps) {
    return fabsf(a.x - b.x) <= eps &&
           fabsf(a.y - b.y) <= eps &&
           fabsf(a.z - b.z) <= eps;
}

static std::string MakeQuantizedPointKey(const Vector3& p, float eps) {
    const float inv = 1.0f / std::max(1e-6f, eps);
    const int x = (int)std::lround(p.x * inv);
    const int y = (int)std::lround(p.y * inv);
    const int z = (int)std::lround(p.z * inv);
    char buffer[96];
    snprintf(buffer, sizeof(buffer), "%d,%d,%d", x, y, z);
    return std::string(buffer);
}

static std::string MakeQuantizedEdgeKey(const Vector3& a, const Vector3& b, float eps) {
    std::string ka = MakeQuantizedPointKey(a, eps);
    std::string kb = MakeQuantizedPointKey(b, eps);
    if (kb < ka) {
        std::swap(ka, kb);
    }
    return ka + "|" + kb;
}

struct EdgeRef {
    uint32_t sourcePolyIndex = 0;
    Vector3 start{};
    Vector3 end{};
};

static bool PhongGroupsCompatible(const MapPolygon& a, const MapPolygon& b) {
    if (a.sourceEntityId != b.sourceEntityId) {
        return false;
    }
    if (a.phongGroup == 0 && b.phongGroup == 0) {
        return true;
    }
    return a.phongGroup != 0 && a.phongGroup == b.phongGroup;
}

static bool IsConcaveJoint(const MapPolygon& a, const MapPolygon& b) {
    const Vector3 ca = PolygonCentroid(a.verts);
    const Vector3 cb = PolygonCentroid(b.verts);
    return Vector3DotProduct(a.normal, Vector3Subtract(cb, ca)) > 1e-3f ||
           Vector3DotProduct(b.normal, Vector3Subtract(ca, cb)) > 1e-3f;
}

static int FindMatchingPolyEdgeIndex(const MapPolygon& poly,
                                     const Vector3& edgeA,
                                     const Vector3& edgeB)
{
    if (poly.verts.size() < 2) {
        return -1;
    }

    constexpr float kMatchEpsilon = 0.05f;
    for (size_t edgeIndex = 0; edgeIndex < poly.verts.size(); ++edgeIndex) {
        const Vector3& a = poly.verts[edgeIndex];
        const Vector3& b = poly.verts[(edgeIndex + 1) % poly.verts.size()];
        if ((PointsEqualEps(a, edgeA, kMatchEpsilon) && PointsEqualEps(b, edgeB, kMatchEpsilon)) ||
            (PointsEqualEps(a, edgeB, kMatchEpsilon) && PointsEqualEps(b, edgeA, kMatchEpsilon)))
        {
            return (int)edgeIndex;
        }
    }

    return -1;
}

static std::vector<PhongSourcePoly> BuildPhongSourcePolys(const std::vector<MapPolygon>& polys) {
    std::vector<PhongSourcePoly> sourcePolys(polys.size());
    constexpr float kEdgeKeyEpsilon = 0.05f;
    std::unordered_map<std::string, std::vector<EdgeRef>> edgesByKey;
    edgesByKey.reserve(polys.size() * 4);

    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& poly = polys[i];
        sourcePolys[i].enabled = poly.phong != 0;
        sourcePolys[i].normal = poly.normal;
        sourcePolys[i].areaWeight = std::max(1.0f, sqrtf(std::max(1.0f, PolygonArea(poly.verts))));
        if (!sourcePolys[i].enabled || poly.verts.size() < 3) {
            continue;
        }
        for (size_t vi = 0; vi < poly.verts.size(); ++vi) {
            const Vector3& a = poly.verts[vi];
            const Vector3& b = poly.verts[(vi + 1) % poly.verts.size()];
            edgesByKey[MakeQuantizedEdgeKey(a, b, kEdgeKeyEpsilon)].push_back({ (uint32_t)i, a, b });
        }
    }

    for (const auto& [key, refs] : edgesByKey) {
        (void)key;
        if (refs.size() < 2) {
            continue;
        }
        for (size_t i = 0; i < refs.size(); ++i) {
            const MapPolygon& a = polys[refs[i].sourcePolyIndex];
            if (!a.phong) {
                continue;
            }
            for (size_t j = i + 1; j < refs.size(); ++j) {
                const MapPolygon& b = polys[refs[j].sourcePolyIndex];
                if (!b.phong || !PhongGroupsCompatible(a, b)) {
                    continue;
                }

                const float dotNormals = std::clamp(Vector3DotProduct(a.normal, b.normal), -1.0f, 1.0f);
                const float angleDeg = acosf(dotNormals) * RAD2DEG;
                const bool concave = IsConcaveJoint(a, b);
                const float aLimit = (concave && a.phongAngleConcave > 0.0f) ? a.phongAngleConcave : a.phongAngle;
                const float bLimit = (concave && b.phongAngleConcave > 0.0f) ? b.phongAngleConcave : b.phongAngle;
                if (angleDeg > std::min(aLimit, bLimit)) {
                    continue;
                }

                const Vector3 edgeA = refs[i].start;
                const Vector3 edgeB = refs[i].end;
                sourcePolys[refs[i].sourcePolyIndex].neighbors.push_back({
                    refs[j].sourcePolyIndex,
                    edgeA,
                    edgeB,
                    b.normal,
                    sourcePolys[refs[j].sourcePolyIndex].areaWeight
                });
                sourcePolys[refs[j].sourcePolyIndex].neighbors.push_back({
                    refs[i].sourcePolyIndex,
                    refs[j].start,
                    refs[j].end,
                    a.normal,
                    sourcePolys[refs[i].sourcePolyIndex].areaWeight
                });
            }
        }
    }

    return sourcePolys;
}

static std::vector<RepairSourcePoly> BuildRepairSourcePolys(const std::vector<MapPolygon>& polys,
                                                            const std::vector<PhongSourcePoly>& sourcePhongs)
{
    std::vector<RepairSourcePoly> repairPolys(polys.size());
    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& poly = polys[i];
        RepairSourcePoly& repair = repairPolys[i];
        repair.planePoint = poly.verts.empty() ? Vector3Zero() : poly.verts[0];
        repair.normal = poly.normal;
        FaceBasis(poly.normal, repair.axisU, repair.axisV);
        repair.poly2d.reserve(poly.verts.size());
        for (const Vector3& vert : poly.verts) {
            repair.poly2d.push_back({
                Vector3DotProduct(vert, repair.axisU),
                Vector3DotProduct(vert, repair.axisV)
            });
        }

        if (i >= sourcePhongs.size()) {
            continue;
        }

        repair.neighbors.reserve(sourcePhongs[i].neighbors.size());
        for (const PhongNeighbor& neighbor : sourcePhongs[i].neighbors) {
            if (neighbor.sourcePolyIndex >= polys.size()) {
                continue;
            }
            const int edgeIndex = FindMatchingPolyEdgeIndex(poly, neighbor.edgeA, neighbor.edgeB);
            if (edgeIndex < 0) {
                continue;
            }
            repair.neighbors.push_back({ neighbor.sourcePolyIndex, edgeIndex });
        }
    }

    return repairPolys;
}

static Vector3 EvaluatePhongNormal(const std::vector<PhongSourcePoly>& sourcePolys,
                                   uint32_t sourcePolyIndex,
                                   const Vector3& samplePoint,
                                   float luxelSize)
{
    if (sourcePolyIndex >= sourcePolys.size()) {
        return Vector3Zero();
    }
    const PhongSourcePoly& source = sourcePolys[sourcePolyIndex];
    if (!source.enabled || source.neighbors.empty()) {
        return source.normal;
    }

    Vector3 accum = Vector3Scale(source.normal, source.areaWeight * 2.0f);
    const float distanceBias = std::max(0.25f, luxelSize);
    for (const PhongNeighbor& neighbor : source.neighbors) {
        const float dist = DistPointSegment3D(samplePoint, neighbor.edgeA, neighbor.edgeB);
        const float weight = neighbor.areaWeight / std::max(distanceBias, dist);
        accum = Vector3Add(accum, Vector3Scale(neighbor.normal, weight));
    }

    if (Vector3LengthSq(accum) <= 1e-8f) {
        return source.normal;
    }
    return Vector3Normalize(accum);
}

template <typename InsideFn, typename IntersectFn>
static std::vector<LocalPolyVert> ClipLocalPolygon(const std::vector<LocalPolyVert>& input,
                                                   InsideFn inside,
                                                   IntersectFn intersect)
{
    std::vector<LocalPolyVert> output;
    if (input.empty()) {
        return output;
    }

    output.reserve(input.size() + 2);
    LocalPolyVert prev = input.back();
    bool prevInside = inside(prev);

    for (const LocalPolyVert& curr : input) {
        const bool currInside = inside(curr);
        if (currInside != prevInside) {
            output.push_back(intersect(prev, curr));
        }
        if (currInside) {
            output.push_back(curr);
        }
        prev = curr;
        prevInside = currInside;
    }

    RemoveConsecutiveDuplicates(output);
    return output;
}

static std::vector<LocalPolyVert> ClipToRect(const std::vector<LocalPolyVert>& poly,
                                             float minU, float maxU,
                                             float minV, float maxV)
{
    auto clipped = ClipLocalPolygon(poly,
        [minU](const LocalPolyVert& p) { return p.u >= minU - 1e-4f; },
        [minU](const LocalPolyVert& a, const LocalPolyVert& b) {
            float denom = b.u - a.u;
            float t = fabsf(denom) > 1e-6f ? (minU - a.u) / denom : 0.0f;
            return LocalPolyVert{ LerpVec3(a.world, b.world, t), minU, a.v + (b.v - a.v) * t };
        });

    clipped = ClipLocalPolygon(clipped,
        [maxU](const LocalPolyVert& p) { return p.u <= maxU + 1e-4f; },
        [maxU](const LocalPolyVert& a, const LocalPolyVert& b) {
            float denom = b.u - a.u;
            float t = fabsf(denom) > 1e-6f ? (maxU - a.u) / denom : 0.0f;
            return LocalPolyVert{ LerpVec3(a.world, b.world, t), maxU, a.v + (b.v - a.v) * t };
        });

    clipped = ClipLocalPolygon(clipped,
        [minV](const LocalPolyVert& p) { return p.v >= minV - 1e-4f; },
        [minV](const LocalPolyVert& a, const LocalPolyVert& b) {
            float denom = b.v - a.v;
            float t = fabsf(denom) > 1e-6f ? (minV - a.v) / denom : 0.0f;
            return LocalPolyVert{ LerpVec3(a.world, b.world, t), a.u + (b.u - a.u) * t, minV };
        });

    clipped = ClipLocalPolygon(clipped,
        [maxV](const LocalPolyVert& p) { return p.v <= maxV + 1e-4f; },
        [maxV](const LocalPolyVert& a, const LocalPolyVert& b) {
            float denom = b.v - a.v;
            float t = fabsf(denom) > 1e-6f ? (maxV - a.v) / denom : 0.0f;
            return LocalPolyVert{ LerpVec3(a.world, b.world, t), a.u + (b.u - a.u) * t, maxV };
        });

    return clipped;
}

static std::vector<BakePatch> SubdivideLightmappedPolygon(const MapPolygon& poly,
                                                          uint32_t sourcePolyIndex,
                                                          float luxelSize) {
    if (poly.verts.size() < 3) {
        return {};
    }

    Vector3 basisU, basisV;
    FaceBasis(poly.normal, basisU, basisV);

    std::vector<LocalPolyVert> localPoly;
    localPoly.reserve(poly.verts.size());

    float minU = 1e30f, maxU = -1e30f;
    float minV = 1e30f, maxV = -1e30f;
    for (const Vector3& vert : poly.verts) {
        const float u = Vector3DotProduct(vert, basisU);
        const float v = Vector3DotProduct(vert, basisV);
        localPoly.push_back({ vert, u, v });
        minU = std::min(minU, u); maxU = std::max(maxU, u);
        minV = std::min(minV, v); maxV = std::max(maxV, v);
    }

    const float safeLuxelSize = std::max(0.125f, luxelSize);
    const float tileExtent = FACE_MAX_LUXELS * safeLuxelSize;
    const float startU = floorf(minU / tileExtent) * tileExtent;
    const float startV = floorf(minV / tileExtent) * tileExtent;
    const int tilesU = std::max(1, (int)ceilf((maxU - startU) / tileExtent));
    const int tilesV = std::max(1, (int)ceilf((maxV - startV) / tileExtent));

    std::vector<BakePatch> out;
    out.reserve((size_t)tilesU * (size_t)tilesV);

    for (int ty = 0; ty < tilesV; ++ty) {
        const float clipMinV = startV + ty * tileExtent;
        const float clipMaxV = clipMinV + tileExtent;
        for (int tx = 0; tx < tilesU; ++tx) {
            const float clipMinU = startU + tx * tileExtent;
            const float clipMaxU = clipMinU + tileExtent;

            std::vector<LocalPolyVert> clipped = ClipToRect(localPoly, clipMinU, clipMaxU, clipMinV, clipMaxV);
            if (clipped.size() < 3 || fabsf(SignedArea2D(clipped)) < 1e-3f) {
                continue;
            }

            BakePatch patch{};
            patch.poly = poly;
            patch.poly.verts.clear();
            patch.poly.verts.reserve(clipped.size());
            patch.sourcePolyIndex = sourcePolyIndex;
            for (const LocalPolyVert& v : clipped) {
                patch.poly.verts.push_back(v.world);
            }

            RemoveDuplicatePoints(patch.poly.verts, 1e-4f);
            if (patch.poly.verts.size() < 3) {
                continue;
            }

            Vector3 subNormal = ComputePolygonNormal(patch.poly.verts);
            if (Vector3LengthSq(subNormal) <= 1e-8f) {
                continue;
            }
            if (Vector3DotProduct(subNormal, poly.normal) < 0.0f) {
                std::reverse(patch.poly.verts.begin(), patch.poly.verts.end());
            }

            out.push_back(std::move(patch));
        }
    }

    if (out.empty()) {
        out.push_back({ poly, sourcePolyIndex });
    }
    return out;
}

static std::vector<BakePatch> SubdivideLightmappedPolygons(const std::vector<MapPolygon>& polys,
                                                           float luxelSize) {
    std::vector<BakePatch> out;
    out.reserve(polys.size());

    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& poly = polys[i];
        std::vector<BakePatch> parts = SubdivideLightmappedPolygon(poly, (uint32_t)i, luxelSize);
        out.insert(out.end(),
                   std::make_move_iterator(parts.begin()),
                   std::make_move_iterator(parts.end()));
    }

    if (out.size() != polys.size()) {
        printf("[Lightmap] subdivided %zu faces into %zu bake faces.\n", polys.size(), out.size());
    }

    std::vector<float> sourceArea(polys.size(), 0.0f);
    std::vector<float> patchArea(polys.size(), 0.0f);
    for (size_t i = 0; i < polys.size(); ++i) {
        sourceArea[i] = PolygonArea(polys[i].verts);
    }
    for (const BakePatch& patch : out) {
        if (patch.sourcePolyIndex < patchArea.size()) {
            patchArea[patch.sourcePolyIndex] += PolygonArea(patch.poly.verts);
        }
    }

    int warned = 0;
    for (size_t i = 0; i < polys.size(); ++i) {
        const float src = sourceArea[i];
        const float patched = patchArea[i];
        const float relErr = (src > 1e-3f) ? fabsf(patched - src) / src : fabsf(patched - src);
        if (relErr > 0.01f) {
            printf("[Lightmap] patch coverage mismatch on face %zu: sourceArea=%.3f patchArea=%.3f relErr=%.4f verts=%zu\n",
                   i, src, patched, relErr, polys[i].verts.size());
            if (++warned >= 12) {
                printf("[Lightmap] patch coverage mismatch warnings truncated.\n");
                break;
            }
        }
    }
    return out;
}

// --------------------------------------------------------------------------
//  Occluders
// --------------------------------------------------------------------------
using OccluderSet = LightmapTraceScene;

struct SurfaceVisibilityInfo {
    std::vector<uint8_t> hidden;
    std::vector<uint8_t> emitterOnly;
    int hiddenCount = 0;
    int emitterOnlyCount = 0;
};

struct BrushSolidPlane {
    Vector3 point{};
    Vector3 normal{};
};

struct BrushSolid {
    int sourceBrushId = -1;
    std::vector<BrushSolidPlane> planes;
};

struct RepairedSamplePoint {
    Vector3 planePoint{};
    Vector3 samplePoint{};
    uint32_t sourcePolyIndex = 0;
    Vector3 ownerNormal{};
    Vector3 ownerAxisU{};
    Vector3 ownerAxisV{};
    bool valid = false;
};

static bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py);
static float MinDistToPolyEdge2D(const std::vector<Vector2>& poly, float px, float py);
static Vector3 OffsetPointAlongSurfaceNormal(const Vector3& planePoint,
                                             const Vector3& faceNormal,
                                             float offsetDistance);
static bool TryRepairSampleCandidate(const std::vector<BrushSolid>& solids,
                                     const Vector3& planePoint,
                                     const Vector3& faceNormal,
                                     float offsetDistance,
                                     RepairedSamplePoint* ioSample);

static void SetRepairedSampleOwnerContext(RepairedSamplePoint* sample,
                                          uint32_t sourcePolyIndex,
                                          const Vector3& planePoint,
                                          const Vector3& ownerNormal,
                                          const Vector3& ownerAxisU,
                                          const Vector3& ownerAxisV)
{
    if (!sample) {
        return;
    }
    sample->sourcePolyIndex = sourcePolyIndex;
    sample->planePoint = planePoint;
    sample->ownerNormal = ownerNormal;
    sample->ownerAxisU = ownerAxisU;
    sample->ownerAxisV = ownerAxisV;
}

static std::vector<BrushSolid> BuildBrushSolids(const std::vector<MapPolygon>& polys) {
    std::vector<BrushSolid> solids;
    std::unordered_map<int, size_t> solidIndexByBrushId;
    for (const MapPolygon& poly : polys) {
        if (poly.sourceBrushId < 0 || poly.verts.empty() || Vector3LengthSq(poly.normal) <= 1e-8f) {
            continue;
        }
        auto [it, inserted] = solidIndexByBrushId.emplace(poly.sourceBrushId, solids.size());
        if (inserted) {
            BrushSolid solid;
            solid.sourceBrushId = poly.sourceBrushId;
            solids.push_back(std::move(solid));
        }
        BrushSolid& solid = solids[it->second];
        const Vector3 normal = Vector3Normalize(poly.normal);
        bool duplicatePlane = false;
        for (const BrushSolidPlane& existing : solid.planes) {
            if (Vector3DotProduct(existing.normal, normal) < 0.9999f) {
                continue;
            }
            if (fabsf(Vector3DotProduct(normal, Vector3Subtract(poly.verts[0], existing.point))) <= 1e-3f) {
                duplicatePlane = true;
                break;
            }
        }
        if (!duplicatePlane) {
            solid.planes.push_back({ poly.verts[0], normal });
        }
    }
    return solids;
}

static bool PointInsideBrushSolid(const BrushSolid& solid, const Vector3& point, float epsilon) {
    for (const BrushSolidPlane& plane : solid.planes) {
        const float planeDist = Vector3DotProduct(plane.normal, Vector3Subtract(point, plane.point));
        if (planeDist > epsilon) {
            return false;
        }
    }
    return !solid.planes.empty();
}

static bool PointInsideAnySolid(const std::vector<BrushSolid>& solids, const Vector3& point, float epsilon) {
    for (const BrushSolid& solid : solids) {
        if (PointInsideBrushSolid(solid, point, epsilon)) {
            return true;
        }
    }
    return false;
}

static float RayBrushSolidHitDistance(const BrushSolid& solid,
                                      const Vector3& rayOrigin,
                                      const Vector3& rayDirection,
                                      float minHitT,
                                      float maxHitT)
{
    if (solid.planes.empty() || maxHitT <= minHitT) {
        return maxHitT;
    }

    constexpr float kPlaneEpsilon = 0.01f;
    constexpr float kParallelEpsilon = 1e-6f;
    float enterT = minHitT;
    float exitT = maxHitT;

    // Convex brush slab test. Brush planes use outward normals; inside is
    // plane distance <= 0. A hit here blocks cracks between snapped faces.
    for (const BrushSolidPlane& plane : solid.planes) {
        const float dist = Vector3DotProduct(plane.normal, Vector3Subtract(rayOrigin, plane.point));
        const float denom = Vector3DotProduct(plane.normal, rayDirection);
        if (fabsf(denom) <= kParallelEpsilon) {
            if (dist > kPlaneEpsilon) {
                return maxHitT;
            }
            continue;
        }

        const float t = -dist / denom;
        if (denom < 0.0f) {
            enterT = std::max(enterT, t);
        } else {
            exitT = std::min(exitT, t);
        }

        if (enterT >= exitT) {
            return maxHitT;
        }
    }

    return (enterT > minHitT && enterT < maxHitT) ? enterT : maxHitT;
}

static float ClosestBrushSolidHitDistance(const std::vector<BrushSolid>& solids,
                                          const Vector3& rayOrigin,
                                          const Vector3& rayDirection,
                                          float minHitT,
                                          float maxHitT)
{
    float closest = maxHitT;
    for (const BrushSolid& solid : solids) {
        closest = std::min(closest, RayBrushSolidHitDistance(solid, rayOrigin, rayDirection, minHitT, closest));
    }
    return closest;
}

static SurfaceVisibilityInfo AnalyzeSurfaceVisibility(const std::vector<MapPolygon>& polys) {
    SurfaceVisibilityInfo info;
    info.hidden.assign(polys.size(), 0);
    info.emitterOnly.assign(polys.size(), 0);
    const std::vector<BrushSolid> solids = BuildBrushSolids(polys);
    auto polygonCentroid = [](const MapPolygon& poly) {
        Vector3 centroid = Vector3Zero();
        for (const Vector3& v : poly.verts) {
            centroid = Vector3Add(centroid, v);
        }
        return poly.verts.empty() ? centroid : Vector3Scale(centroid, 1.0f / (float)poly.verts.size());
    };

    auto pointInsidePolygonProjected = [](const MapPolygon& poly, const Vector3& point) {
        Vector3 axisU, axisV;
        FaceBasis(poly.normal, axisU, axisV);
        std::vector<Vector2> poly2d;
        poly2d.reserve(poly.verts.size());
        for (const Vector3& vert : poly.verts) {
            poly2d.push_back({
                Vector3DotProduct(vert, axisU),
                Vector3DotProduct(vert, axisV)
            });
        }
        return InsidePoly2D(poly2d,
                            Vector3DotProduct(point, axisU),
                            Vector3DotProduct(point, axisV));
    };

    auto projectPolygon = [](const MapPolygon& poly, const Vector3& axisU, const Vector3& axisV) {
        std::vector<Vector2> projected;
        projected.reserve(poly.verts.size());
        for (const Vector3& vert : poly.verts) {
            projected.push_back({
                Vector3DotProduct(vert, axisU),
                Vector3DotProduct(vert, axisV)
            });
        }
        return projected;
    };

    auto edgeMidpoint = [](const Vector3& a, const Vector3& b) {
        return Vector3Scale(Vector3Add(a, b), 0.5f);
    };

    for (size_t i = 0; i < polys.size(); ++i) {
        if (polys[i].occluderGroup >= 0) {
            info.emitterOnly[i] = 1;
        }
    }
    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& poly = polys[i];
        if (info.emitterOnly[i]) {
            continue;
        }
        const Vector3 centroid = polygonCentroid(poly);
        const Vector3 frontSample = Vector3Add(centroid, Vector3Scale(poly.normal, 0.5f));
        Vector3 axisU, axisV;
        FaceBasis(poly.normal, axisU, axisV);
        const float planeD = Vector3DotProduct(poly.verts[0], poly.normal);
        const Vector3 planeOrigin = Vector3Scale(poly.normal, planeD);
        const std::vector<Vector2> poly2d = projectPolygon(poly, axisU, axisV);

        std::vector<Vector3> samplePoints;
        samplePoints.reserve(poly.verts.size() * 2 + 80);
        samplePoints.push_back(centroid);
        for (size_t vi = 0; vi < poly.verts.size(); ++vi) {
            samplePoints.push_back(poly.verts[vi]);
            samplePoints.push_back(edgeMidpoint(poly.verts[vi], poly.verts[(vi + 1) % poly.verts.size()]));
        }

        float minU = 1e30f, maxU = -1e30f;
        float minV = 1e30f, maxV = -1e30f;
        for (const Vector2& p : poly2d) {
            minU = std::min(minU, p.x);
            maxU = std::max(maxU, p.x);
            minV = std::min(minV, p.y);
            maxV = std::max(maxV, p.y);
        }
        constexpr int kHiddenFaceSampleGrid = 8;
        constexpr float kHiddenFaceFrontOffset = 0.5f;
        constexpr float kHiddenFaceSolidEpsilon = 0.05f;
        for (int gy = 0; gy < kHiddenFaceSampleGrid; ++gy) {
            const float fy = (float(gy) + 0.5f) / float(kHiddenFaceSampleGrid);
            const float sv = minV + (maxV - minV) * fy;
            for (int gx = 0; gx < kHiddenFaceSampleGrid; ++gx) {
                const float fx = (float(gx) + 0.5f) / float(kHiddenFaceSampleGrid);
                const float su = minU + (maxU - minU) * fx;
                if (!InsidePoly2D(poly2d, su, sv)) {
                    continue;
                }
                samplePoints.push_back(Vector3Add(planeOrigin,
                                                  Vector3Add(Vector3Scale(axisU, su),
                                                             Vector3Scale(axisV, sv))));
            }
        }

        bool allInsideOtherSolid = (poly.sourceBrushId >= 0) && !samplePoints.empty() && !solids.empty();
        for (const Vector3& samplePoint : samplePoints) {
            const Vector3 sampleFront = Vector3Add(samplePoint, Vector3Scale(poly.normal, kHiddenFaceFrontOffset));
            bool insideOtherSolid = false;
            for (const BrushSolid& solid : solids) {
                if (solid.sourceBrushId == poly.sourceBrushId) {
                    continue;
                }
                if (PointInsideBrushSolid(solid, sampleFront, kHiddenFaceSolidEpsilon)) {
                    insideOtherSolid = true;
                    break;
                }
            }
            if (!insideOtherSolid) {
                allInsideOtherSolid = false;
                break;
            }
        }

        if (allInsideOtherSolid) {
            info.hidden[i] = 1;
            continue;
        }

        bool allCovered = !samplePoints.empty();
        for (const Vector3& samplePoint : samplePoints) {
            const Vector3 sampleFront = Vector3Add(samplePoint, Vector3Scale(poly.normal, kHiddenFaceFrontOffset));
            bool covered = false;
            for (size_t j = 0; j < polys.size(); ++j) {
                if (i == j) {
                    continue;
                }
                const MapPolygon& other = polys[j];
                if (Vector3DotProduct(poly.normal, other.normal) > -0.99f) {
                    continue;
                }

                const float samplePlaneDist = Vector3DotProduct(other.normal, Vector3Subtract(samplePoint, other.verts[0]));
                if (fabsf(samplePlaneDist) > 0.05f) {
                    continue;
                }

                const float frontPlaneDist = Vector3DotProduct(other.normal, Vector3Subtract(sampleFront, other.verts[0]));
                if (frontPlaneDist > -0.05f) {
                    continue;
                }

                const Vector3 projected = Vector3Subtract(sampleFront, Vector3Scale(other.normal, frontPlaneDist));
                if (pointInsidePolygonProjected(other, projected)) {
                    covered = true;
                    break;
                }
            }
            if (!covered) {
                allCovered = false;
                break;
            }
        }

        if (allCovered) {
            info.hidden[i] = 1;
            continue;
        }
    }

    for (size_t i = 0; i < polys.size(); ++i) {
        if (info.emitterOnly[i]) {
            ++info.emitterOnlyCount;
        }
        if (info.hidden[i]) {
            ++info.hiddenCount;
        }
    }
    return info;
}

static std::vector<MapPolygon> FilterVisibleBakePolygons(const std::vector<MapPolygon>& polys,
                                                         const SurfaceVisibilityInfo& visibility)
{
    std::vector<MapPolygon> visible;
    visible.reserve(polys.size());
    for (size_t i = 0; i < polys.size(); ++i) {
        if (i < visibility.hidden.size() && visibility.hidden[i]) {
            continue;
        }
        visible.push_back(polys[i]);
    }
    return visible;
}

static bool IsLightBrushTextureName(const std::string& name);
static bool PolygonUsesSurfaceLightTemplate(const MapPolygon& poly,
                                            const std::vector<SurfaceLightTemplate>& surfaceLights);

static uint32_t HashMaterialName(const std::string& name) {
    uint32_t hash = 2166136261u;
    for (unsigned char ch : name) {
        hash ^= (uint32_t)ch;
        hash *= 16777619u;
    }
    return hash;
}

static std::string LowercaseTextureName(const std::string& texture) {
    std::string lowered = texture;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
        return (char)std::tolower(ch);
    });
    return lowered;
}

static std::string TextureBaseNameLower(const std::string& texture) {
    const std::string lowered = LowercaseTextureName(texture);
    const size_t slash = lowered.find_last_of('/');
    return (slash == std::string::npos) ? lowered : lowered.substr(slash + 1);
}

static bool TextureNameLooksLikeSky(const std::string& texture) {
    if (texture.empty()) {
        return false;
    }
    const std::string lowered = LowercaseTextureName(texture);
    return lowered == "sky"
        || lowered.rfind("sky_", 0) == 0
        || lowered.find("/sky") != std::string::npos
        || lowered.find("sky/") != std::string::npos;
}

static bool TextureNameLooksLikeNonShadowCaster(const std::string& texture) {
    if (texture.empty()) {
        return false;
    }
    const std::string lowered = LowercaseTextureName(texture);
    if (!lowered.empty() && lowered[0] == '*') {
        return true;
    }

    const std::string base = TextureBaseNameLower(texture);
    return base == "skip"
        || base == "hint"
        || base == "clip"
        || base == "playerclip"
        || base == "monsterclip"
        || base == "fullclip"
        || base == "weapclip"
        || base == "origin"
        || base == "trigger"
        || base == "nodraw"
        || base == "caulk"
        || base == "null"
        || base == "water"
        || base == "lava"
        || base == "slime"
        || base == "teleport"
        || base == "areaportal";
}

static bool PolygonCastsShadowForLighting(const MapPolygon& poly) {
    if (poly.occluderGroup >= 0) {
        return false;
    }
    if (TextureNameLooksLikeSky(poly.texture)) {
        return false;
    }
    if (TextureNameLooksLikeNonShadowCaster(poly.texture)) {
        return false;
    }
    return true;
}

static bool PolygonCanEmitBounceForLighting(const MapPolygon& poly,
                                            const std::vector<SurfaceLightTemplate>& surfaceLights)
{
    if (!PolygonCastsShadowForLighting(poly)) {
        return false;
    }
    if (poly.noBounce != 0) {
        return false;
    }
    if (IsLightBrushTextureName(poly.texture)) {
        return false;
    }
    if (PolygonUsesSurfaceLightTemplate(poly, surfaceLights)) {
        return false;
    }
    return true;
}

static OccluderSet BuildOccluders(const std::vector<MapPolygon>& polys) {
    OccluderSet o;
    int nonShadowCasterCount = 0;
    int emitterOnlyCount = 0;
    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& p = polys[i];
        if (p.occluderGroup >= 0) {
            ++emitterOnlyCount;
            continue;
        }
        if (!PolygonCastsShadowForLighting(p)) {
            ++nonShadowCasterCount;
            continue;
        }
        for (size_t t = 1; t + 1 < p.verts.size(); ++t) {
            LightmapTraceTri tr{};
            tr.a = p.verts[0];
            tr.b = p.verts[t];
            tr.c = p.verts[t + 1];
            tr.bounds = AABBInvalid();
            tr.occluderGroup = p.occluderGroup;
            tr.sourcePolyIndex = (int)i;
            tr.flags = TextureNameLooksLikeSky(p.texture) ? LIGHTMAP_TRACE_TRI_SKY : LIGHTMAP_TRACE_TRI_NONE;
            tr.materialId = (int)(HashMaterialName(p.texture) & 0x7fffffffu);
            AABBExtend(&tr.bounds, tr.a);
            AABBExtend(&tr.bounds, tr.b);
            AABBExtend(&tr.bounds, tr.c);
            o.tris.push_back(tr);
        }
    }
    if (emitterOnlyCount > 0) {
        printf("[Lightmap] skipped %d light brush faces from occluders.\n", emitterOnlyCount);
    }
    if (nonShadowCasterCount > 0) {
        printf("[Lightmap] skipped %d non-shadow-casting utility faces from occluders.\n", nonShadowCasterCount);
    }
    LightmapTraceBuildAcceleration(&o);
    return o;
}

static void BuildComputeBrushSolids(const std::vector<BrushSolid>& solids,
                                    std::vector<LightmapComputeBrushSolid>* outSolids,
                                    std::vector<LightmapComputeSolidPlane>* outPlanes)
{
    if (!outSolids || !outPlanes) {
        return;
    }

    outSolids->clear();
    outPlanes->clear();
    outSolids->reserve(solids.size());
    for (const BrushSolid& solid : solids) {
        LightmapComputeBrushSolid gpuSolid{};
        gpuSolid.firstPlane = (int)outPlanes->size();
        gpuSolid.planeCount = (int)solid.planes.size();
        outSolids->push_back(gpuSolid);
        for (const BrushSolidPlane& plane : solid.planes) {
            LightmapComputeSolidPlane gpuPlane{};
            gpuPlane.point = plane.point;
            gpuPlane.normal = plane.normal;
            outPlanes->push_back(gpuPlane);
        }
    }
}

static void BuildComputeRepairSourceGraph(const std::vector<RepairSourcePoly>& repairPolys,
                                          std::vector<LightmapComputeRepairSourcePoly>* outPolys,
                                          std::vector<LightmapComputeRepairSourceNeighbor>* outNeighbors)
{
    if (!outPolys || !outNeighbors) {
        return;
    }

    outPolys->clear();
    outNeighbors->clear();
    outPolys->reserve(repairPolys.size());
    for (const RepairSourcePoly& repairPoly : repairPolys) {
        LightmapComputeRepairSourcePoly gpuPoly{};
        gpuPoly.planePoint = repairPoly.planePoint;
        gpuPoly.axisU = repairPoly.axisU;
        gpuPoly.axisV = repairPoly.axisV;
        gpuPoly.normal = repairPoly.normal;
        gpuPoly.polyCount = (int)std::min(repairPoly.poly2d.size(), (size_t)LIGHTMAP_COMPUTE_MAX_POLY_VERTS);
        gpuPoly.firstNeighbor = (int)outNeighbors->size();
        gpuPoly.neighborCount = (int)repairPoly.neighbors.size();
        for (int i = 0; i < gpuPoly.polyCount; ++i) {
            gpuPoly.polyVerts[i][0] = repairPoly.poly2d[(size_t)i].x;
            gpuPoly.polyVerts[i][1] = repairPoly.poly2d[(size_t)i].y;
            gpuPoly.polyVerts[i][2] = 0.0f;
            gpuPoly.polyVerts[i][3] = 0.0f;
        }
        outPolys->push_back(gpuPoly);

        for (const RepairSourceNeighbor& neighbor : repairPoly.neighbors) {
            LightmapComputeRepairSourceNeighbor gpuNeighbor{};
            gpuNeighbor.sourcePolyIndex = (int)neighbor.sourcePolyIndex;
            gpuNeighbor.edgeIndex = neighbor.edgeIndex;
            outNeighbors->push_back(gpuNeighbor);
        }
    }
}

static void BuildComputePhongGraph(const std::vector<PhongSourcePoly>& sourcePhongs,
                                   std::vector<LightmapComputePhongSourcePoly>* outPolys,
                                   std::vector<LightmapComputePhongNeighbor>* outNeighbors)
{
    if (!outPolys || !outNeighbors) {
        return;
    }

    outPolys->clear();
    outNeighbors->clear();
    outPolys->reserve(sourcePhongs.size());
    for (const PhongSourcePoly& sourcePoly : sourcePhongs) {
        LightmapComputePhongSourcePoly gpuPoly{};
        gpuPoly.normal = sourcePoly.normal;
        gpuPoly.areaWeight = sourcePoly.areaWeight;
        gpuPoly.enabled = sourcePoly.enabled ? 1 : 0;
        gpuPoly.firstNeighbor = (int)outNeighbors->size();
        gpuPoly.neighborCount = (int)sourcePoly.neighbors.size();
        outPolys->push_back(gpuPoly);

        for (const PhongNeighbor& neighbor : sourcePoly.neighbors) {
            LightmapComputePhongNeighbor gpuNeighbor{};
            gpuNeighbor.sourcePolyIndex = (int)neighbor.sourcePolyIndex;
            gpuNeighbor.edgeA = neighbor.edgeA;
            gpuNeighbor.edgeB = neighbor.edgeB;
            gpuNeighbor.normal = neighbor.normal;
            gpuNeighbor.areaWeight = neighbor.areaWeight;
            outNeighbors->push_back(gpuNeighbor);
        }
    }
}

static void BuildComputeSurfaceEmitterPayload(
    const std::vector<SurfaceLightEmitter>& surfaceEmitters,
    std::vector<LightmapComputeSurfaceEmitter>* outEmitters,
    std::vector<LightmapComputeSurfaceEmitterSample>* outSamples)
{
    if (!outEmitters || !outSamples) {
        return;
    }

    outEmitters->clear();
    outSamples->clear();
    outEmitters->reserve(surfaceEmitters.size());

    size_t totalSampleCount = 0;
    for (const SurfaceLightEmitter& emitter : surfaceEmitters) {
        totalSampleCount += emitter.samplePoints.size();
    }
    outSamples->reserve(totalSampleCount);

    for (const SurfaceLightEmitter& emitter : surfaceEmitters) {
        LightmapComputeSurfaceEmitter gpuEmitter{};
        gpuEmitter.baseLight = emitter.baseLight;
        gpuEmitter.surfaceNormal = emitter.surfaceNormal;
        gpuEmitter.sampleIntensityScale = emitter.sampleIntensityScale;
        gpuEmitter.attenuationScale = emitter.attenuationScale;
        gpuEmitter.transportScale = emitter.transportScale;
        gpuEmitter.hotspotClamp = emitter.hotspotClamp;
        gpuEmitter.firstSamplePoint = (int)outSamples->size();
        gpuEmitter.samplePointCount = (int)emitter.samplePoints.size();
        gpuEmitter.omnidirectional = emitter.omnidirectional;
        gpuEmitter.rescale = emitter.rescale;
        outEmitters->push_back(gpuEmitter);

        for (const Vector3& samplePoint : emitter.samplePoints) {
            LightmapComputeSurfaceEmitterSample gpuSample{};
            gpuSample.point = samplePoint;
            outSamples->push_back(gpuSample);
        }
    }
}

// --------------------------------------------------------------------------
//  Point-in-convex-polygon test (2-D, polygon already CCW in plane space).
// --------------------------------------------------------------------------
static bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py) {
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const float ex = poly[j].x - poly[i].x;
        const float ey = poly[j].y - poly[i].y;
        const float cx = px - poly[i].x;
        const float cy = py - poly[i].y;
        if (ex * cy - ey * cx < -1e-3f) return false;
    }
    return true;
}

static float DistSqPointSegment2D(const Vector2& p, const Vector2& a, const Vector2& b) {
    const Vector2 ab = { b.x - a.x, b.y - a.y };
    const Vector2 ap = { p.x - a.x, p.y - a.y };
    const float abLenSq = ab.x * ab.x + ab.y * ab.y;
    if (abLenSq <= 1e-8f) {
        const float dx = p.x - a.x;
        const float dy = p.y - a.y;
        return dx * dx + dy * dy;
    }
    const float t = std::clamp((ap.x * ab.x + ap.y * ab.y) / abLenSq, 0.0f, 1.0f);
    const float qx = a.x + ab.x * t;
    const float qy = a.y + ab.y * t;
    const float dx = p.x - qx;
    const float dy = p.y - qy;
    return dx * dx + dy * dy;
}

static Vector2 ClosestPointOnPolyBoundary2D(const std::vector<Vector2>& poly, const Vector2& p) {
    if (poly.size() < 2) {
        return p;
    }

    float bestDistSq = FLT_MAX;
    Vector2 bestPoint = p;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector2& a = poly[i];
        const Vector2& b = poly[(i + 1) % poly.size()];
        const Vector2 ab = { b.x - a.x, b.y - a.y };
        const float abLenSq = ab.x * ab.x + ab.y * ab.y;
        float t = 0.0f;
        if (abLenSq > 1e-8f) {
            t = std::clamp(((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / abLenSq, 0.0f, 1.0f);
        }
        const Vector2 q = { a.x + ab.x * t, a.y + ab.y * t };
        const float dx = p.x - q.x;
        const float dy = p.y - q.y;
        const float distSq = dx * dx + dy * dy;
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestPoint = q;
        }
    }

    return bestPoint;
}

static int FindBestRepairEdgeIndex(const RepairSourcePoly& poly, const Vector2& pointUv) {
    if (poly.poly2d.size() < 2) {
        return -1;
    }

    int bestEdgeIndex = -1;
    float bestEdgeDist = FLT_MAX;
    for (size_t edgeIndex = 0; edgeIndex < poly.poly2d.size(); ++edgeIndex) {
        const Vector2& a = poly.poly2d[edgeIndex];
        const Vector2& b = poly.poly2d[(edgeIndex + 1) % poly.poly2d.size()];
        const Vector2 edge = { b.x - a.x, b.y - a.y };
        const Vector2 rel = { pointUv.x - a.x, pointUv.y - a.y };
        if (edge.x * rel.y - edge.y * rel.x >= -1e-3f) {
            continue;
        }

        const float edgeLen = sqrtf(std::max(1e-8f, edge.x * edge.x + edge.y * edge.y));
        const float edgeLenSq = edgeLen * edgeLen;
        const float t = edgeLenSq > 1e-8f
            ? ((pointUv.x - a.x) * edge.x + (pointUv.y - a.y) * edge.y) / edgeLenSq
            : 0.0f;
        float edgeDist = 0.0f;
        if (t < 0.0f) {
            edgeDist = fabsf(t) * edgeLen;
        } else if (t > 1.0f) {
            edgeDist = (t - 1.0f) * edgeLen;
        }

        if (edgeDist < bestEdgeDist) {
            bestEdgeDist = edgeDist;
            bestEdgeIndex = (int)edgeIndex;
        }
    }

    return bestEdgeIndex;
}

static float MinDistToPolyEdge2D(const std::vector<Vector2>& poly, float px, float py) {
    if (poly.size() < 2) {
        return 0.0f;
    }
    const Vector2 p = { px, py };
    float minDistSq = FLT_MAX;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector2& a = poly[i];
        const Vector2& b = poly[(i + 1) % poly.size()];
        minDistSq = std::min(minDistSq, DistSqPointSegment2D(p, a, b));
    }
    return sqrtf(minDistSq);
}

static Vector3 ProjectPointOntoPlane(const Vector3& point,
                                     const Vector3& planePoint,
                                     const Vector3& planeNormal)
{
    const Vector3 normal = Vector3Normalize(planeNormal);
    const float planeDist = Vector3DotProduct(normal, Vector3Subtract(point, planePoint));
    return Vector3Subtract(point, Vector3Scale(normal, planeDist));
}

static bool PointInsidePolygonProjected(const std::vector<Vector2>& poly2d,
                                        const Vector3& point,
                                        const Vector3& axisU,
                                        const Vector3& axisV)
{
    return InsidePoly2D(poly2d,
                        Vector3DotProduct(point, axisU),
                        Vector3DotProduct(point, axisV));
}

static Vector3 ClosestPointOnPolyBoundaryProjected(const std::vector<Vector2>& poly2d,
                                                   const Vector3& planePoint,
                                                   const Vector3& axisU,
                                                   const Vector3& axisV)
{
    if (poly2d.size() < 2) {
        return planePoint;
    }

    const Vector2 point2d = {
        Vector3DotProduct(planePoint, axisU),
        Vector3DotProduct(planePoint, axisV)
    };
    const Vector2 bestPoint = ClosestPointOnPolyBoundary2D(poly2d, point2d);

    const float currentU = Vector3DotProduct(planePoint, axisU);
    const float currentV = Vector3DotProduct(planePoint, axisV);
    return Vector3Add(planePoint,
                      Vector3Add(Vector3Scale(axisU, bestPoint.x - currentU),
                                 Vector3Scale(axisV, bestPoint.y - currentV)));
}

struct FaceRect {
    LightmapComputeFaceRect gpu;
    std::vector<Vector2> poly2d;
    AABB bounds{};
    std::vector<uint32_t> lightIndices;
    std::vector<uint32_t> surfaceEmitterIndices;
    uint32_t page = 0;
    uint32_t sourcePolyIndex = 0;
    float maxU = 0.0f;
    float maxV = 0.0f;
    bool computeCompatible = true;
    std::string computeFallbackReason;
};

static void BuildComputeRectSurfaceEmitterDispatchData(
    const std::vector<FaceRect>& rects,
    const std::vector<size_t>& rectIndices,
    std::vector<LightmapComputeRectSurfaceEmitterRange>* outRanges,
    std::vector<uint32_t>* outFlattenedEmitterIndices,
    const std::vector<std::vector<uint32_t>>* emitterIndicesByRect = nullptr)
{
    if (!outRanges || !outFlattenedEmitterIndices) {
        return;
    }

    outRanges->clear();
    outFlattenedEmitterIndices->clear();
    outRanges->reserve(rectIndices.size());

    size_t totalEmitterRefs = 0;
    for (size_t rectIndex : rectIndices) {
        if (rectIndex >= rects.size()) {
            continue;
        }
        const std::vector<uint32_t>& emitterIndices = emitterIndicesByRect
            ? (*emitterIndicesByRect)[rectIndex]
            : rects[rectIndex].surfaceEmitterIndices;
        totalEmitterRefs += emitterIndices.size();
    }
    outFlattenedEmitterIndices->reserve(totalEmitterRefs);

    for (size_t rectIndex : rectIndices) {
        LightmapComputeRectSurfaceEmitterRange range{};
        range.firstEmitterIndex = (int)outFlattenedEmitterIndices->size();
        if (rectIndex < rects.size()) {
            const std::vector<uint32_t>& emitterIndices = emitterIndicesByRect
                ? (*emitterIndicesByRect)[rectIndex]
                : rects[rectIndex].surfaceEmitterIndices;
            range.emitterCount = (int)emitterIndices.size();
            outFlattenedEmitterIndices->insert(outFlattenedEmitterIndices->end(),
                                               emitterIndices.begin(),
                                               emitterIndices.end());
        }
        outRanges->push_back(range);
    }
}

struct LightmapPageLayout {
    int cursorX = 0;
    int cursorY = 0;
    int rowH = 0;
    int usedHeight = 0;
};

static float DistSqPointAABB(const Vector3& p, const AABB& bounds) {
    float dx = 0.0f;
    if (p.x < bounds.min.x) dx = bounds.min.x - p.x;
    else if (p.x > bounds.max.x) dx = p.x - bounds.max.x;

    float dy = 0.0f;
    if (p.y < bounds.min.y) dy = bounds.min.y - p.y;
    else if (p.y > bounds.max.y) dy = p.y - bounds.max.y;

    float dz = 0.0f;
    if (p.z < bounds.min.z) dz = bounds.min.z - p.z;
    else if (p.z > bounds.max.z) dz = p.z - bounds.max.z;

    return dx * dx + dy * dy + dz * dz;
}

static float DistSqAABBAABB(const AABB& a, const AABB& b) {
    float dx = 0.0f;
    if (a.max.x < b.min.x) dx = b.min.x - a.max.x;
    else if (b.max.x < a.min.x) dx = a.min.x - b.max.x;

    float dy = 0.0f;
    if (a.max.y < b.min.y) dy = b.min.y - a.max.y;
    else if (b.max.y < a.min.y) dy = a.min.y - b.max.y;

    float dz = 0.0f;
    if (a.max.z < b.min.z) dz = b.min.z - a.max.z;
    else if (b.max.z < a.min.z) dz = a.min.z - b.max.z;

    return dx * dx + dy * dy + dz * dz;
}

static std::vector<FaceRect> BuildFaceRects(const std::vector<BakePatch>& patches,
                                            const std::vector<PointLight>& lights,
                                            const std::vector<RepairSourcePoly>& repairPolys,
                                            const std::vector<PhongSourcePoly>& sourcePhongs,
                                            float luxelSize) {
    std::vector<FaceRect> rects(patches.size());
    const float safeLuxelSize = std::max(0.125f, luxelSize);
    for (size_t i = 0; i < patches.size(); ++i) {
        const MapPolygon& p = patches[i].poly;
        Vector3 U, V;
        FaceBasis(p.normal, U, V);
        float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
        FaceRect& r = rects[i];
        r.bounds = AABBInvalid();
        for (const Vector3& vv : p.verts) {
            const float u = Vector3DotProduct(vv, U);
            const float v = Vector3DotProduct(vv, V);
            minU = std::min(minU, u); maxU = std::max(maxU, u);
            minV = std::min(minV, v); maxV = std::max(maxV, v);
            AABBExtend(&r.bounds, vv);
        }

        const int interiorW = ComputeInteriorLuxelSpan(maxU - minU, safeLuxelSize);
        const int interiorH = ComputeInteriorLuxelSpan(maxV - minV, safeLuxelSize);
        r.gpu.w = interiorW + LM_PAD * 2;
        r.gpu.h = interiorH + LM_PAD * 2;
        r.gpu.luxelSize = safeLuxelSize;
        r.gpu.minU = minU;
        r.gpu.minV = minV;
        r.gpu.axisU = U;
        r.gpu.axisV = V;
        r.gpu.normal = p.normal;
        r.sourcePolyIndex = patches[i].sourcePolyIndex;
        r.gpu.sourcePolyIndex = (int)r.sourcePolyIndex;
        r.maxU = maxU;
        r.maxV = maxV;
        if (r.sourcePolyIndex >= repairPolys.size()) {
            r.computeCompatible = false;
            r.computeFallbackReason = "page references an invalid repair source polygon";
        } else if (repairPolys[r.sourcePolyIndex].poly2d.size() > LIGHTMAP_COMPUTE_MAX_POLY_VERTS) {
            r.computeCompatible = false;
            r.computeFallbackReason = "page contains source polygon(s) exceeding compute repair vertex limit";
        }
        if (r.sourcePolyIndex < sourcePhongs.size()) {
            const PhongSourcePoly& source = sourcePhongs[r.sourcePolyIndex];
            r.gpu.phongBaseNormal = source.normal;
            r.gpu.phongBaseAreaWeight = source.areaWeight;
            if (source.neighbors.size() > LIGHTMAP_COMPUTE_MAX_PHONG_NEIGHBORS) {
                r.computeCompatible = false;
                r.computeFallbackReason = "page uses phong smoothing with too many neighboring faces for the compute baker";
            } else {
                r.gpu.phongNeighborCount = (int)source.neighbors.size();
                for (int neighborIndex = 0; neighborIndex < r.gpu.phongNeighborCount; ++neighborIndex) {
                    const PhongNeighbor& neighbor = source.neighbors[(size_t)neighborIndex];
                    r.gpu.phongNeighborEdgeA[neighborIndex][0] = neighbor.edgeA.x;
                    r.gpu.phongNeighborEdgeA[neighborIndex][1] = neighbor.edgeA.y;
                    r.gpu.phongNeighborEdgeA[neighborIndex][2] = neighbor.edgeA.z;
                    r.gpu.phongNeighborEdgeA[neighborIndex][3] = 0.0f;
                    r.gpu.phongNeighborEdgeB[neighborIndex][0] = neighbor.edgeB.x;
                    r.gpu.phongNeighborEdgeB[neighborIndex][1] = neighbor.edgeB.y;
                    r.gpu.phongNeighborEdgeB[neighborIndex][2] = neighbor.edgeB.z;
                    r.gpu.phongNeighborEdgeB[neighborIndex][3] = 0.0f;
                    r.gpu.phongNeighborNormalWeight[neighborIndex][0] = neighbor.normal.x;
                    r.gpu.phongNeighborNormalWeight[neighborIndex][1] = neighbor.normal.y;
                    r.gpu.phongNeighborNormalWeight[neighborIndex][2] = neighbor.normal.z;
                    r.gpu.phongNeighborNormalWeight[neighborIndex][3] = neighbor.areaWeight;
                }
            }
        }
        const float d = Vector3DotProduct(p.verts[0], p.normal);
        r.gpu.origin = Vector3Add(
            Vector3Scale(p.normal, d),
            Vector3Add(Vector3Scale(U, minU), Vector3Scale(V, minV)));
        r.poly2d.reserve(p.verts.size());
        for (const Vector3& vv : p.verts) {
            r.poly2d.push_back({
                (Vector3DotProduct(vv, U) - minU) / safeLuxelSize,
                (Vector3DotProduct(vv, V) - minV) / safeLuxelSize
            });
        }
        if (r.poly2d.size() > LIGHTMAP_COMPUTE_MAX_POLY_VERTS) {
            r.computeCompatible = false;
            r.computeFallbackReason = "page contains polygon(s) exceeding compute vertex limit";
            printf("[Lightmap] compute polygon vertex limit exceeded (%zu > %d), page will use CPU fallback.\n",
                   r.poly2d.size(), LIGHTMAP_COMPUTE_MAX_POLY_VERTS);
        } else {
            r.gpu.polyCount = (int)r.poly2d.size();
            for (int pi = 0; pi < r.gpu.polyCount; ++pi) {
                r.gpu.polyVerts[pi][0] = r.poly2d[(size_t)pi].x;
                r.gpu.polyVerts[pi][1] = r.poly2d[(size_t)pi].y;
                r.gpu.polyVerts[pi][2] = 0.0f;
                r.gpu.polyVerts[pi][3] = 0.0f;
            }
        }
        r.lightIndices.reserve(lights.size());
        for (uint32_t lightIndex = 0; lightIndex < lights.size(); ++lightIndex) {
            const PointLight& light = lights[lightIndex];
            if (IsParallelLight(light)) {
                r.lightIndices.push_back(lightIndex);
                continue;
            }
            const float radiusSq = light.intensity * light.intensity;
            if (DistSqPointAABB(light.position, r.bounds) <= radiusSq) {
                r.lightIndices.push_back(lightIndex);
            }
        }

        if (interiorW > FACE_MAX_LUXELS || interiorH > FACE_MAX_LUXELS) {
            printf("[Lightmap] warning: patch %zu exceeded face luxel cap after subdivision (%dx%d > %d).\n",
                   i, interiorW, interiorH, FACE_MAX_LUXELS);
        }
    }
    return rects;
}

static bool CoverageContains(const std::vector<uint8_t>& coverage,
                             const LightmapPage& page,
                             int x, int y)
{
    if (x < 0 || y < 0 || x >= page.width || y >= page.height) {
        return false;
    }
    const size_t idx = (size_t)y * (size_t)page.width + (size_t)x;
    return idx < coverage.size() && coverage[idx] > 0;
}

static int InteriorSpanX(const FaceRect& rect) {
    return std::max(0, rect.gpu.w - LM_PAD * 2);
}

static int InteriorSpanY(const FaceRect& rect) {
    return std::max(0, rect.gpu.h - LM_PAD * 2);
}

static bool LocalInteriorIndexFromGlobalCenter(float rectMinCoord,
                                               float luxelSize,
                                               int interiorSpan,
                                               float globalCenter,
                                               int* outInteriorIndex)
{
    if (!outInteriorIndex || interiorSpan <= 0 || luxelSize <= 0.0f) {
        return false;
    }

    const float local = (globalCenter - rectMinCoord) / luxelSize;
    const int candidate = (int)floorf(local);
    if (candidate < 0 || candidate >= interiorSpan) {
        return false;
    }
    const float expectedCenter = (float)candidate + 0.5f;
    if (fabsf(local - expectedCenter) > 1e-3f) {
        return false;
    }

    *outInteriorIndex = candidate;
    return true;
}

static void AverageLuxelPair(LightmapPage& aPage, int ax, int ay,
                             LightmapPage& bPage, int bx, int by)
{
    const size_t aOff = ((size_t)ay * (size_t)aPage.width + (size_t)ax) * 4;
    const size_t bOff = ((size_t)by * (size_t)bPage.width + (size_t)bx) * 4;
    if (aOff + 3 >= aPage.pixels.size() || bOff + 3 >= bPage.pixels.size()) {
        return;
    }

    for (int c = 0; c < 3; ++c) {
        const float avg = (aPage.pixels[aOff + c] + bPage.pixels[bOff + c]) * 0.5f;
        aPage.pixels[aOff + c] = avg;
        bPage.pixels[bOff + c] = avg;
    }
    aPage.pixels[aOff + 3] = 1.0f;
    bPage.pixels[bOff + 3] = 1.0f;
}

static size_t WeldVerticalSiblingSeam(const FaceRect& left,
                                      const FaceRect& right,
                                      std::vector<LightmapPage>& pages,
                                      const std::vector<std::vector<uint8_t>>& coverageMasks)
{
    if (left.page >= pages.size() || right.page >= pages.size() ||
        left.page >= coverageMasks.size() || right.page >= coverageMasks.size()) {
        return 0;
    }

    const int leftInteriorW = InteriorSpanX(left);
    const int leftInteriorH = InteriorSpanY(left);
    const int rightInteriorH = InteriorSpanY(right);
    if (leftInteriorW <= 0 || leftInteriorH <= 0 || rightInteriorH <= 0) {
        return 0;
    }

    LightmapPage& leftPage = pages[left.page];
    LightmapPage& rightPage = pages[right.page];
    const std::vector<uint8_t>& leftCoverage = coverageMasks[left.page];
    const std::vector<uint8_t>& rightCoverage = coverageMasks[right.page];
    const int leftX = left.gpu.x + LM_PAD + leftInteriorW - 1;
    const int rightX = right.gpu.x + LM_PAD;

    size_t welded = 0;
    for (int leftRow = 0; leftRow < leftInteriorH; ++leftRow) {
        const float globalVCenter = left.gpu.minV + ((float)leftRow + 0.5f) * left.gpu.luxelSize;
        int rightRow = 0;
        if (!LocalInteriorIndexFromGlobalCenter(right.gpu.minV, right.gpu.luxelSize, rightInteriorH, globalVCenter, &rightRow)) {
            continue;
        }

        const int leftY = left.gpu.y + LM_PAD + leftRow;
        const int rightY = right.gpu.y + LM_PAD + rightRow;
        if (!CoverageContains(leftCoverage, leftPage, leftX, leftY) ||
            !CoverageContains(rightCoverage, rightPage, rightX, rightY)) {
            continue;
        }

        AverageLuxelPair(leftPage, leftX, leftY, rightPage, rightX, rightY);
        ++welded;
    }

    return welded;
}

static size_t WeldHorizontalSiblingSeam(const FaceRect& bottom,
                                        const FaceRect& top,
                                        std::vector<LightmapPage>& pages,
                                        const std::vector<std::vector<uint8_t>>& coverageMasks)
{
    if (bottom.page >= pages.size() || top.page >= pages.size() ||
        bottom.page >= coverageMasks.size() || top.page >= coverageMasks.size()) {
        return 0;
    }

    const int bottomInteriorW = InteriorSpanX(bottom);
    const int bottomInteriorH = InteriorSpanY(bottom);
    const int topInteriorW = InteriorSpanX(top);
    if (bottomInteriorW <= 0 || bottomInteriorH <= 0 || topInteriorW <= 0) {
        return 0;
    }

    LightmapPage& bottomPage = pages[bottom.page];
    LightmapPage& topPage = pages[top.page];
    const std::vector<uint8_t>& bottomCoverage = coverageMasks[bottom.page];
    const std::vector<uint8_t>& topCoverage = coverageMasks[top.page];
    const int bottomY = bottom.gpu.y + LM_PAD + bottomInteriorH - 1;
    const int topY = top.gpu.y + LM_PAD;

    size_t welded = 0;
    for (int bottomCol = 0; bottomCol < bottomInteriorW; ++bottomCol) {
        const float globalUCenter = bottom.gpu.minU + ((float)bottomCol + 0.5f) * bottom.gpu.luxelSize;
        int topCol = 0;
        if (!LocalInteriorIndexFromGlobalCenter(top.gpu.minU, top.gpu.luxelSize, topInteriorW, globalUCenter, &topCol)) {
            continue;
        }

        const int bottomX = bottom.gpu.x + LM_PAD + bottomCol;
        const int topX = top.gpu.x + LM_PAD + topCol;
        if (!CoverageContains(bottomCoverage, bottomPage, bottomX, bottomY) ||
            !CoverageContains(topCoverage, topPage, topX, topY)) {
            continue;
        }

        AverageLuxelPair(bottomPage, bottomX, bottomY, topPage, topX, topY);
        ++welded;
    }

    return welded;
}

static size_t WeldSiblingPatchSeams(const std::vector<FaceRect>& rects,
                                    std::vector<LightmapPage>& pages,
                                    const std::vector<std::vector<uint8_t>>& coverageMasks,
                                    float luxelSize)
{
    const float kSeamCoordEpsilon = std::max(0.125f, luxelSize) * 0.05f;

    std::unordered_map<uint32_t, std::vector<size_t>> rectsBySourcePoly;
    rectsBySourcePoly.reserve(rects.size());
    for (size_t i = 0; i < rects.size(); ++i) {
        rectsBySourcePoly[rects[i].sourcePolyIndex].push_back(i);
    }

    size_t weldedSamples = 0;
    for (const auto& [sourcePolyIndex, group] : rectsBySourcePoly) {
        (void)sourcePolyIndex;
        if (group.size() < 2) {
            continue;
        }

        for (size_t i = 0; i < group.size(); ++i) {
            const FaceRect& a = rects[group[i]];
            for (size_t j = i + 1; j < group.size(); ++j) {
                const FaceRect& b = rects[group[j]];

                if (fabsf(a.maxU - b.gpu.minU) <= kSeamCoordEpsilon) {
                    weldedSamples += WeldVerticalSiblingSeam(a, b, pages, coverageMasks);
                } else if (fabsf(b.maxU - a.gpu.minU) <= kSeamCoordEpsilon) {
                    weldedSamples += WeldVerticalSiblingSeam(b, a, pages, coverageMasks);
                }

                if (fabsf(a.maxV - b.gpu.minV) <= kSeamCoordEpsilon) {
                    weldedSamples += WeldHorizontalSiblingSeam(a, b, pages, coverageMasks);
                } else if (fabsf(b.maxV - a.gpu.minV) <= kSeamCoordEpsilon) {
                    weldedSamples += WeldHorizontalSiblingSeam(b, a, pages, coverageMasks);
                }
            }
        }
    }

    return weldedSamples;
}

static bool PlaceRectOnPage(FaceRect& rect, LightmapPageLayout& page) {
    if (rect.gpu.w > LIGHTMAP_PAGE_SIZE || rect.gpu.h > LIGHTMAP_PAGE_SIZE) {
        return false;
    }

    int x = page.cursorX;
    int y = page.cursorY;
    int rowH = page.rowH;

    if (x + rect.gpu.w > LIGHTMAP_PAGE_SIZE) {
        y += rowH;
        x = 0;
        rowH = 0;
    }

    if (y + rect.gpu.h > LIGHTMAP_PAGE_SIZE) {
        return false;
    }

    rect.gpu.x = x;
    rect.gpu.y = y;

    page.cursorX = x + rect.gpu.w;
    page.cursorY = y;
    page.rowH = std::max(rowH, rect.gpu.h);
    page.usedHeight = std::max(page.usedHeight, page.cursorY + page.rowH);
    return true;
}

static std::vector<LightmapPageLayout> PackLightmapPages(std::vector<FaceRect>& rects) {
    std::vector<size_t> order(rects.size());
    for (size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return rects[a].gpu.h > rects[b].gpu.h;
    });

    std::vector<LightmapPageLayout> pages;
    for (size_t idx : order) {
        bool placed = false;
        for (uint32_t pageIndex = 0; pageIndex < pages.size(); ++pageIndex) {
            FaceRect candidate = rects[idx];
            LightmapPageLayout trial = pages[pageIndex];
            if (!PlaceRectOnPage(candidate, trial)) {
                continue;
            }
            rects[idx] = candidate;
            rects[idx].page = pageIndex;
            pages[pageIndex] = trial;
            placed = true;
            break;
        }

        if (!placed) {
            LightmapPageLayout page;
            if (!PlaceRectOnPage(rects[idx], page)) {
                printf("[Lightmap] face rect %zux%zu exceeds page size %d.\n",
                       (size_t)rects[idx].gpu.w, (size_t)rects[idx].gpu.h, LIGHTMAP_PAGE_SIZE);
                return {};
            }
            rects[idx].page = (uint32_t)pages.size();
            pages.push_back(page);
        }
    }

    for (LightmapPageLayout& page : pages) {
        page.usedHeight = std::max(1, page.usedHeight);
    }
    return pages;
}

static void FillPatchUVs(const std::vector<BakePatch>& patches,
                         const std::vector<FaceRect>& rects,
                         const std::vector<LightmapPage>& pages,
                         LightmapAtlas& atlas)
{
    atlas.patches.resize(patches.size());
    for (size_t i = 0; i < patches.size(); ++i) {
        const MapPolygon& p = patches[i].poly;
        const FaceRect& r = rects[i];
        const LightmapPage& page = pages[r.page];
        atlas.patches[i].poly = p;
        atlas.patches[i].page = r.page;
        atlas.patches[i].sourcePolyIndex = patches[i].sourcePolyIndex;
        atlas.patches[i].uv.reserve(p.verts.size());
        for (const Vector3& vv : p.verts) {
            const float u = (Vector3DotProduct(vv, r.gpu.axisU) - r.gpu.minU) / r.gpu.luxelSize;
            const float v = (Vector3DotProduct(vv, r.gpu.axisV) - r.gpu.minV) / r.gpu.luxelSize;
            // Vertex UVs map to luxel-grid boundaries within the rect (NOT pixel
            // centers). The bake loop samples luxel `k` over the polygon chunk
            // [k, k+1) (see ju/jv in BakeLightmapCPUPage) so luxel k's value is
            // geometrically centered at local coord k+0.5. Placing the polygon's
            // vertex at the grid-line `k` lets linear interpolation of the vertex
            // UVs hit luxel k's center exactly at u_local = k+0.5, and the
            // polygon edges (u_local = 0 and u_local = interiorW) land at the
            // boundary between an interior luxel and the adjacent padding luxel.
            // After DilatePage those padding luxels carry the nearest interior
            // value, so bilinear at the edge resolves to ~the edge luxel value.
            //
            // The old formula added `+ 0.5f` on top, which shifted the entire
            // lightmap half-a-luxel across every polygon: the minU vertex still
            // sampled the first interior luxel's center, but the maxU vertex
            // sampled the *first padding luxel's* center (past the interior).
            // At lmscale=1 that 0.5-luxel shift is 0.5 world units (invisible),
            // but at lmscale=16 it's 8 world units, which manifests as very
            // visible seams at polygon edges because adjacent polygons shift in
            // different directions relative to their shared edge.
            atlas.patches[i].uv.push_back({
                (r.gpu.x + LM_PAD + u) / page.width,
                (r.gpu.y + LM_PAD + v) / page.height
            });
        }
    }
}

static std::vector<uint8_t> BuildCoverageMask(const std::vector<FaceRect>& rects,
                                              uint32_t pageIndex,
                                              int W, int H)
{
    std::vector<uint8_t> coverage((size_t)W * (size_t)H, 0);
    const float invGrid = 1.0f / (float)g_aaGrid;
    for (const FaceRect& r : rects) {
        if (r.page != pageIndex) {
            continue;
        }
        for (int ly = 0; ly < r.gpu.h; ++ly) {
            for (int lx = 0; lx < r.gpu.w; ++lx) {
                uint8_t coveredSamples = 0;
                for (int sy = 0; sy < g_aaGrid; ++sy) {
                    for (int sx = 0; sx < g_aaGrid; ++sx) {
                        const float cu = (lx - LM_PAD) + (sx + 0.5f) * invGrid;
                        const float cv = (ly - LM_PAD) + (sy + 0.5f) * invGrid;
                        if (InsidePoly2D(r.poly2d, cu, cv)) {
                            ++coveredSamples;
                        }
                    }
                }
                coverage[(size_t)(r.gpu.y + ly) * W + (r.gpu.x + lx)] = coveredSamples;
            }
        }
    }
    return coverage;
}

static std::vector<uint8_t> CoverageToValidMask(const std::vector<uint8_t>& coverage) {
    std::vector<uint8_t> valid(coverage.size(), 0);
    for (size_t i = 0; i < coverage.size(); ++i) {
        valid[i] = coverage[i] > 0 ? 1 : 0;
    }
    return valid;
}

static std::vector<uint8_t> BuildStrictInteriorMask(const std::vector<FaceRect>& rects,
                                                    uint32_t pageIndex,
                                                    int W,
                                                    int H)
{
    std::vector<uint8_t> strictInterior((size_t)W * (size_t)H, 0);
    for (const FaceRect& rect : rects) {
        if (rect.page != pageIndex) {
            continue;
        }
        for (int ly = 0; ly < rect.gpu.h; ++ly) {
            for (int lx = 0; lx < rect.gpu.w; ++lx) {
                const float cu = (float)(lx - LM_PAD) + 0.5f;
                const float cv = (float)(ly - LM_PAD) + 0.5f;
                if (!InsidePoly2D(rect.poly2d, cu, cv)) {
                    continue;
                }
                if (MinDistToPolyEdge2D(rect.poly2d, cu, cv) < EDGE_SEAM_GUARD_LUXELS) {
                    continue;
                }

                const int pageX = rect.gpu.x + lx;
                const int pageY = rect.gpu.y + ly;
                if (pageX < 0 || pageY < 0 || pageX >= W || pageY >= H) {
                    continue;
                }
                strictInterior[(size_t)pageY * (size_t)W + (size_t)pageX] = 1;
            }
        }
    }
    return strictInterior;
}

static void StabilizeEdgeTexels(LightmapPage& page,
                                const std::vector<uint8_t>& coverage)
{
    const int W = page.width;
    const int H = page.height;
    const uint8_t kFullCoverage = (uint8_t)(g_aaGrid * g_aaGrid);
    const size_t pixelCount = (size_t)W * (size_t)H;
    if (coverage.size() < pixelCount) {
        return;
    }

    std::vector<uint8_t> stable(pixelCount, 0);
    for (size_t i = 0; i < pixelCount; ++i) {
        stable[i] = (coverage[i] == kFullCoverage) ? 1 : 0;
    }

    bool anyInterior = false;
    for (uint8_t v : stable) {
        if (v) {
            anyInterior = true;
            break;
        }
    }
    if (!anyInterior) {
        return;
    }

    for (int pass = 0; pass < STABILIZE_EDGE_PASSES; ++pass) {
        bool changed = false;
        std::vector<uint8_t> nextStable = stable;
        std::vector<float> nextPixels = page.pixels;

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * W + x;
                if (coverage[idx] == 0 || stable[idx]) {
                    continue;
                }

                float sr = 0.0f, sg = 0.0f, sb = 0.0f;
                int n = 0;
                for (int oy = -1; oy <= 1; ++oy) {
                    for (int ox = -1; ox <= 1; ++ox) {
                        if (ox == 0 && oy == 0) {
                            continue;
                        }
                        const int nx = x + ox;
                        const int ny = y + oy;
                        if (nx < 0 || ny < 0 || nx >= W || ny >= H) {
                            continue;
                        }
                        const size_t ni = (size_t)ny * W + nx;
                        if (!stable[ni]) {
                            continue;
                        }
                        sr += page.pixels[ni * 4 + 0];
                        sg += page.pixels[ni * 4 + 1];
                        sb += page.pixels[ni * 4 + 2];
                        ++n;
                    }
                }

                if (n > 0) {
                    nextPixels[idx * 4 + 0] = sr / (float)n;
                    nextPixels[idx * 4 + 1] = sg / (float)n;
                    nextPixels[idx * 4 + 2] = sb / (float)n;
                    nextPixels[idx * 4 + 3] = 1.0f;
                    nextStable[idx] = 1;
                    changed = true;
                }
            }
        }

        page.pixels.swap(nextPixels);
        stable.swap(nextStable);
        if (!changed) {
            break;
        }
    }
}

struct StitchedSourceFaceCanvas {
    float minU = 0.0f;
    float minV = 0.0f;
    float luxelSize = 0.0f;
    int width = 0;
    int height = 0;
    std::vector<uint8_t> valid;
    std::vector<float> pixelsR;
    std::vector<float> pixelsG;
    std::vector<float> pixelsB;
};

struct OversampledRectBuffer {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> opaque;
    std::vector<float> pixelsR;
    std::vector<float> pixelsG;
    std::vector<float> pixelsB;
};

static bool GlobalCenterToStitchedIndex(float stitchedMinU,
                                        float stitchedMinV,
                                        float stitchedLuxelSize,
                                        int stitchedWidth,
                                        int stitchedHeight,
                                        float globalUCenter,
                                        float globalVCenter,
                                        int* outX,
                                        int* outY)
{
    if (!outX || !outY) {
        return false;
    }

    int stitchedX = 0;
    int stitchedY = 0;
    if (!LocalInteriorIndexFromGlobalCenter(stitchedMinU, stitchedLuxelSize, stitchedWidth, globalUCenter, &stitchedX) ||
        !LocalInteriorIndexFromGlobalCenter(stitchedMinV, stitchedLuxelSize, stitchedHeight, globalVCenter, &stitchedY)) {
        return false;
    }

    *outX = stitchedX;
    *outY = stitchedY;
    return true;
}

static bool InitializeStitchedSourceFaceCanvas(const std::vector<FaceRect>& rects,
                                               const std::vector<size_t>& rectGroup,
                                               int sampleScale,
                                               StitchedSourceFaceCanvas* outCanvas)
{
    if (!outCanvas || rectGroup.empty()) {
        return false;
    }

    const FaceRect& firstRect = rects[rectGroup.front()];
    float minU = firstRect.gpu.minU;
    float minV = firstRect.gpu.minV;
    float maxU = firstRect.maxU;
    float maxV = firstRect.maxV;
    const float baseLuxelSize = std::max(0.125f, firstRect.gpu.luxelSize);
    for (size_t rectIndex : rectGroup) {
        const FaceRect& rect = rects[rectIndex];
        minU = std::min(minU, rect.gpu.minU);
        minV = std::min(minV, rect.gpu.minV);
        maxU = std::max(maxU, rect.maxU);
        maxV = std::max(maxV, rect.maxV);
    }

    const int safeSampleScale = std::max(1, sampleScale);
    const int stitchedWidth = ComputeInteriorLuxelSpan(maxU - minU, baseLuxelSize) * safeSampleScale;
    const int stitchedHeight = ComputeInteriorLuxelSpan(maxV - minV, baseLuxelSize) * safeSampleScale;
    if (stitchedWidth <= 0 || stitchedHeight <= 0) {
        return false;
    }

    const size_t pixelCount = (size_t)stitchedWidth * (size_t)stitchedHeight;
    outCanvas->minU = minU;
    outCanvas->minV = minV;
    outCanvas->luxelSize = baseLuxelSize / (float)safeSampleScale;
    outCanvas->width = stitchedWidth;
    outCanvas->height = stitchedHeight;
    outCanvas->valid.assign(pixelCount, 0);
    outCanvas->pixelsR.assign(pixelCount, 0.0f);
    outCanvas->pixelsG.assign(pixelCount, 0.0f);
    outCanvas->pixelsB.assign(pixelCount, 0.0f);
    return true;
}

static bool RectLocalPixelToStitchedIndex(const FaceRect& rect,
                                          float stitchedMinU,
                                          float stitchedMinV,
                                          float stitchedLuxelSize,
                                          int stitchedWidth,
                                          int stitchedHeight,
                                          int localX,
                                          int localY,
                                          int* outX,
                                          int* outY)
{
    if (!outX || !outY) {
        return false;
    }

    const float globalUCenter = rect.gpu.minU + ((float)(localX - LM_PAD) + 0.5f) * rect.gpu.luxelSize;
    const float globalVCenter = rect.gpu.minV + ((float)(localY - LM_PAD) + 0.5f) * rect.gpu.luxelSize;
    return GlobalCenterToStitchedIndex(stitchedMinU,
                                       stitchedMinV,
                                       stitchedLuxelSize,
                                       stitchedWidth,
                                       stitchedHeight,
                                       globalUCenter,
                                       globalVCenter,
                                       outX,
                                       outY);
}

static bool RectLocalSubsampleToStitchedIndex(const FaceRect& rect,
                                              float stitchedMinU,
                                              float stitchedMinV,
                                              float stitchedLuxelSize,
                                              int stitchedWidth,
                                              int stitchedHeight,
                                              int localX,
                                              int localY,
                                              int subX,
                                              int subY,
                                              int sampleScale,
                                              int* outX,
                                              int* outY)
{
    if (!outX || !outY || sampleScale <= 0) {
        return false;
    }

    const float invScale = 1.0f / (float)sampleScale;
    const float globalUCenter = rect.gpu.minU + ((float)(localX - LM_PAD) + ((float)subX + 0.5f) * invScale) * rect.gpu.luxelSize;
    const float globalVCenter = rect.gpu.minV + ((float)(localY - LM_PAD) + ((float)subY + 0.5f) * invScale) * rect.gpu.luxelSize;
    return GlobalCenterToStitchedIndex(stitchedMinU,
                                       stitchedMinV,
                                       stitchedLuxelSize,
                                       stitchedWidth,
                                       stitchedHeight,
                                       globalUCenter,
                                       globalVCenter,
                                       outX,
                                       outY);
}

static std::unordered_map<uint32_t, std::vector<size_t>> GroupRectsBySourcePoly(const std::vector<FaceRect>& rects) {
    std::unordered_map<uint32_t, std::vector<size_t>> rectsBySourcePoly;
    rectsBySourcePoly.reserve(rects.size());
    for (size_t i = 0; i < rects.size(); ++i) {
        rectsBySourcePoly[rects[i].sourcePolyIndex].push_back(i);
    }
    return rectsBySourcePoly;
}

static bool BuildStitchedSourceFaceCanvas(const std::vector<FaceRect>& rects,
                                          const std::vector<LightmapPage>& pages,
                                          const std::vector<std::vector<uint8_t>>& validMasks,
                                          const std::vector<size_t>& rectGroup,
                                          StitchedSourceFaceCanvas* outCanvas)
{
    if (!InitializeStitchedSourceFaceCanvas(rects, rectGroup, 1, outCanvas)) {
        return false;
    }

    const size_t pixelCount = (size_t)outCanvas->width * (size_t)outCanvas->height;
    std::vector<uint16_t> sampleCounts(pixelCount, 0);
    for (size_t rectIndex : rectGroup) {
        const FaceRect& rect = rects[rectIndex];
        if (rect.page >= pages.size() || rect.page >= validMasks.size()) {
            continue;
        }

        const LightmapPage& page = pages[rect.page];
        const std::vector<uint8_t>& valid = validMasks[rect.page];
        const size_t pagePixelCount = (size_t)page.width * (size_t)page.height;
        if (page.pixels.size() < pagePixelCount * 4 || valid.size() < pagePixelCount) {
            continue;
        }

        for (int ly = 0; ly < rect.gpu.h; ++ly) {
            for (int lx = 0; lx < rect.gpu.w; ++lx) {
                const int pageX = rect.gpu.x + lx;
                const int pageY = rect.gpu.y + ly;
                if (pageX < 0 || pageY < 0 || pageX >= page.width || pageY >= page.height) {
                    continue;
                }

                const size_t pagePixelIndex = (size_t)pageY * (size_t)page.width + (size_t)pageX;
                if (!valid[pagePixelIndex]) {
                    continue;
                }

                int stitchedX = 0;
                int stitchedY = 0;
                if (!RectLocalPixelToStitchedIndex(rect,
                                                   outCanvas->minU,
                                                   outCanvas->minV,
                                                   outCanvas->luxelSize,
                                                   outCanvas->width,
                                                   outCanvas->height,
                                                   lx,
                                                   ly,
                                                   &stitchedX,
                                                   &stitchedY)) {
                    continue;
                }

                const size_t stitchedIndex = (size_t)stitchedY * (size_t)outCanvas->width + (size_t)stitchedX;
                outCanvas->pixelsR[stitchedIndex] += page.pixels[pagePixelIndex * 4 + 0];
                outCanvas->pixelsG[stitchedIndex] += page.pixels[pagePixelIndex * 4 + 1];
                outCanvas->pixelsB[stitchedIndex] += page.pixels[pagePixelIndex * 4 + 2];
                sampleCounts[stitchedIndex] += 1;
            }
        }
    }

    bool anyValid = false;
    for (size_t i = 0; i < pixelCount; ++i) {
        if (sampleCounts[i] == 0) {
            continue;
        }
        const float invCount = 1.0f / (float)sampleCounts[i];
        outCanvas->pixelsR[i] *= invCount;
        outCanvas->pixelsG[i] *= invCount;
        outCanvas->pixelsB[i] *= invCount;
        outCanvas->valid[i] = 1;
        anyValid = true;
    }
    return anyValid;
}

static void WriteStitchedSourceFaceCanvas(const StitchedSourceFaceCanvas& canvas,
                                          const std::vector<FaceRect>& rects,
                                          const std::vector<std::vector<uint8_t>>& validMasks,
                                          const std::vector<size_t>& rectGroup,
                                          std::vector<LightmapPage>& pages)
{
    if (canvas.width <= 0 || canvas.height <= 0) {
        return;
    }

    const size_t stitchedPixelCount = (size_t)canvas.width * (size_t)canvas.height;
    for (size_t rectIndex : rectGroup) {
        const FaceRect& rect = rects[rectIndex];
        if (rect.page >= pages.size() || rect.page >= validMasks.size()) {
            continue;
        }

        LightmapPage& page = pages[rect.page];
        const std::vector<uint8_t>& valid = validMasks[rect.page];
        const size_t pagePixelCount = (size_t)page.width * (size_t)page.height;
        if (page.pixels.size() < pagePixelCount * 4 || valid.size() < pagePixelCount) {
            continue;
        }

        for (int ly = 0; ly < rect.gpu.h; ++ly) {
            for (int lx = 0; lx < rect.gpu.w; ++lx) {
                const int pageX = rect.gpu.x + lx;
                const int pageY = rect.gpu.y + ly;
                if (pageX < 0 || pageY < 0 || pageX >= page.width || pageY >= page.height) {
                    continue;
                }

                const size_t pagePixelIndex = (size_t)pageY * (size_t)page.width + (size_t)pageX;
                if (!valid[pagePixelIndex]) {
                    continue;
                }

                int stitchedX = 0;
                int stitchedY = 0;
                if (!RectLocalPixelToStitchedIndex(rect,
                                                   canvas.minU,
                                                   canvas.minV,
                                                   canvas.luxelSize,
                                                   canvas.width,
                                                   canvas.height,
                                                   lx,
                                                   ly,
                                                   &stitchedX,
                                                   &stitchedY)) {
                    continue;
                }

                const size_t stitchedIndex = (size_t)stitchedY * (size_t)canvas.width + (size_t)stitchedX;
                if (stitchedIndex >= stitchedPixelCount || !canvas.valid[stitchedIndex]) {
                    continue;
                }

                page.pixels[pagePixelIndex * 4 + 0] = std::max(0.0f, canvas.pixelsR[stitchedIndex]);
                page.pixels[pagePixelIndex * 4 + 1] = std::max(0.0f, canvas.pixelsG[stitchedIndex]);
                page.pixels[pagePixelIndex * 4 + 2] = std::max(0.0f, canvas.pixelsB[stitchedIndex]);
                page.pixels[pagePixelIndex * 4 + 3] = 1.0f;
            }
        }
    }
}

static void FloodFillTransparentCanvas(int width,
                                       int height,
                                       std::vector<uint8_t>& opaque,
                                       std::vector<float>& pixelsR,
                                       std::vector<float>& pixelsG,
                                       std::vector<float>& pixelsB)
{
    if (width <= 0 || height <= 0) {
        return;
    }

    const size_t pixelCount = (size_t)width * (size_t)height;
    while (true) {
        int unhandledPixels = 0;
        bool changed = false;
        std::vector<uint8_t> nextOpaque = opaque;
        std::vector<float> nextR = pixelsR;
        std::vector<float> nextG = pixelsG;
        std::vector<float> nextB = pixelsB;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const size_t idx = (size_t)y * (size_t)width + (size_t)x;
                if (opaque[idx]) {
                    continue;
                }

                int opaqueNeighbors = 0;
                float sumR = 0.0f;
                float sumG = 0.0f;
                float sumB = 0.0f;
                for (int oy = -1; oy <= 1; ++oy) {
                    for (int ox = -1; ox <= 1; ++ox) {
                        if (ox == 0 && oy == 0) {
                            continue;
                        }
                        const int nx = x + ox;
                        const int ny = y + oy;
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }
                        const size_t ni = (size_t)ny * (size_t)width + (size_t)nx;
                        if (!opaque[ni]) {
                            continue;
                        }
                        sumR += pixelsR[ni];
                        sumG += pixelsG[ni];
                        sumB += pixelsB[ni];
                        ++opaqueNeighbors;
                    }
                }

                if (opaqueNeighbors > 0) {
                    const float invNeighborCount = 1.0f / (float)opaqueNeighbors;
                    nextR[idx] = sumR * invNeighborCount;
                    nextG[idx] = sumG * invNeighborCount;
                    nextB[idx] = sumB * invNeighborCount;
                    nextOpaque[idx] = 1;
                    changed = true;
                } else {
                    ++unhandledPixels;
                }
            }
        }

        opaque.swap(nextOpaque);
        pixelsR.swap(nextR);
        pixelsG.swap(nextG);
        pixelsB.swap(nextB);

        if (!changed || unhandledPixels == 0 || unhandledPixels == (int)pixelCount) {
            break;
        }
    }
}

static void ApplyLightmapAA(std::vector<LightmapPage>& pages,
                            const std::vector<FaceRect>& rects,
                            const std::vector<std::vector<uint8_t>>& validMasks,
                            int lmAAScale) {
    if (lmAAScale <= 0) return;

    const float sigma = lmAAScale / 2.0f;
    const int radius = (int)ceilf(2.0f * sigma);

    // Precompute 1D Gaussian kernel
    std::vector<float> kernel(radius + 1);
    {
        float sum = 0.0f;
        for (int i = 0; i <= radius; ++i) {
            kernel[i] = expf(-(float)(i * i) / (2.0f * sigma * sigma));
            sum += (i == 0) ? kernel[i] : 2.0f * kernel[i];
        }
        for (int i = 0; i <= radius; ++i) kernel[i] /= sum;
    }

    const auto rectsBySourcePoly = GroupRectsBySourcePoly(rects);
    for (const auto& [sourcePolyIndex, rectGroup] : rectsBySourcePoly) {
        (void)sourcePolyIndex;
        StitchedSourceFaceCanvas canvas;
        if (!BuildStitchedSourceFaceCanvas(rects, pages, validMasks, rectGroup, &canvas)) {
            continue;
        }

        const int W = canvas.width;
        const int H = canvas.height;
        const size_t pixelCount = (size_t)W * (size_t)H;
        std::vector<float> tempR(pixelCount, 0.0f);
        std::vector<float> tempG(pixelCount, 0.0f);
        std::vector<float> tempB(pixelCount, 0.0f);

        // Blur in stitched source-face space so split bake patches filter as
        // one continuous lightmap surface while still respecting polygon
        // coverage via the stitched valid mask.
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * (size_t)W + (size_t)x;
                if (!canvas.valid[idx]) {
                    continue;
                }

                float sumR = 0.0f;
                float sumG = 0.0f;
                float sumB = 0.0f;
                float wTotal = 0.0f;
                for (int k = -radius; k <= radius; ++k) {
                    const int nx = x + k;
                    if (nx < 0 || nx >= W) {
                        continue;
                    }
                    const size_t ni = (size_t)y * (size_t)W + (size_t)nx;
                    if (!canvas.valid[ni]) {
                        continue;
                    }
                    const float w = kernel[abs(k)];
                    sumR += w * canvas.pixelsR[ni];
                    sumG += w * canvas.pixelsG[ni];
                    sumB += w * canvas.pixelsB[ni];
                    wTotal += w;
                }
                if (wTotal > 0.0f) {
                    tempR[idx] = sumR / wTotal;
                    tempG[idx] = sumG / wTotal;
                    tempB[idx] = sumB / wTotal;
                }
            }
        }

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * (size_t)W + (size_t)x;
                if (!canvas.valid[idx]) {
                    continue;
                }

                float sumR = 0.0f;
                float sumG = 0.0f;
                float sumB = 0.0f;
                float wTotal = 0.0f;
                for (int k = -radius; k <= radius; ++k) {
                    const int ny = y + k;
                    if (ny < 0 || ny >= H) {
                        continue;
                    }
                    const size_t ni = (size_t)ny * (size_t)W + (size_t)x;
                    if (!canvas.valid[ni]) {
                        continue;
                    }
                    const float w = kernel[abs(k)];
                    sumR += w * tempR[ni];
                    sumG += w * tempG[ni];
                    sumB += w * tempB[ni];
                    wTotal += w;
                }
                if (wTotal > 0.0f) {
                    canvas.pixelsR[idx] = sumR / wTotal;
                    canvas.pixelsG[idx] = sumG / wTotal;
                    canvas.pixelsB[idx] = sumB / wTotal;
                }
            }
        }

        WriteStitchedSourceFaceCanvas(canvas, rects, validMasks, rectGroup, pages);
    }
}

static void DilatePage(LightmapPage& page, std::vector<uint8_t>& valid) {
    const int W = page.width;
    const int H = page.height;
    for (int pass = 0; pass < DILATE_PASSES; ++pass) {
        std::vector<uint8_t> nextValid = valid;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * W + x;
                if (valid[idx]) continue;
                float sr = 0.0f, sg = 0.0f, sb = 0.0f;
                int n = 0;
                const int dx[4] = {-1, 1, 0, 0};
                const int dy[4] = {0, 0, -1, 1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = x + dx[k];
                    const int ny = y + dy[k];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    const size_t ni = (size_t)ny * W + nx;
                    if (!valid[ni]) continue;
                    sr += page.pixels[ni * 4 + 0];
                    sg += page.pixels[ni * 4 + 1];
                    sb += page.pixels[ni * 4 + 2];
                    ++n;
                }
                if (n) {
                    page.pixels[idx * 4 + 0] = sr / (float)n;
                    page.pixels[idx * 4 + 1] = sg / (float)n;
                    page.pixels[idx * 4 + 2] = sb / (float)n;
                    page.pixels[idx * 4 + 3] = 1.0f;
                    nextValid[idx] = 1;
                }
            }
        }
        valid.swap(nextValid);
    }
}

struct DarkLuxelStats {
    size_t validCount = 0;
    size_t darkValidCount = 0;
    size_t fullCoverageCount = 0;
    size_t darkFullCoverageCount = 0;
    size_t strictInteriorCount = 0;
    size_t darkStrictInteriorCount = 0;
};

static DarkLuxelStats GatherDarkLuxelStats(const LightmapPage& page,
                                           const std::vector<uint8_t>& valid,
                                           const std::vector<uint8_t>& coverage,
                                           const std::vector<uint8_t>& strictInterior)
{
    const size_t pixelCount = (size_t)page.width * (size_t)page.height;
    DarkLuxelStats stats;
    if (valid.size() < pixelCount ||
        coverage.size() < pixelCount ||
        strictInterior.size() < pixelCount ||
        page.pixels.size() < pixelCount * 4)
    {
        stats.darkValidCount = 1;
        return stats;
    }

    const uint8_t kFullCoverage = (uint8_t)(g_aaGrid * g_aaGrid);
    for (size_t i = 0; i < pixelCount; ++i) {
        if (!valid[i]) {
            continue;
        }
        ++stats.validCount;
        const float r = page.pixels[i * 4 + 0];
        const float g = page.pixels[i * 4 + 1];
        const float b = page.pixels[i * 4 + 2];
        const bool dark = r < DARK_LUXEL_THRESHOLD &&
                          g < DARK_LUXEL_THRESHOLD &&
                          b < DARK_LUXEL_THRESHOLD;
        if (dark) {
            ++stats.darkValidCount;
        }
        if (coverage[i] == kFullCoverage) {
            ++stats.fullCoverageCount;
            if (dark) {
                ++stats.darkFullCoverageCount;
            }
        }
        if (strictInterior[i]) {
            ++stats.strictInteriorCount;
            if (dark) {
                ++stats.darkStrictInteriorCount;
            }
        }
    }
    return stats;
}

static bool HasInvalidDarkValidLuxels(const DarkLuxelStats& stats)
{
    if (stats.validCount == 0) {
        return false;
    }

    // A strict interior mask avoids false positives from aa=1 boundary samples,
    // where CPU and GPU inside-polygon tests can disagree by one pixel.
    if (stats.strictInteriorCount > 0) {
        return stats.darkStrictInteriorCount > 0;
    }

    // Full-coverage luxels are only a strong fallback signal once super-sampling
    // is enabled; with aa=1 every valid luxel is "full coverage".
    if (g_aaGrid > 1 && stats.fullCoverageCount > 0) {
        return stats.darkFullCoverageCount > 0;
    }

    // Sliver-only pages may have no interior luxels at all. Only treat those as
    // broken when every valid luxel came back black.
    return stats.darkValidCount == stats.validCount;
}

static std::vector<std::vector<uint32_t>> BuildRectLightIndices(const std::vector<FaceRect>& rects,
                                                                const std::vector<PointLight>& lights)
{
    std::vector<std::vector<uint32_t>> lightIndices(rects.size());
    for (size_t rectIndex = 0; rectIndex < rects.size(); ++rectIndex) {
        const FaceRect& r = rects[rectIndex];
        std::vector<uint32_t>& indices = lightIndices[rectIndex];
        indices.reserve(lights.size());
        for (uint32_t lightIndex = 0; lightIndex < lights.size(); ++lightIndex) {
            const PointLight& light = lights[lightIndex];
            if (IsParallelLight(light)) {
                indices.push_back(lightIndex);
                continue;
            }
            const float radiusSq = light.intensity * light.intensity;
            if (DistSqPointAABB(light.position, r.bounds) <= radiusSq) {
                indices.push_back(lightIndex);
            }
        }
    }
    return lightIndices;
}

static std::vector<std::vector<uint32_t>> BuildSurfaceEmitterIndicesByRect(const std::vector<FaceRect>& rects,
                                                                           const std::vector<SurfaceLightEmitter>& surfaceEmitters)
{
    std::vector<std::vector<uint32_t>> emitterIndices(rects.size());
    for (size_t rectIndex = 0; rectIndex < rects.size(); ++rectIndex) {
        const FaceRect& rect = rects[rectIndex];
        std::vector<uint32_t>& rectEmitterIndices = emitterIndices[rectIndex];
        rectEmitterIndices.reserve(surfaceEmitters.size());
        for (uint32_t emitterIndex = 0; emitterIndex < surfaceEmitters.size(); ++emitterIndex) {
            const SurfaceLightEmitter& emitter = surfaceEmitters[emitterIndex];
            if (IsParallelLight(emitter.baseLight)) {
                rectEmitterIndices.push_back(emitterIndex);
                continue;
            }
            const float radiusSq = emitter.baseLight.intensity * emitter.baseLight.intensity;
            if (DistSqAABBAABB(rect.bounds, emitter.bounds) <= radiusSq) {
                rectEmitterIndices.push_back(emitterIndex);
            }
        }
    }
    return emitterIndices;
}

static void BuildRectSurfaceEmitterIndices(std::vector<FaceRect>& rects,
                                           const std::vector<SurfaceLightEmitter>& surfaceEmitters)
{
    const std::vector<std::vector<uint32_t>> emitterIndices = BuildSurfaceEmitterIndicesByRect(rects, surfaceEmitters);
    for (size_t rectIndex = 0; rectIndex < rects.size(); ++rectIndex) {
        rects[rectIndex].surfaceEmitterIndices = emitterIndices[rectIndex];
    }
}

static std::vector<PointLight> GatherPageLights(const std::vector<FaceRect>& rects,
                                                uint32_t pageIndex,
                                                const std::vector<PointLight>& lights,
                                                const std::vector<std::vector<uint32_t>>* lightIndicesByRect = nullptr)
{
    if (lights.empty()) {
        return {};
    }

    std::vector<uint8_t> used(lights.size(), 0);
    for (size_t rectIndex = 0; rectIndex < rects.size(); ++rectIndex) {
        const FaceRect& r = rects[rectIndex];
        if (r.page != pageIndex) {
            continue;
        }
        const std::vector<uint32_t>& rectLightIndices = lightIndicesByRect ? (*lightIndicesByRect)[rectIndex] : r.lightIndices;
        for (uint32_t lightIndex : rectLightIndices) {
            used[lightIndex] = 1;
        }
    }

    std::vector<PointLight> pageLights;
    for (size_t i = 0; i < lights.size(); ++i) {
        if (used[i]) {
            pageLights.push_back(lights[i]);
        }
    }
    return pageLights;
}

static size_t CountPageSurfaceEmitters(const std::vector<FaceRect>& rects,
                                       uint32_t pageIndex,
                                       size_t emitterCount,
                                       const std::vector<std::vector<uint32_t>>* emitterIndicesByRect = nullptr)
{
    if (emitterCount == 0) {
        return 0;
    }

    std::vector<uint8_t> used(emitterCount, 0);
    for (size_t rectIndex = 0; rectIndex < rects.size(); ++rectIndex) {
        const FaceRect& rect = rects[rectIndex];
        if (rect.page != pageIndex) {
            continue;
        }
        const std::vector<uint32_t>& rectEmitterIndices = emitterIndicesByRect
            ? (*emitterIndicesByRect)[rectIndex]
            : rect.surfaceEmitterIndices;
        for (uint32_t emitterIndex : rectEmitterIndices) {
            if (emitterIndex < used.size()) {
                used[emitterIndex] = 1;
            }
        }
    }

    size_t count = 0;
    for (uint8_t flag : used) {
        count += (flag != 0);
    }
    return count;
}

static AABB ComputeMapBounds(const std::vector<MapPolygon>& polys) {
    AABB bounds = AABBInvalid();
    for (const MapPolygon& poly : polys) {
        for (const Vector3& vert : poly.verts) {
            AABBExtend(&bounds, vert);
        }
    }
    if (bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z) {
        bounds.min = Vector3Zero();
        bounds.max = Vector3Zero();
    }
    return bounds;
}

static Vector3 SampleConeDirection(const Vector3& axis, float coneAngleDeg, int sampleIndex, int sampleCount) {
    const Vector3 dirAxis = Vector3Normalize(axis);
    if (sampleCount <= 1 || coneAngleDeg <= 1e-3f) {
        return dirAxis;
    }

    Vector3 tangent{};
    Vector3 bitangent{};
    FaceBasis(dirAxis, tangent, bitangent);
    const float u1 = ((float)sampleIndex + 0.5f) / (float)sampleCount;
    const float u2 = fmodf(((float)sampleIndex + 0.5f) * 0.61803398875f, 1.0f);
    const float cosMax = cosf(std::clamp(coneAngleDeg * 0.5f, 0.0f, 89.0f) * DEG2RAD);
    const float cosTheta = 1.0f - u1 * (1.0f - cosMax);
    const float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));
    const float phi = u2 * PI * 2.0f;
    const Vector3 radial = Vector3Add(Vector3Scale(tangent, cosf(phi)), Vector3Scale(bitangent, sinf(phi)));
    return Vector3Normalize(Vector3Add(Vector3Scale(dirAxis, cosTheta), Vector3Scale(radial, sinTheta)));
}

static int EricwSkyDomeSampleCount() {
    int iterations = (int)lroundf(sqrtf((float)(ERICW_SUNSAMPLES - 1) / 4.0f)) + 1;
    iterations = std::max(iterations, 2);
    const int elevationSteps = iterations - 1;
    const int angleSteps = elevationSteps * 4;
    return angleSteps * elevationSteps + 1;
}

static Vector3 EricwSkyDomeDirection(bool upperHemisphere, int sampleIndex, float rotationAngle) {
    int iterations = (int)lroundf(sqrtf((float)(ERICW_SUNSAMPLES - 1) / 4.0f)) + 1;
    iterations = std::max(iterations, 2);
    const int elevationSteps = iterations - 1;
    const int angleSteps = elevationSteps * 4;
    const int sampleCount = angleSteps * elevationSteps + 1;
    const float elevationStep = (90.0f / (float)(elevationSteps + 1)) * DEG2RAD;
    const float angleStep = (360.0f / (float)angleSteps) * DEG2RAD;

    if (sampleIndex >= sampleCount - 1) {
        return upperHemisphere ? Vector3{0.0f, 1.0f, 0.0f} : Vector3{0.0f, -1.0f, 0.0f};
    }

    const int ringIndex = sampleIndex / angleSteps;
    const int angleIndex = sampleIndex % angleSteps;
    const float elevation = elevationStep * (0.5f + (float)ringIndex);
    const float angle = angleStep * ((float)angleIndex + ((float)ringIndex / (float)elevationSteps)) + rotationAngle;
    const float radialScale = cosf(elevation);
    const float verticalScale = sinf(elevation) * (upperHemisphere ? 1.0f : -1.0f);
    return Vector3Normalize({
        cosf(angle) * radialScale,
        verticalScale,
        sinf(angle) * radialScale
    });
}

static PointLight BuildDirectionalSunLight(const Vector3& toLightDirection,
                                           const Vector3& color,
                                           float angleScale,
                                           int dirtOverride,
                                           bool requireSkyVisibility,
                                           const Vector3& mapCenter,
                                           float sunDistance)
{
    PointLight light{};
    const Vector3 dir = Vector3Normalize(toLightDirection);
    light.position = Vector3Add(mapCenter, Vector3Scale(dir, sunDistance));
    light.color = color;
    light.intensity = std::max(1.0f, sunDistance * 2.0f);
    light.parallelDirection = dir;
    light.parallel = 1;
    light.requiresSkyVisibility = requireSkyVisibility ? 1 : 0;
    light.attenuationMode = POINT_LIGHT_ATTEN_NONE;
    light.angleScale = angleScale;
    light.dirt = (int8_t)dirtOverride;
    return light;
}

static std::vector<Vector3> GenerateSurfaceEmitterPlanePoints(const MapPolygon& poly,
                                                              float sampleSpacing)
{
    Vector3 axisU{};
    Vector3 axisV{};
    FaceBasis(poly.normal, axisU, axisV);

    float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
    std::vector<LocalPolyVert> localPoly;
    localPoly.reserve(poly.verts.size());
    for (const Vector3& vert : poly.verts) {
        const float u = Vector3DotProduct(vert, axisU);
        const float v = Vector3DotProduct(vert, axisV);
        minU = std::min(minU, u);
        maxU = std::max(maxU, u);
        minV = std::min(minV, v);
        maxV = std::max(maxV, v);
        localPoly.push_back({vert, u, v});
    }

    const float safeSpacing = std::max(1.0f, sampleSpacing);
    const float extentU = std::max(0.0f, maxU - minU);
    const float extentV = std::max(0.0f, maxV - minV);
    const int samplesU = std::max(1, std::min(32, (int)ceilf(extentU / safeSpacing)));
    const int samplesV = std::max(1, std::min(32, (int)ceilf(extentV / safeSpacing)));
    const float stepU = (samplesU > 0) ? (extentU / (float)samplesU) : 0.0f;
    const float stepV = (samplesV > 0) ? (extentV / (float)samplesV) : 0.0f;

    std::vector<Vector3> planePoints;
    planePoints.reserve((size_t)samplesU * (size_t)samplesV);
    for (int y = 0; y < samplesV; ++y) {
        const float clipMinV = minV + (float)y * stepV;
        const float clipMaxV = (y + 1 == samplesV) ? maxV : (clipMinV + stepV);
        for (int x = 0; x < samplesU; ++x) {
            const float clipMinU = minU + (float)x * stepU;
            const float clipMaxU = (x + 1 == samplesU) ? maxU : (clipMinU + stepU);
            std::vector<LocalPolyVert> clipped = ClipToRect(localPoly, clipMinU, clipMaxU, clipMinV, clipMaxV);
            if (clipped.size() < 3 || fabsf(SignedArea2D(clipped)) < 1e-4f) {
                continue;
            }

            std::vector<Vector3> clippedVerts;
            clippedVerts.reserve(clipped.size());
            for (const LocalPolyVert& vert : clipped) {
                clippedVerts.push_back(vert.world);
            }
            planePoints.push_back(PolygonCentroid(clippedVerts));
        }
    }

    if (planePoints.empty()) {
        planePoints.push_back(PolygonCentroid(poly.verts));
    }
    return planePoints;
}

static std::vector<Vector3> GenerateFixedSurfaceEmitterSamples(const MapPolygon& poly,
                                                               float sampleSpacing,
                                                               float offsetAlongNormal,
                                                               const std::vector<BrushSolid>& solids,
                                                               size_t* outAdjustedCount = nullptr,
                                                               size_t* outCulledCount = nullptr)
{
    if (outAdjustedCount) {
        *outAdjustedCount = 0;
    }
    if (outCulledCount) {
        *outCulledCount = 0;
    }

    const std::vector<Vector3> planePoints = GenerateSurfaceEmitterPlanePoints(poly, sampleSpacing);
    std::vector<Vector3> samples;
    samples.reserve(planePoints.size());
    for (const Vector3& planePoint : planePoints) {
        RepairedSamplePoint candidate{};
        const Vector3 idealSamplePoint = OffsetPointAlongSurfaceNormal(planePoint, poly.normal, offsetAlongNormal);
        if (TryRepairSampleCandidate(solids, planePoint, poly.normal, offsetAlongNormal, &candidate)) {
            if (outAdjustedCount &&
                Vector3LengthSq(Vector3Subtract(candidate.samplePoint, idealSamplePoint)) > 1e-6f)
            {
                ++(*outAdjustedCount);
            }
            samples.push_back(candidate.samplePoint);
        } else if (outCulledCount) {
            ++(*outCulledCount);
        }
    }
    return samples;
}

static std::vector<PointLight> BuildDirectPointLights(const std::vector<PointLight>& explicitLights,
                                                      const LightBakeSettings& settings,
                                                      const Vector3& mapCenter,
                                                      float sunDistance)
{
    std::vector<PointLight> lights = explicitLights;

    if (settings.sunlightIntensity > 0.0f) {
        const int sunSamples = (settings.sunlightPenumbra > 0.1f) ? ERICW_SUNSAMPLES : 1;
        const Vector3 sampleColor = Vector3Scale(settings.sunlightColor, settings.sunlightIntensity / (300.0f * (float)sunSamples));
        const int dirtOverride = (settings.sunlightDirt == -2) ? settings.dirt : settings.sunlightDirt;
        const bool requireSkyVisibility = settings.sunlightNoSky == 0;
        for (int i = 0; i < sunSamples; ++i) {
            const Vector3 toLight = SampleConeDirection(settings.sunlightDirection, settings.sunlightPenumbra, i, sunSamples);
            lights.push_back(BuildDirectionalSunLight(toLight, sampleColor, settings.sunlightAngleScale, dirtOverride, requireSkyVisibility, mapCenter, sunDistance));
        }
    }

    return lights;
}

static std::vector<SurfaceLightEmitter> BuildSurfaceEmitters(const std::vector<MapPolygon>& polys,
                                                             const std::vector<SurfaceLightTemplate>& surfaceLights,
                                                             const std::vector<BrushSolid>& solids,
                                                             const LightBakeSettings& settings)
{
    std::vector<SurfaceLightEmitter> emitters;
    if (surfaceLights.empty() || settings.surfLightScale <= 0.0f) {
        return emitters;
    }
    int matchedSurfaceCount = 0;
    size_t adjustedSampleCount = 0;
    size_t culledSampleCount = 0;
    const float sampleSpacing = std::max(1.0f, settings.surfLightSubdivision);
    for (const SurfaceLightTemplate& templ : surfaceLights) {
        for (const MapPolygon& poly : polys) {
            if (poly.texture != templ.texture) {
                continue;
            }
            if (templ.surfaceLightGroup != 0 && poly.surfaceLightGroup != templ.surfaceLightGroup) {
                continue;
            }

            SurfaceLightEmitter emitter{};
            emitter.baseLight = templ.light;
            emitter.surfaceNormal = poly.normal;
            size_t emitterAdjustedSamples = 0;
            size_t emitterCulledSamples = 0;
            emitter.samplePoints = GenerateFixedSurfaceEmitterSamples(
                poly,
                sampleSpacing,
                std::max(0.0f, templ.surfaceOffset),
                solids,
                &emitterAdjustedSamples,
                &emitterCulledSamples);
            adjustedSampleCount += emitterAdjustedSamples;
            culledSampleCount += emitterCulledSamples;
            if (emitter.samplePoints.empty()) {
                continue;
            }
            emitter.baseLight.position = AveragePoints(emitter.samplePoints);
            emitter.surfaceArea = std::max(1.0f, PolygonArea(poly.verts));
            emitter.sampleIntensityScale = ComputeEmitterSampleIntensityScale(
                emitter.surfaceArea,
                emitter.samplePoints.size());
            emitter.transportScale = settings.surfLightScale;
            emitter.attenuationScale = (poly.surfLightAttenuation >= 0.0f)
                ? poly.surfLightAttenuation
                : settings.surfLightAttenuation;
            emitter.hotspotClamp = DIRECT_SURFACE_HOTSPOT_CLAMP;
            emitter.rescale = (poly.surfLightRescale >= 0) ? (uint8_t)poly.surfLightRescale : 1u;
            emitter.baseLight.intensity = EstimateSurfaceEmitterCullRadius(
                emitter.baseLight.color,
                emitter.surfaceArea,
                emitter.transportScale,
                emitter.attenuationScale,
                emitter.hotspotClamp);
            if (templ.surfaceSpotlight) {
                emitter.baseLight.spotDirection = poly.normal;
                if (emitter.baseLight.spotOuterCos <= -1.5f) {
                    emitter.baseLight.spotOuterCos = cosf(20.0f * DEG2RAD);
                    emitter.baseLight.spotInnerCos = emitter.baseLight.spotOuterCos;
                }
            }
            emitter.bounds = ComputeBoundsFromPoints(emitter.samplePoints);
            AppendDeviatedSurfaceEmitters(emitter, templ.deviance, templ.devianceSamples, ++matchedSurfaceCount, emitters);
        }
    }

    if (!surfaceLights.empty()) {
        size_t totalSurfaceSamples = 0;
        for (const SurfaceLightEmitter& emitter : emitters) {
            totalSurfaceSamples += emitter.samplePoints.size();
        }
        printf("[Lightmap] built %zu grouped surface emitters (%zu samples, %zu adjusted, %zu culled) from %zu surface-light templates.\n",
               emitters.size(), totalSurfaceSamples, adjustedSampleCount, culledSampleCount, surfaceLights.size());
        fflush(stdout);
    }
    return emitters;
}

static bool IsLightBrushTextureName(const std::string& name) {
    return name.rfind("__light_brush_", 0) == 0;
}

static bool PolygonUsesSurfaceLightTemplate(const MapPolygon& poly,
                                            const std::vector<SurfaceLightTemplate>& surfaceLights)
{
    for (const SurfaceLightTemplate& templ : surfaceLights) {
        if (poly.texture != templ.texture) {
            continue;
        }
        if (templ.surfaceLightGroup != 0 && poly.surfaceLightGroup != templ.surfaceLightGroup) {
            continue;
        }
        return true;
    }
    return false;
}

static Vector3 AverageRectLighting(const LightmapPage& page,
                                   const FaceRect& rect,
                                   const std::vector<uint8_t>& coverage,
                                   const Vector3& ambientColor)
{
    const size_t pixelCount = (size_t)page.width * (size_t)page.height;
    if (coverage.size() < pixelCount || page.pixels.size() < pixelCount * 4) {
        return Vector3Zero();
    }

    const uint8_t kFullCoverage = (uint8_t)(g_aaGrid * g_aaGrid);
    Vector3 fullAccum = Vector3Zero();
    size_t fullCount = 0;
    Vector3 validAccum = Vector3Zero();
    size_t validCount = 0;
    for (int ly = 0; ly < rect.gpu.h; ++ly) {
        for (int lx = 0; lx < rect.gpu.w; ++lx) {
            const size_t pixelIndex = (size_t)(rect.gpu.y + ly) * (size_t)page.width + (size_t)(rect.gpu.x + lx);
            if (pixelIndex >= pixelCount || coverage[pixelIndex] == 0) {
                continue;
            }
            const size_t off = pixelIndex * 4;
            const Vector3 sample = {
                page.pixels[off + 0],
                page.pixels[off + 1],
                page.pixels[off + 2]
            };
            validAccum = Vector3Add(validAccum, sample);
            ++validCount;
            if (coverage[pixelIndex] == kFullCoverage) {
                fullAccum = Vector3Add(fullAccum, sample);
                ++fullCount;
            }
        }
    }

    Vector3 avg = Vector3Zero();
    if (fullCount > 0) {
        avg = Vector3Scale(fullAccum, 1.0f / (float)fullCount);
    } else if (validCount > 0) {
        avg = Vector3Scale(validAccum, 1.0f / (float)validCount);
    }
    avg.x = std::max(0.0f, avg.x - ambientColor.x);
    avg.y = std::max(0.0f, avg.y - ambientColor.y);
    avg.z = std::max(0.0f, avg.z - ambientColor.z);
    return avg;
}

static Vector3 AverageStitchedSourceFaceLighting(const StitchedSourceFaceCanvas& canvas,
                                                 const Vector3& ambientColor)
{
    if (canvas.width <= 0 || canvas.height <= 0) {
        return Vector3Zero();
    }

    const size_t pixelCount = (size_t)canvas.width * (size_t)canvas.height;
    Vector3 accum = Vector3Zero();
    size_t validCount = 0;
    for (size_t i = 0; i < pixelCount; ++i) {
        if (!canvas.valid[i]) {
            continue;
        }
        accum.x += canvas.pixelsR[i];
        accum.y += canvas.pixelsG[i];
        accum.z += canvas.pixelsB[i];
        ++validCount;
    }

    if (validCount == 0) {
        return Vector3Zero();
    }

    const float invCount = 1.0f / (float)validCount;
    Vector3 avg = {
        accum.x * invCount,
        accum.y * invCount,
        accum.z * invCount
    };
    avg.x = std::max(0.0f, avg.x - ambientColor.x);
    avg.y = std::max(0.0f, avg.y - ambientColor.y);
    avg.z = std::max(0.0f, avg.z - ambientColor.z);
    return avg;
}

static std::vector<SurfaceLightEmitter> BuildIndirectBounceEmitters(const std::vector<MapPolygon>& sourcePolys,
                                                                    const std::vector<FaceRect>& rects,
                                                                    const std::vector<LightmapPage>& pages,
                                                                    const std::vector<std::vector<uint8_t>>& coverageMasks,
                                                                    const std::vector<SurfaceLightTemplate>& surfaceLights,
                                                                    const std::unordered_map<std::string, Vector3>& textureBounceColors,
                                                                    const Vector3& ambientColor,
                                                                    const std::vector<BrushSolid>& solids,
                                                                    const LightBakeSettings& settings,
                                                                    int bounceDepth)
{
    std::vector<SurfaceLightEmitter> bounceEmitters;
    const float emittedScale = INDIRECT_BOUNCE_REFLECTANCE;
    const float bounceTransportScale = std::max(0.0f, settings.bounceScale * 0.5f);
    if (emittedScale <= 0.0f || bounceTransportScale <= 0.0f) {
        return bounceEmitters;
    }
    const float sampleSpacing = std::max(1.0f, settings.bounceLightSubdivision);
    const auto rectsBySourcePoly = GroupRectsBySourcePoly(rects);
    bounceEmitters.reserve(rectsBySourcePoly.size());
    size_t adjustedSampleCount = 0;
    size_t culledSampleCount = 0;
    for (const auto& [sourcePolyIndex, rectGroup] : rectsBySourcePoly) {
        if (sourcePolyIndex >= sourcePolys.size()) {
            continue;
        }
        const MapPolygon& sourcePoly = sourcePolys[sourcePolyIndex];
        if (!PolygonCanEmitBounceForLighting(sourcePoly, surfaceLights)) {
            continue;
        }
        if (rectGroup.empty()) {
            continue;
        }

        StitchedSourceFaceCanvas canvas;
        if (!BuildStitchedSourceFaceCanvas(rects, pages, coverageMasks, rectGroup, &canvas)) {
            continue;
        }

        const Vector3 avgDirect = AverageStitchedSourceFaceLighting(canvas, ambientColor);
        const float maxChannel = std::max(avgDirect.x, std::max(avgDirect.y, avgDirect.z));
        if (maxChannel * emittedScale * bounceTransportScale < INDIRECT_BOUNCE_EMITTER_THRESHOLD) {
            continue;
        }

        const float area = std::max(1.0f, PolygonArea(sourcePoly.verts));
        const Vector3 textureBounceColor = ComputeTextureBounceColor(
            sourcePoly.texture, textureBounceColors, settings.bounceColorScale);
        size_t emitterAdjustedSamples = 0;
        size_t emitterCulledSamples = 0;
        const std::vector<Vector3> samplePoints = GenerateFixedSurfaceEmitterSamples(
            sourcePoly,
            sampleSpacing,
            INDIRECT_BOUNCE_SURFACE_OFFSET,
            solids,
            &emitterAdjustedSamples,
            &emitterCulledSamples);
        adjustedSampleCount += emitterAdjustedSamples;
        culledSampleCount += emitterCulledSamples;
        if (samplePoints.empty()) {
            continue;
        }
        SurfaceLightEmitter bounce{};
        bounce.baseLight.position = AveragePoints(samplePoints);
        bounce.baseLight.color = {
            avgDirect.x * textureBounceColor.x * emittedScale,
            avgDirect.y * textureBounceColor.y * emittedScale,
            avgDirect.z * textureBounceColor.z * emittedScale
        };
        bounce.baseLight.emissionNormal = sourcePoly.normal;
        bounce.baseLight.directional = 1;
        bounce.baseLight.ignoreOccluderGroup = -1;
        bounce.baseLight.attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
        bounce.baseLight.dirt = -1;
        bounce.surfaceNormal = sourcePoly.normal;
        bounce.samplePoints = samplePoints;
        bounce.bounds = ComputeBoundsFromPoints(bounce.samplePoints);
        bounce.sampleIntensityScale = ComputeEmitterSampleIntensityScale(
            area,
            bounce.samplePoints.size());
        bounce.transportScale = bounceTransportScale;
        bounce.attenuationScale = 1.0f;
        bounce.hotspotClamp = INDIRECT_SURFACE_HOTSPOT_CLAMP;
        bounce.baseLight.intensity = EstimateSurfaceEmitterCullRadius(
            bounce.baseLight.color,
            area,
            bounce.transportScale,
            bounce.attenuationScale,
            bounce.hotspotClamp);
        bounce.surfaceArea = area;
        bounce.bounceDepth = bounceDepth;
        bounceEmitters.push_back(std::move(bounce));
    }
    if (adjustedSampleCount > 0 || culledSampleCount > 0) {
        printf("[Lightmap] bounce pass %d emitter samples: %zu adjusted, %zu culled.\n",
               bounceDepth, adjustedSampleCount, culledSampleCount);
        fflush(stdout);
    }
    return bounceEmitters;
}

static LightmapPage MakeBlankPageLike(const LightmapPage& src) {
    LightmapPage page;
    page.width = src.width;
    page.height = src.height;
    page.pixels.assign((size_t)page.width * (size_t)page.height * 4, 0.0f);
    return page;
}

static void AddPagePixels(LightmapPage& dst, const LightmapPage& src) {
    if (dst.pixels.size() != src.pixels.size()) {
        return;
    }
    for (size_t i = 0; i + 3 < dst.pixels.size(); i += 4) {
        dst.pixels[i + 0] += src.pixels[i + 0];
        dst.pixels[i + 1] += src.pixels[i + 1];
        dst.pixels[i + 2] += src.pixels[i + 2];
        dst.pixels[i + 3] = 1.0f;
    }
}

static float EffectiveOutputRangeScale(const LightBakeSettings& settings)
{
    return std::max(0.0f, settings.rangeScale) / 0.5f;
}

static float EffectiveOutputGamma(const LightBakeSettings& settings)
{
    return std::max(0.01f, settings.lightmapGamma);
}

static void ApplyLightmapOutputConditioning(std::vector<LightmapPage>& pages,
                                            const std::vector<std::vector<uint8_t>>& validMasks,
                                            const LightBakeSettings& settings)
{
    const float rangeScale = EffectiveOutputRangeScale(settings);
    const float maxLight = std::max(0.0f, settings.maxLight);
    const float lightmapGamma = EffectiveOutputGamma(settings);
    const bool clampMaxLight = maxLight > 0.0f;
    const bool scaleRange = fabsf(rangeScale - 1.0f) > 1e-6f;
    const bool applyGamma = fabsf(lightmapGamma - 1.0f) > 1e-6f;
    if (!clampMaxLight && !scaleRange && !applyGamma) {
        return;
    }

    for (size_t pageIndex = 0; pageIndex < pages.size() && pageIndex < validMasks.size(); ++pageIndex) {
        LightmapPage& page = pages[pageIndex];
        const std::vector<uint8_t>& valid = validMasks[pageIndex];
        const size_t pixelCount = (size_t)page.width * (size_t)page.height;
        if (page.pixels.size() < pixelCount * 4 || valid.size() < pixelCount) {
            continue;
        }

        for (size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            if (!valid[pixelIndex]) {
                continue;
            }

            float r = std::max(0.0f, page.pixels[pixelIndex * 4 + 0]);
            float g = std::max(0.0f, page.pixels[pixelIndex * 4 + 1]);
            float b = std::max(0.0f, page.pixels[pixelIndex * 4 + 2]);

            if (clampMaxLight) {
                const float peak = std::max(r, std::max(g, b));
                if (peak > maxLight && peak > 1e-6f) {
                    const float scale = maxLight / peak;
                    r *= scale;
                    g *= scale;
                    b *= scale;
                }
            }

            if (scaleRange) {
                r *= rangeScale;
                g *= rangeScale;
                b *= rangeScale;
            }

            if (applyGamma) {
                r = powf(std::max(0.0f, r), 1.0f / lightmapGamma);
                g = powf(std::max(0.0f, g), 1.0f / lightmapGamma);
                b = powf(std::max(0.0f, b), 1.0f / lightmapGamma);
            }

            page.pixels[pixelIndex * 4 + 0] = std::max(0.0f, r);
            page.pixels[pixelIndex * 4 + 1] = std::max(0.0f, g);
            page.pixels[pixelIndex * 4 + 2] = std::max(0.0f, b);
            page.pixels[pixelIndex * 4 + 3] = 1.0f;
        }
    }
}

static Vector3 ComputeLuxelPlanePoint(const FaceRect& rect, float ju, float jv) {
    return Vector3Add(
        rect.gpu.origin,
        Vector3Add(Vector3Scale(rect.gpu.axisU, ju * rect.gpu.luxelSize),
                   Vector3Scale(rect.gpu.axisV, jv * rect.gpu.luxelSize)));
}

static Vector3 OffsetPointAlongSurfaceNormal(const Vector3& planePoint,
                                             const Vector3& faceNormal,
                                             float offsetDistance)
{
    return Vector3Add(planePoint,
                      Vector3Scale(Vector3Normalize(faceNormal), offsetDistance));
}

static Vector3 OffsetSamplePointOffSurface(const Vector3& planePoint,
                                           const Vector3& faceNormal,
                                           float offsetDistance) {
    return OffsetPointAlongSurfaceNormal(planePoint, faceNormal, offsetDistance);
}

static Vector3 BuildFaceLocalShadowRayOrigin(const Vector3& planePoint,
                                             const Vector3& faceNormal)
{
    const Vector3 rayNormal = (Vector3LengthSq(faceNormal) > 1e-8f)
        ? Vector3Normalize(faceNormal)
        : Vector3Zero();
    return Vector3Add(planePoint, Vector3Scale(rayNormal, SHADOW_BIAS));
}

static float ComputeEdgeAwareNearHitT(const std::vector<Vector2>& poly2d,
                                      float ju,
                                      float jv)
{
    (void)poly2d;
    (void)ju;
    (void)jv;
    return OCCLUSION_NEAR_TMIN;
}

static float ComputeDirtAttenuation(float occlusionRatio, float dirtScale, float dirtGain) {
    const float scaled = std::clamp(occlusionRatio * std::max(0.0f, dirtScale), 0.0f, 1.0f);
    const float gained = powf(scaled, std::max(0.01f, dirtGain));
    return std::clamp(1.0f - gained, 0.0f, 1.0f);
}

static float ComputeDirtOcclusionRatio(const OccluderSet& occ,
                                       const std::vector<BrushSolid>& shadowSolids,
                                       const Vector3& samplePoint,
                                       const Vector3& sampleNormal,
                                       const LightBakeSettings& settings)
{
    const float depth = std::max(1.0f, settings.dirtDepth);
    const Vector3 normal = Vector3Normalize(sampleNormal);
    Vector3 tangent{};
    Vector3 bitangent{};
    FaceBasis(normal, tangent, bitangent);

    uint32_t seed = HashLightSeed(samplePoint, 7919);
    float accumulatedDistance = 0.0f;
    for (int i = 0; i < DIRT_RAY_COUNT; ++i) {
        const Vector3 localDir = BuildEricwDirtVector(settings, i, seed);
        const Vector3 dir = Vector3Normalize(TransformToTangentSpace(normal, tangent, bitangent, localDir));
        const float triHitDistance = LightmapTraceClosestHitDistance(
            occ,
            samplePoint,
            dir,
            LightmapTraceQuery{
                OCCLUSION_NEAR_TMIN,
                depth,
                -1,
                -1
            });
        const float solidHitDistance = ClosestBrushSolidHitDistance(
            shadowSolids, samplePoint, dir, OCCLUSION_NEAR_TMIN, depth);
        const float hitDistance = std::min(triHitDistance, solidHitDistance);
        accumulatedDistance += std::min(depth, hitDistance);
    }
    const float avgHitDistance = accumulatedDistance / (float)DIRT_RAY_COUNT;
    return std::clamp(1.0f - (avgHitDistance / depth), 0.0f, 1.0f);
}

static bool TryRepairSampleCandidate(const std::vector<BrushSolid>& solids,
                                     const Vector3& planePoint,
                                     const Vector3& faceNormal,
                                     float offsetDistance,
                                     RepairedSamplePoint* ioSample)
{
    if (!ioSample) {
        return false;
    }

    ioSample->valid = false;
    ioSample->planePoint = planePoint;
    ioSample->samplePoint = OffsetPointAlongSurfaceNormal(planePoint, faceNormal, offsetDistance);
    if (!PointInsideAnySolid(solids, ioSample->samplePoint, SOLID_REPAIR_EPSILON)) {
        ioSample->valid = true;
        return true;
    }

    for (int x = -1; x <= 1; x += 2) {
        for (int y = -1; y <= 1; y += 2) {
            for (int z = -1; z <= 1; z += 2) {
                const Vector3 jitter = {
                    (float)x * SAMPLE_REPAIR_JITTER,
                    (float)y * SAMPLE_REPAIR_JITTER,
                    (float)z * SAMPLE_REPAIR_JITTER
                };
                const Vector3 jitteredPoint = Vector3Add(ioSample->samplePoint, jitter);
                if (!PointInsideAnySolid(solids, jitteredPoint, SOLID_REPAIR_EPSILON)) {
                    ioSample->samplePoint = jitteredPoint;
                    ioSample->valid = true;
                    return true;
                }
            }
        }
    }

    return false;
}

static void FaceBasisFromNormal(const Vector3& normal, Vector3* axisU, Vector3* axisV) {
    if (!axisU || !axisV) {
        return;
    }
    FaceBasis(normal, *axisU, *axisV);
}

static void InitializeRepairOwnerFromSourcePoly(const std::vector<RepairSourcePoly>& repairPolys,
                                                uint32_t sourcePolyIndex,
                                                const Vector3& planePoint,
                                                const Vector3& fallbackNormal,
                                                RepairedSamplePoint* sample)
{
    if (!sample) {
        return;
    }
    if (sourcePolyIndex < repairPolys.size() && Vector3LengthSq(repairPolys[sourcePolyIndex].normal) > 1e-8f) {
        const RepairSourcePoly& poly = repairPolys[sourcePolyIndex];
        SetRepairedSampleOwnerContext(sample,
                                      sourcePolyIndex,
                                      planePoint,
                                      poly.normal,
                                      poly.axisU,
                                      poly.axisV);
        return;
    }

    Vector3 axisU{};
    Vector3 axisV{};
    FaceBasisFromNormal(fallbackNormal, &axisU, &axisV);
    SetRepairedSampleOwnerContext(sample,
                                  sourcePolyIndex,
                                  planePoint,
                                  fallbackNormal,
                                  axisU,
                                  axisV);
}

static bool SourcePolyVisited(const std::vector<uint32_t>& visitedPath, uint32_t sourcePolyIndex) {
    return std::find(visitedPath.begin(), visitedPath.end(), sourcePolyIndex) != visitedPath.end();
}

static bool TryRecursiveRepairWalk(const std::vector<RepairSourcePoly>& repairPolys,
                                   const std::vector<BrushSolid>& solids,
                                   uint32_t sourcePolyIndex,
                                   const Vector3& seedPoint,
                                   float luxelSize,
                                   float sampleOffset,
                                   int depth,
                                   std::vector<uint32_t>& visitedPath,
                                   RepairedSamplePoint* outSample)
{
    if (!outSample || sourcePolyIndex >= repairPolys.size()) {
        return false;
    }

    const RepairSourcePoly& poly = repairPolys[sourcePolyIndex];
    if (poly.poly2d.size() < 3 || Vector3LengthSq(poly.normal) <= 1e-8f) {
        return false;
    }

    const Vector3 projected = ProjectPointOntoPlane(seedPoint, poly.planePoint, poly.normal);
    const Vector2 projectedUv = {
        Vector3DotProduct(projected, poly.axisU),
        Vector3DotProduct(projected, poly.axisV)
    };
    const Vector3 faceSeedPoint = OffsetSamplePointOffSurface(projected, poly.normal, sampleOffset);
    const bool insideFace = InsidePoly2D(poly.poly2d, projectedUv.x, projectedUv.y);
    if (insideFace) {
        RepairedSamplePoint candidate{};
        SetRepairedSampleOwnerContext(&candidate,
                                      sourcePolyIndex,
                                      projected,
                                      poly.normal,
                                      poly.axisU,
                                      poly.axisV);
        if (TryRepairSampleCandidate(solids, projected, poly.normal, sampleOffset, &candidate)) {
            *outSample = candidate;
            return true;
        }
    }

    if (depth < SAMPLE_REPAIR_RECURSION_MAX && !poly.neighbors.empty()) {
        if (insideFace) {
            struct NeighborCandidate {
                float edgeDist = 0.0f;
                uint32_t sourcePolyIndex = 0;
            };

            std::vector<NeighborCandidate> candidates;
            candidates.reserve(poly.neighbors.size());
            const float edgeDistanceLimit = std::max(1.0f, luxelSize * 1.5f);
            for (const RepairSourceNeighbor& neighbor : poly.neighbors) {
                if (neighbor.sourcePolyIndex >= repairPolys.size() ||
                    neighbor.edgeIndex < 0 ||
                    SourcePolyVisited(visitedPath, neighbor.sourcePolyIndex))
                {
                    continue;
                }

                const size_t edgeIndex = (size_t)neighbor.edgeIndex;
                if (edgeIndex >= poly.poly2d.size()) {
                    continue;
                }
                const Vector2& a = poly.poly2d[edgeIndex];
                const Vector2& b = poly.poly2d[(edgeIndex + 1) % poly.poly2d.size()];
                const float edgeDist = sqrtf(DistSqPointSegment2D(projectedUv, a, b));
                if (edgeDist > edgeDistanceLimit) {
                    continue;
                }
                candidates.push_back({ edgeDist, neighbor.sourcePolyIndex });
            }

            std::sort(candidates.begin(), candidates.end(), [](const NeighborCandidate& a, const NeighborCandidate& b) {
                if (fabsf(a.edgeDist - b.edgeDist) > 1e-4f) {
                    return a.edgeDist < b.edgeDist;
                }
                return a.sourcePolyIndex < b.sourcePolyIndex;
            });

            for (const NeighborCandidate& candidate : candidates) {
                visitedPath.push_back(candidate.sourcePolyIndex);
                if (TryRecursiveRepairWalk(repairPolys,
                                           solids,
                                           candidate.sourcePolyIndex,
                                           faceSeedPoint,
                                           luxelSize,
                                           sampleOffset,
                                           depth + 1,
                                           visitedPath,
                                           outSample))
                {
                    return true;
                }
                visitedPath.pop_back();
            }
        } else {
            const int bestEdgeIndex = FindBestRepairEdgeIndex(poly, projectedUv);
            if (bestEdgeIndex >= 0) {
                for (const RepairSourceNeighbor& neighbor : poly.neighbors) {
                    if (neighbor.edgeIndex != bestEdgeIndex ||
                        neighbor.sourcePolyIndex >= repairPolys.size() ||
                        SourcePolyVisited(visitedPath, neighbor.sourcePolyIndex))
                    {
                        continue;
                    }

                    visitedPath.push_back(neighbor.sourcePolyIndex);
                    if (TryRecursiveRepairWalk(repairPolys,
                                               solids,
                                               neighbor.sourcePolyIndex,
                                               faceSeedPoint,
                                               luxelSize,
                                               sampleOffset,
                                               depth + 1,
                                               visitedPath,
                                               outSample))
                    {
                        return true;
                    }
                    visitedPath.pop_back();
                }
            }
        }
    }

    const Vector2 snappedUv = ClosestPointOnPolyBoundary2D(poly.poly2d, projectedUv);
    const Vector3 snappedPoint = Vector3Add(
        projected,
        Vector3Add(Vector3Scale(poly.axisU, snappedUv.x - projectedUv.x),
                   Vector3Scale(poly.axisV, snappedUv.y - projectedUv.y)));
    if (Vector3LengthSq(Vector3Subtract(snappedPoint, projected)) <= (luxelSize * luxelSize)) {
        RepairedSamplePoint boundarySample{};
        SetRepairedSampleOwnerContext(&boundarySample,
                                      sourcePolyIndex,
                                      snappedPoint,
                                      poly.normal,
                                      poly.axisU,
                                      poly.axisV);
        if (TryRepairSampleCandidate(solids, snappedPoint, poly.normal, sampleOffset, &boundarySample)) {
            *outSample = boundarySample;
            return true;
        }
    }

    return false;
}

static RepairedSamplePoint RepairSamplePoint(const FaceRect& rect,
                                             const std::vector<RepairSourcePoly>& repairPolys,
                                             const std::vector<BrushSolid>& solids,
                                             const Vector3& planePoint,
                                             float luxelSize,
                                             float sampleOffset)
{
    RepairedSamplePoint repaired{};
    const Vector3 baseNormal = (rect.sourcePolyIndex < repairPolys.size() &&
                                Vector3LengthSq(repairPolys[rect.sourcePolyIndex].normal) > 1e-8f)
        ? repairPolys[rect.sourcePolyIndex].normal
        : rect.gpu.normal;
    InitializeRepairOwnerFromSourcePoly(repairPolys,
                                        rect.sourcePolyIndex,
                                        planePoint,
                                        baseNormal,
                                        &repaired);
    repaired.samplePoint = OffsetSamplePointOffSurface(planePoint, baseNormal, sampleOffset);
    if (!PointInsideAnySolid(solids, repaired.samplePoint, SOLID_REPAIR_EPSILON)) {
        repaired.valid = true;
        return repaired;
    }

    if (TryRepairSampleCandidate(solids, planePoint, baseNormal, sampleOffset, &repaired)) {
        return repaired;
    }

    if (rect.sourcePolyIndex < repairPolys.size()) {
        std::vector<uint32_t> visitedPath;
        visitedPath.reserve(SAMPLE_REPAIR_RECURSION_MAX + 2);
        visitedPath.push_back(rect.sourcePolyIndex);
        RepairedSamplePoint recursivelyRepaired{};
        if (TryRecursiveRepairWalk(repairPolys,
                                   solids,
                                   rect.sourcePolyIndex,
                                   repaired.samplePoint,
                                   luxelSize,
                                   sampleOffset,
                                   0,
                                   visitedPath,
                                   &recursivelyRepaired))
        {
            return recursivelyRepaired;
        }
    }

    return repaired;
}

static bool SkyDomeUsesDirt(const LightBakeSettings& settings) {
    const int dirtSetting = (settings.sunlight2Dirt == -2) ? settings.dirt : settings.sunlight2Dirt;
    return dirtSetting == 1;
}

static Vector3 ComputeSkyDomeContribution(const OccluderSet& occ,
                                          const std::vector<BrushSolid>& shadowSolids,
                                          uint32_t sourcePolyIndex,
                                          const Vector3& visibilityPlanePoint,
                                          const Vector3& visibilityFaceNormal,
                                          const Vector3& samplePoint,
                                          const Vector3& sampleNormal,
                                          float nearHitT,
                                          float skyTraceDistance,
                                          float dirtOcclusion,
                                          const LightBakeSettings& settings)
{
    Vector3 contrib = Vector3Zero();
    const int sampleCount = EricwSkyDomeSampleCount();
    const float upperPerSample = (settings.sunlight2Intensity > 0.0f)
        ? settings.sunlight2Intensity / (300.0f * (float)sampleCount)
        : 0.0f;
    const float lowerPerSample = (settings.sunlight3Intensity > 0.0f)
        ? settings.sunlight3Intensity / (300.0f * (float)sampleCount)
        : 0.0f;
    if (upperPerSample <= 0.0f && lowerPerSample <= 0.0f) {
        return contrib;
    }

    uint32_t upperSeed = HashLightSeed(samplePoint, 13007);
    uint32_t lowerSeed = HashLightSeed(samplePoint, 17011);
    const float upperRotation = RandomFloat01(upperSeed) * PI * 2.0f;
    const float lowerRotation = RandomFloat01(lowerSeed) * PI * 2.0f;
    const bool useDirt = SkyDomeUsesDirt(settings);
    const float dirtScale = useDirt ? ComputeDirtAttenuation(dirtOcclusion, settings.dirtScale, settings.dirtGain) : 1.0f;

    for (int i = 0; i < sampleCount; ++i) {
        if (upperPerSample > 0.0f) {
            const Vector3 dir = EricwSkyDomeDirection(true, i, upperRotation);
            const Vector3 ro = BuildFaceLocalShadowRayOrigin(visibilityPlanePoint, visibilityFaceNormal);
            const LightmapTraceQuery skyQuery{
                nearHitT,
                skyTraceDistance,
                -1,
                (int)sourcePolyIndex
            };
            const float solidHitDistance = ClosestBrushSolidHitDistance(shadowSolids, ro, dir, nearHitT, skyTraceDistance);
            const LightmapTraceHit hit = LightmapTraceClosestHit(occ, ro, dir, skyQuery);
            const bool visible = (settings.sunlightNoSky != 0)
                ? (hit.kind == LIGHTMAP_TRACE_HIT_NONE && solidHitDistance >= skyTraceDistance)
                : (hit.kind == LIGHTMAP_TRACE_HIT_SKY && solidHitDistance >= hit.distance - 0.05f);
            if (visible) {
                const float incidence = EvaluateAngleScale(settings.sunlightAngleScale, Vector3DotProduct(sampleNormal, dir));
                const float scale = upperPerSample * incidence * dirtScale;
                contrib = Vector3Add(contrib, Vector3Scale(settings.sunlight2Color, scale));
            }
        }
        if (lowerPerSample > 0.0f) {
            const Vector3 dir = EricwSkyDomeDirection(false, i, lowerRotation);
            const Vector3 ro = BuildFaceLocalShadowRayOrigin(visibilityPlanePoint, visibilityFaceNormal);
            const LightmapTraceQuery skyQuery{
                nearHitT,
                skyTraceDistance,
                -1,
                (int)sourcePolyIndex
            };
            const float solidHitDistance = ClosestBrushSolidHitDistance(shadowSolids, ro, dir, nearHitT, skyTraceDistance);
            const LightmapTraceHit hit = LightmapTraceClosestHit(occ, ro, dir, skyQuery);
            const bool visible = (settings.sunlightNoSky != 0)
                ? (hit.kind == LIGHTMAP_TRACE_HIT_NONE && solidHitDistance >= skyTraceDistance)
                : (hit.kind == LIGHTMAP_TRACE_HIT_SKY && solidHitDistance >= hit.distance - 0.05f);
            if (visible) {
                const float incidence = EvaluateAngleScale(settings.sunlightAngleScale, Vector3DotProduct(sampleNormal, dir));
                const float scale = lowerPerSample * incidence * dirtScale;
                contrib = Vector3Add(contrib, Vector3Scale(settings.sunlight3Color, scale));
            }
        }
    }

    return contrib;
}

static Vector3 ComputeDirectLightContribution(const PointLight& light,
                                              const Vector3& lightPosition,
                                              const OccluderSet& occ,
                                              const std::vector<BrushSolid>& shadowSolids,
                                              uint32_t ownerSourcePolyIndex,
                                              const Vector3& visibilityPlanePoint,
                                              const Vector3& visibilityFaceNormal,
                                              const Vector3& samplePoint,
                                              const Vector3& sampleNormal,
                                              float nearHitT,
                                              float dirtOcclusion,
                                              const LightBakeSettings& settings)
{
    Vector3 dir{};
    float dist = 0.0f;
    float att = 1.0f;
    if (IsParallelLight(light)) {
        dir = Vector3Normalize(light.parallelDirection);
        dist = std::max(1.0f, light.intensity);
    } else {
        const Vector3 toL = Vector3Subtract(lightPosition, samplePoint);
        dist = Vector3Length(toL);
        if (dist > light.intensity || dist < 1e-3f) {
            return Vector3Zero();
        }
        dir = Vector3Scale(toL, 1.0f / dist);
        att = EvaluateLightAttenuation(light, dist);
        if (att <= 0.0f) {
            return Vector3Zero();
        }
    }

    float emit = 1.0f;
    if (light.directional) {
        const Vector3 lightToSurface = Vector3Scale(dir, -1.0f);
        emit = Vector3DotProduct(light.emissionNormal, lightToSurface);
        if (emit <= 0.0f) {
            return Vector3Zero();
        }
    }

    const float spot = EvaluateSpotlightFactor(light, dir);
    if (spot <= 0.0f) {
        return Vector3Zero();
    }

    const float incidence = EvaluateIncidenceScale(light, Vector3DotProduct(sampleNormal, dir));
    if (incidence <= 0.0f) {
        return Vector3Zero();
    }

    const Vector3 ro = BuildFaceLocalShadowRayOrigin(visibilityPlanePoint, visibilityFaceNormal);
    const LightmapTraceQuery shadowQuery{
        nearHitT,
        std::max(0.0f, dist - (SHADOW_BIAS * 2.0f)),
        light.ignoreOccluderGroup,
        (int)ownerSourcePolyIndex
    };
    const float solidHitDistance = ClosestBrushSolidHitDistance(
        shadowSolids, ro, dir, shadowQuery.minHitT, shadowQuery.maxHitT);
    if (light.requiresSkyVisibility != 0) {
        const LightmapTraceHit hit = LightmapTraceClosestHit(occ, ro, dir, shadowQuery);
        if (hit.kind != LIGHTMAP_TRACE_HIT_SKY || solidHitDistance < hit.distance - 0.05f) {
            return Vector3Zero();
        }
    } else if (solidHitDistance < shadowQuery.maxHitT || LightmapTraceOccluded(occ, ro, dir, shadowQuery)) {
        return Vector3Zero();
    }

    const float dirt = LightUsesDirt(light, settings)
        ? ComputeDirtAttenuation(dirtOcclusion, EffectiveLightDirtScale(light, settings), EffectiveLightDirtGain(light, settings))
        : 1.0f;
    return Vector3Scale(light.color, emit * spot * incidence * att * dirt);
}

static Vector3 ComputeSurfaceEmitterContribution(const SurfaceLightEmitter& emitter,
                                                 const Vector3& emitterSamplePoint,
                                                 const OccluderSet& occ,
                                                 const std::vector<BrushSolid>& shadowSolids,
                                                 uint32_t ownerSourcePolyIndex,
                                                 const Vector3& visibilityPlanePoint,
                                                 const Vector3& visibilityFaceNormal,
                                                 const Vector3& samplePoint,
                                                 const Vector3& sampleNormal,
                                                 float nearHitT,
                                                 float dirtOcclusion,
                                                 const LightBakeSettings& settings)
{
    const Vector3 toLight = Vector3Subtract(emitterSamplePoint, samplePoint);
    const float dist = Vector3Length(toLight);
    if (dist > emitter.baseLight.intensity || dist < 1e-3f) {
        return Vector3Zero();
    }

    const Vector3 dirToLight = Vector3Scale(toLight, 1.0f / dist);
    const Vector3 lightToSurface = Vector3Scale(dirToLight, -1.0f);

    float geometric = 1.0f;
    const float receiverDot = Vector3DotProduct(sampleNormal, dirToLight);
    if (emitter.omnidirectional) {
        geometric = std::max(0.0f, receiverDot * 0.5f);
    } else {
        float emitterDot = Vector3DotProduct(emitter.surfaceNormal, lightToSurface);
        if (emitterDot < -LIGHT_ANGLE_EPSILON || receiverDot < -LIGHT_ANGLE_EPSILON) {
            return Vector3Zero();
        }
        if (emitter.rescale) {
            emitterDot = 0.5f + emitterDot * 0.5f;
            const float rescaledReceiverDot = 0.5f + receiverDot * 0.5f;
            geometric = std::max(0.0f, emitterDot * rescaledReceiverDot);
        } else {
            geometric = std::max(0.0f, emitterDot * receiverDot);
        }
    }
    if (geometric <= 0.0f) {
        return Vector3Zero();
    }

    const float spot = EvaluateSpotlightFactor(emitter.baseLight, dirToLight);
    if (spot <= 0.0f) {
        return Vector3Zero();
    }

    const float falloff = EvaluateSurfaceEmitterDistanceFalloff(emitter, dist);
    if (falloff <= 0.0f) {
        return Vector3Zero();
    }

    const float unshadowedScale = geometric * spot * falloff;
    const float peakUnshadowed = std::max(emitter.baseLight.color.x,
                                          std::max(emitter.baseLight.color.y, emitter.baseLight.color.z)) * unshadowedScale;
    if (peakUnshadowed <= SURFACE_EMITTER_TRACE_THRESHOLD) {
        return Vector3Zero();
    }

    const Vector3 ro = BuildFaceLocalShadowRayOrigin(visibilityPlanePoint, visibilityFaceNormal);
    const LightmapTraceQuery shadowQuery{
        nearHitT,
        std::max(0.0f, dist - (SHADOW_BIAS * 2.0f)),
        emitter.baseLight.ignoreOccluderGroup,
        (int)ownerSourcePolyIndex
    };
    const float solidHitDistance = ClosestBrushSolidHitDistance(
        shadowSolids, ro, dirToLight, shadowQuery.minHitT, shadowQuery.maxHitT);
    if (emitter.baseLight.requiresSkyVisibility != 0) {
        const LightmapTraceHit hit = LightmapTraceClosestHit(occ, ro, dirToLight, shadowQuery);
        if (hit.kind != LIGHTMAP_TRACE_HIT_SKY || solidHitDistance < hit.distance - 0.05f) {
            return Vector3Zero();
        }
    } else if (solidHitDistance < shadowQuery.maxHitT || LightmapTraceOccluded(occ, ro, dirToLight, shadowQuery)) {
        return Vector3Zero();
    }

    const float dirt = LightUsesDirt(emitter.baseLight, settings)
        ? ComputeDirtAttenuation(dirtOcclusion,
                                 EffectiveLightDirtScale(emitter.baseLight, settings),
                                 EffectiveLightDirtGain(emitter.baseLight, settings))
        : 1.0f;
    return Vector3Scale(emitter.baseLight.color, unshadowedScale * dirt);
}

enum LightmapCPUOnlyFeature : uint32_t {
    LIGHTMAP_CPU_ONLY_NONE = 0u,
    LIGHTMAP_CPU_ONLY_TRACE_HIT_CLASSIFICATION = 1u << 0,
    LIGHTMAP_CPU_ONLY_REPAIRED_SAMPLE_OWNERSHIP = 1u << 1,
    LIGHTMAP_CPU_ONLY_SURFACE_EMITTERS = 1u << 2,
    LIGHTMAP_CPU_ONLY_SAMPLE_POSITION_SEMANTICS = 1u << 3,
    LIGHTMAP_CPU_ONLY_STITCHED_EXTRA_RESOLVE = 1u << 4,
};

static uint32_t GatherCPUOnlyLightingFeatures(uint32_t pageIndex,
                                              const std::vector<FaceRect>& rects,
                                              const std::vector<PhongSourcePoly>& sourcePhongs,
                                              const std::vector<PointLight>& pageLights,
                                              bool usesSurfaceEmitters,
                                              bool allowHybridSurfaceEmitters,
                                              const LightBakeSettings& settings)
{
    // Later refactor phases can light up bits here as soon as a feature lands on
    // the CPU reference path before shader parity exists.
    (void)sourcePhongs;
    uint32_t features = LIGHTMAP_CPU_ONLY_NONE;
    if (settings.sunlightNoSky == 0 &&
        (settings.sunlight2Intensity > 0.0f || settings.sunlight3Intensity > 0.0f)) {
        features |= LIGHTMAP_CPU_ONLY_TRACE_HIT_CLASSIFICATION;
    }
    for (const PointLight& light : pageLights) {
        if (light.requiresSkyVisibility != 0) {
            features |= LIGHTMAP_CPU_ONLY_TRACE_HIT_CLASSIFICATION;
            break;
        }
    }
    bool pageUsesSurfaceEmitters = false;
    for (const FaceRect& rect : rects) {
        if (rect.page != pageIndex) {
            continue;
        }
        if (usesSurfaceEmitters && !pageUsesSurfaceEmitters && !rect.surfaceEmitterIndices.empty()) {
            pageUsesSurfaceEmitters = true;
        }
        if (pageUsesSurfaceEmitters) {
            break;
        }
    }
    if (pageUsesSurfaceEmitters && !allowHybridSurfaceEmitters) {
        features |= LIGHTMAP_CPU_ONLY_SURFACE_EMITTERS;
    }
    return features;
}

static bool PageUsesCPUOnlyLightingFeatures(uint32_t pageIndex,
                                            const std::vector<FaceRect>& rects,
                                            const std::vector<PhongSourcePoly>& sourcePhongs,
                                            const std::vector<PointLight>& pageLights,
                                            bool usesSurfaceEmitters,
                                            bool allowHybridSurfaceEmitters,
                                            const LightBakeSettings& settings,
                                            std::string* reason)
{
    if (reason) {
        reason->clear();
    }
    const uint32_t requiredCpuFeatures = GatherCPUOnlyLightingFeatures(
        pageIndex, rects, sourcePhongs, pageLights, usesSurfaceEmitters, allowHybridSurfaceEmitters, settings);
    if ((requiredCpuFeatures & LIGHTMAP_CPU_ONLY_TRACE_HIT_CLASSIFICATION) != 0u) {
        if (reason) {
            *reason = "page uses trace-hit classification semantics that the compute baker does not support yet";
        }
        return true;
    }
    if ((requiredCpuFeatures & LIGHTMAP_CPU_ONLY_SURFACE_EMITTERS) != 0u) {
        if (reason) {
            *reason = "page uses surface-emitter lighting that the compute baker does not support yet";
        }
        return true;
    }
    return false;
}

static void ShadeRectOversampled(const FaceRect& rect,
                                 const std::vector<PhongSourcePoly>& sourcePhongs,
                                 const std::vector<RepairSourcePoly>& repairPolys,
                                 const std::vector<PointLight>& lights,
                                 const std::vector<SurfaceLightEmitter>* surfaceEmitters,
                                 const std::vector<uint32_t>& rectLightIndices,
                                 const std::vector<uint32_t>& rectSurfaceEmitterIndices,
                                 const OccluderSet& occ,
                                 const std::vector<BrushSolid>& repairSolids,
                                 const LightBakeSettings& settings,
                                 float skyTraceDistance,
                                 OversampledRectBuffer* outBuffer)
{
    if (!outBuffer) {
        return;
    }

    const int aaGrid = g_aaGrid;
    const float invG = 1.0f / (float)aaGrid;
    const int hiW = rect.gpu.w * aaGrid;
    const int hiH = rect.gpu.h * aaGrid;
    const size_t hiPixelCount = (size_t)hiW * (size_t)hiH;
    outBuffer->width = hiW;
    outBuffer->height = hiH;
    outBuffer->opaque.assign(hiPixelCount, 0);
    outBuffer->pixelsR.assign(hiPixelCount, 0.0f);
    outBuffer->pixelsG.assign(hiPixelCount, 0.0f);
    outBuffer->pixelsB.assign(hiPixelCount, 0.0f);

    bool usesDirt = false;
    for (uint32_t lightIndex : rectLightIndices) {
        if (lightIndex >= lights.size()) {
            continue;
        }
        if (LightUsesDirt(lights[lightIndex], settings)) {
            usesDirt = true;
            break;
        }
    }
    if (!usesDirt && surfaceEmitters) {
        for (uint32_t emitterIndex : rectSurfaceEmitterIndices) {
            if (emitterIndex >= surfaceEmitters->size()) {
                continue;
            }
            if (LightUsesDirt((*surfaceEmitters)[emitterIndex].baseLight, settings)) {
                usesDirt = true;
                break;
            }
        }
    }
    usesDirt = usesDirt || SkyDomeUsesDirt(settings);

    for (int ly = 0; ly < rect.gpu.h; ++ly) {
        for (int lx = 0; lx < rect.gpu.w; ++lx) {
            for (int sy = 0; sy < aaGrid; ++sy) {
                for (int sx = 0; sx < aaGrid; ++sx) {
                    const float ju = (lx - LM_PAD) + (sx + 0.5f) * invG;
                    const float jv = (ly - LM_PAD) + (sy + 0.5f) * invG;
                    if (!InsidePoly2D(rect.poly2d, ju, jv)) {
                        continue;
                    }

                    const Vector3 planePoint = ComputeLuxelPlanePoint(rect, ju, jv);
                    const RepairedSamplePoint repairedSample = RepairSamplePoint(
                        rect, repairPolys, repairSolids, planePoint, rect.gpu.luxelSize, settings.surfaceSampleOffset);
                    const Vector3 faceSamplePoint = OffsetSamplePointOffSurface(
                        planePoint, rect.gpu.normal, settings.surfaceSampleOffset);
                    const bool faceSampleInsideSolid = PointInsideAnySolid(repairSolids, faceSamplePoint, SOLID_REPAIR_EPSILON);
                    if (faceSampleInsideSolid && !repairedSample.valid) {
                        const int hiX = lx * aaGrid + sx;
                        const int hiY = ly * aaGrid + sy;
                        const size_t hiIndex = (size_t)hiY * (size_t)hiW + (size_t)hiX;
                        outBuffer->opaque[hiIndex] = 1;
                        outBuffer->pixelsR[hiIndex] = std::max(0.0f, settings.ambientColor.x);
                        outBuffer->pixelsG[hiIndex] = std::max(0.0f, settings.ambientColor.y);
                        outBuffer->pixelsB[hiIndex] = std::max(0.0f, settings.ambientColor.z);
                        continue;
                    }
                    const uint32_t ownerSourcePolyIndex = faceSampleInsideSolid ? repairedSample.sourcePolyIndex : rect.sourcePolyIndex;
                    const Vector3 ownerPlanePoint = faceSampleInsideSolid ? repairedSample.planePoint : planePoint;
                    const Vector3 ownerNormal = (faceSampleInsideSolid && Vector3LengthSq(repairedSample.ownerNormal) > 1e-8f)
                        ? repairedSample.ownerNormal
                        : rect.gpu.normal;
                    const Vector3 samplePoint = faceSampleInsideSolid ? repairedSample.samplePoint : faceSamplePoint;
                    Vector3 sampleNormal = EvaluatePhongNormal(sourcePhongs, ownerSourcePolyIndex, ownerPlanePoint, rect.gpu.luxelSize);
                    if (Vector3LengthSq(sampleNormal) <= 1e-8f) {
                        sampleNormal = ownerNormal;
                    }
                    const float nearHitT = ComputeEdgeAwareNearHitT(rect.poly2d, ju, jv);

                    float cr = settings.ambientColor.x;
                    float cg = settings.ambientColor.y;
                    float cb = settings.ambientColor.z;
                    const float dirtOcclusion = usesDirt
                        ? ComputeDirtOcclusionRatio(occ, repairSolids, samplePoint, sampleNormal, settings)
                        : 0.0f;
                    for (uint32_t lightIndex : rectLightIndices) {
                        if (lightIndex >= lights.size()) {
                            continue;
                        }
                        const PointLight& light = lights[lightIndex];
                        const Vector3 contrib = ComputeDirectLightContribution(
                            light,
                            light.position,
                            occ,
                            repairSolids,
                            ownerSourcePolyIndex,
                            ownerPlanePoint,
                            ownerNormal,
                            samplePoint,
                            sampleNormal,
                            nearHitT,
                            dirtOcclusion,
                            settings);
                        cr += contrib.x;
                        cg += contrib.y;
                        cb += contrib.z;
                    }
                    if (surfaceEmitters) {
                        for (uint32_t emitterIndex : rectSurfaceEmitterIndices) {
                            if (emitterIndex >= surfaceEmitters->size()) {
                                continue;
                            }
                            const SurfaceLightEmitter& emitter = (*surfaceEmitters)[emitterIndex];
                            for (const Vector3& emitterSamplePoint : emitter.samplePoints) {
                                const Vector3 contrib = ComputeSurfaceEmitterContribution(
                                    emitter,
                                    emitterSamplePoint,
                                    occ,
                                    repairSolids,
                                    ownerSourcePolyIndex,
                                    ownerPlanePoint,
                                    ownerNormal,
                                    samplePoint,
                                    sampleNormal,
                                    nearHitT,
                                    dirtOcclusion,
                                    settings);
                                cr += contrib.x;
                                cg += contrib.y;
                                cb += contrib.z;
                            }
                        }
                    }
                    const Vector3 skyContrib = ComputeSkyDomeContribution(
                        occ, repairSolids, ownerSourcePolyIndex, ownerPlanePoint, ownerNormal, samplePoint, sampleNormal, nearHitT, skyTraceDistance, dirtOcclusion, settings);
                    cr += skyContrib.x;
                    cg += skyContrib.y;
                    cb += skyContrib.z;

                    const int hiX = lx * aaGrid + sx;
                    const int hiY = ly * aaGrid + sy;
                    const size_t hiIndex = (size_t)hiY * (size_t)hiW + (size_t)hiX;
                    outBuffer->opaque[hiIndex] = 1;
                    outBuffer->pixelsR[hiIndex] = std::max(0.0f, cr);
                    outBuffer->pixelsG[hiIndex] = std::max(0.0f, cg);
                    outBuffer->pixelsB[hiIndex] = std::max(0.0f, cb);
                }
            }
        }
    }
}

static void ResolveOversampledBufferToPage(const FaceRect& rect,
                                           const OversampledRectBuffer& buffer,
                                           LightmapPage& page)
{
    const int W = page.width;
    const int aaGrid = g_aaGrid;
    for (int ly = 0; ly < rect.gpu.h; ++ly) {
        for (int lx = 0; lx < rect.gpu.w; ++lx) {
            float sumR = 0.0f;
            float sumG = 0.0f;
            float sumB = 0.0f;
            int opaqueCount = 0;
            float sumRIgnoringCoverage = 0.0f;
            float sumGIgnoringCoverage = 0.0f;
            float sumBIgnoringCoverage = 0.0f;
            int totalCount = 0;

            for (int sy = 0; sy < aaGrid; ++sy) {
                for (int sx = 0; sx < aaGrid; ++sx) {
                    const int hiX = lx * aaGrid + sx;
                    const int hiY = ly * aaGrid + sy;
                    const size_t hiIndex = (size_t)hiY * (size_t)buffer.width + (size_t)hiX;
                    if (hiIndex >= buffer.opaque.size() ||
                        hiIndex >= buffer.pixelsR.size() ||
                        hiIndex >= buffer.pixelsG.size() ||
                        hiIndex >= buffer.pixelsB.size()) {
                        continue;
                    }
                    sumRIgnoringCoverage += buffer.pixelsR[hiIndex];
                    sumGIgnoringCoverage += buffer.pixelsG[hiIndex];
                    sumBIgnoringCoverage += buffer.pixelsB[hiIndex];
                    ++totalCount;
                    if (!buffer.opaque[hiIndex]) {
                        continue;
                    }
                    sumR += buffer.pixelsR[hiIndex];
                    sumG += buffer.pixelsG[hiIndex];
                    sumB += buffer.pixelsB[hiIndex];
                    ++opaqueCount;
                }
            }

            float outR = 0.0f;
            float outG = 0.0f;
            float outB = 0.0f;
            if (opaqueCount > 0) {
                const float invCount = 1.0f / (float)opaqueCount;
                outR = sumR * invCount;
                outG = sumG * invCount;
                outB = sumB * invCount;
            } else if (totalCount > 0) {
                const float invCount = 1.0f / (float)totalCount;
                outR = sumRIgnoringCoverage * invCount;
                outG = sumGIgnoringCoverage * invCount;
                outB = sumBIgnoringCoverage * invCount;
            }

            const size_t off = ((size_t)(rect.gpu.y + ly) * (size_t)W + (size_t)(rect.gpu.x + lx)) * 4;
            if (off + 3 >= page.pixels.size()) {
                continue;
            }
            page.pixels[off + 0] = std::max(0.0f, outR);
            page.pixels[off + 1] = std::max(0.0f, outG);
            page.pixels[off + 2] = std::max(0.0f, outB);
            page.pixels[off + 3] = 1.0f;
        }
    }
}

static bool ResolveStitchedOversampledCanvas(const StitchedSourceFaceCanvas& hiCanvas,
                                             int sampleScale,
                                             StitchedSourceFaceCanvas* outCanvas)
{
    if (!outCanvas || sampleScale <= 0 || hiCanvas.width <= 0 || hiCanvas.height <= 0) {
        return false;
    }

    if ((hiCanvas.width % sampleScale) != 0 || (hiCanvas.height % sampleScale) != 0) {
        return false;
    }

    const int lowWidth = hiCanvas.width / sampleScale;
    const int lowHeight = hiCanvas.height / sampleScale;
    if (lowWidth <= 0 || lowHeight <= 0) {
        return false;
    }

    const size_t lowPixelCount = (size_t)lowWidth * (size_t)lowHeight;
    outCanvas->minU = hiCanvas.minU;
    outCanvas->minV = hiCanvas.minV;
    outCanvas->luxelSize = hiCanvas.luxelSize * (float)sampleScale;
    outCanvas->width = lowWidth;
    outCanvas->height = lowHeight;
    outCanvas->valid.assign(lowPixelCount, 0);
    outCanvas->pixelsR.assign(lowPixelCount, 0.0f);
    outCanvas->pixelsG.assign(lowPixelCount, 0.0f);
    outCanvas->pixelsB.assign(lowPixelCount, 0.0f);

    bool anyResolved = false;
    for (int y = 0; y < lowHeight; ++y) {
        for (int x = 0; x < lowWidth; ++x) {
            float sumR = 0.0f;
            float sumG = 0.0f;
            float sumB = 0.0f;
            int opaqueCount = 0;
            float sumRIgnoringCoverage = 0.0f;
            float sumGIgnoringCoverage = 0.0f;
            float sumBIgnoringCoverage = 0.0f;
            int totalCount = 0;

            for (int sy = 0; sy < sampleScale; ++sy) {
                for (int sx = 0; sx < sampleScale; ++sx) {
                    const int hiX = x * sampleScale + sx;
                    const int hiY = y * sampleScale + sy;
                    const size_t hiIndex = (size_t)hiY * (size_t)hiCanvas.width + (size_t)hiX;
                    if (hiIndex >= hiCanvas.valid.size() ||
                        hiIndex >= hiCanvas.pixelsR.size() ||
                        hiIndex >= hiCanvas.pixelsG.size() ||
                        hiIndex >= hiCanvas.pixelsB.size()) {
                        continue;
                    }
                    sumRIgnoringCoverage += hiCanvas.pixelsR[hiIndex];
                    sumGIgnoringCoverage += hiCanvas.pixelsG[hiIndex];
                    sumBIgnoringCoverage += hiCanvas.pixelsB[hiIndex];
                    ++totalCount;
                    if (!hiCanvas.valid[hiIndex]) {
                        continue;
                    }
                    sumR += hiCanvas.pixelsR[hiIndex];
                    sumG += hiCanvas.pixelsG[hiIndex];
                    sumB += hiCanvas.pixelsB[hiIndex];
                    ++opaqueCount;
                }
            }

            const size_t lowIndex = (size_t)y * (size_t)lowWidth + (size_t)x;
            if (opaqueCount > 0) {
                const float invCount = 1.0f / (float)opaqueCount;
                outCanvas->pixelsR[lowIndex] = std::max(0.0f, sumR * invCount);
                outCanvas->pixelsG[lowIndex] = std::max(0.0f, sumG * invCount);
                outCanvas->pixelsB[lowIndex] = std::max(0.0f, sumB * invCount);
                outCanvas->valid[lowIndex] = 1;
                anyResolved = true;
            } else if (totalCount > 0) {
                const float invCount = 1.0f / (float)totalCount;
                outCanvas->pixelsR[lowIndex] = std::max(0.0f, sumRIgnoringCoverage * invCount);
                outCanvas->pixelsG[lowIndex] = std::max(0.0f, sumGIgnoringCoverage * invCount);
                outCanvas->pixelsB[lowIndex] = std::max(0.0f, sumBIgnoringCoverage * invCount);
            }
        }
    }
    return anyResolved;
}

static bool BuildComputeOversampledGroupRects(const std::vector<FaceRect>& rects,
                                              const std::vector<size_t>& rectGroup,
                                              const StitchedSourceFaceCanvas& hiCanvas,
                                              std::vector<LightmapComputeFaceRect>* outRects,
                                              std::vector<size_t>* outRectIndices = nullptr)
{
    if (!outRects) {
        return false;
    }

    outRects->clear();
    if (outRectIndices) {
        outRectIndices->clear();
        outRectIndices->reserve(rectGroup.size());
    }
    outRects->reserve(rectGroup.size());
    for (size_t rectIndex : rectGroup) {
        if (rectIndex >= rects.size()) {
            continue;
        }

        const FaceRect& rect = rects[rectIndex];
        const int interiorW = InteriorSpanX(rect);
        const int interiorH = InteriorSpanY(rect);
        if (interiorW <= 0 || interiorH <= 0) {
            continue;
        }

        int hiX = 0;
        int hiY = 0;
        if (!GlobalCenterToStitchedIndex(hiCanvas.minU,
                                         hiCanvas.minV,
                                         hiCanvas.luxelSize,
                                         hiCanvas.width,
                                         hiCanvas.height,
                                         rect.gpu.minU + hiCanvas.luxelSize * 0.5f,
                                         rect.gpu.minV + hiCanvas.luxelSize * 0.5f,
                                         &hiX,
                                         &hiY)) {
            return false;
        }

        LightmapComputeFaceRect gpuRect = rect.gpu;
        gpuRect.x = hiX;
        gpuRect.y = hiY;
        gpuRect.w = interiorW * g_aaGrid;
        gpuRect.h = interiorH * g_aaGrid;
        outRects->push_back(gpuRect);
        if (outRectIndices) {
            outRectIndices->push_back(rectIndex);
        }
    }

    return !outRects->empty();
}

static bool BakeLightmapComputeStitchedExtra(const std::vector<FaceRect>& rects,
                                             const std::vector<std::vector<uint8_t>>& validMasks,
                                             const std::vector<LightmapComputeOccluderTri>& computeOccluders,
                                             const LightmapComputeBvh& computeBvh,
                                             const std::vector<LightmapComputeBrushSolid>& computeBrushSolids,
                                             const std::vector<LightmapComputeSolidPlane>& computeSolidPlanes,
                                             const std::vector<LightmapComputeRepairSourcePoly>& computeRepairSourcePolys,
                                             const std::vector<LightmapComputeRepairSourceNeighbor>& computeRepairSourceNeighbors,
                                             const std::vector<LightmapComputePhongSourcePoly>& computePhongSourcePolys,
                                             const std::vector<LightmapComputePhongNeighbor>& computePhongNeighbors,
                                             const std::vector<PointLight>& lights,
                                             const std::vector<LightmapComputeSurfaceEmitter>& surfaceEmitters,
                                             const std::vector<LightmapComputeSurfaceEmitterSample>& surfaceEmitterSamples,
                                             const LightBakeSettings& settings,
                                             float skyTraceDistance,
                                             std::vector<LightmapPage>& pages,
                                             std::string* error)
{
    if (g_aaGrid <= 1) {
        return true;
    }

    const auto rectsBySourcePoly = GroupRectsBySourcePoly(rects);
    for (const auto& [sourcePolyIndex, rectGroup] : rectsBySourcePoly) {
        (void)sourcePolyIndex;
        StitchedSourceFaceCanvas hiCanvas;
        if (!InitializeStitchedSourceFaceCanvas(rects, rectGroup, g_aaGrid, &hiCanvas)) {
            continue;
        }

        std::vector<LightmapComputeFaceRect> hiRects;
        std::vector<size_t> hiRectIndices;
        if (!BuildComputeOversampledGroupRects(rects, rectGroup, hiCanvas, &hiRects, &hiRectIndices)) {
            if (error) {
                *error = "failed to build oversampled compute rects for a stitched source face";
            }
            return false;
        }

        std::vector<LightmapComputeRectSurfaceEmitterRange> hiRectSurfaceEmitterRanges;
        std::vector<uint32_t> hiRectSurfaceEmitterIndices;
        BuildComputeRectSurfaceEmitterDispatchData(rects,
                                                  hiRectIndices,
                                                  &hiRectSurfaceEmitterRanges,
                                                  &hiRectSurfaceEmitterIndices);

        std::vector<float> hiPixels;
        if (!BakeLightmapCompute(hiRects,
                                 computeOccluders,
                                 computeBvh,
                                 computeBrushSolids,
                                 computeSolidPlanes,
                                 computeRepairSourcePolys,
                                 computeRepairSourceNeighbors,
                                 computePhongSourcePolys,
                                 computePhongNeighbors,
                                 lights,
                                 surfaceEmitters,
                                 surfaceEmitterSamples,
                                 hiRectSurfaceEmitterRanges,
                                 hiRectSurfaceEmitterIndices,
                                 settings,
                                 skyTraceDistance,
                                 hiCanvas.width,
                                 hiCanvas.height,
                                 true,
                                 hiPixels,
                                 error)) {
            return false;
        }

        const size_t hiPixelCount = (size_t)hiCanvas.width * (size_t)hiCanvas.height;
        if (hiPixels.size() < hiPixelCount * 4) {
            if (error) {
                *error = "compute oversampled output buffer was smaller than expected";
            }
            return false;
        }
        for (size_t i = 0; i < hiPixelCount; ++i) {
            hiCanvas.pixelsR[i] = std::max(0.0f, hiPixels[i * 4 + 0]);
            hiCanvas.pixelsG[i] = std::max(0.0f, hiPixels[i * 4 + 1]);
            hiCanvas.pixelsB[i] = std::max(0.0f, hiPixels[i * 4 + 2]);
            hiCanvas.valid[i] = (hiPixels[i * 4 + 3] > 0.5f) ? 1 : 0;
        }

        FloodFillTransparentCanvas(hiCanvas.width, hiCanvas.height, hiCanvas.valid, hiCanvas.pixelsR, hiCanvas.pixelsG, hiCanvas.pixelsB);

        StitchedSourceFaceCanvas lowCanvas;
        if (!ResolveStitchedOversampledCanvas(hiCanvas, g_aaGrid, &lowCanvas)) {
            continue;
        }
        WriteStitchedSourceFaceCanvas(lowCanvas, rects, validMasks, rectGroup, pages);
    }

    return true;
}

static void BakeLightmapCPUStitchedExtra(const std::vector<PhongSourcePoly>& sourcePhongs,
                                         const std::vector<RepairSourcePoly>& repairPolys,
                                         const std::vector<PointLight>& lights,
                                         const std::vector<SurfaceLightEmitter>* surfaceEmitters,
                                         const std::vector<FaceRect>& rects,
                                         const std::vector<std::vector<uint32_t>>* lightIndicesByRect,
                                         const std::vector<std::vector<uint32_t>>* surfaceEmitterIndicesByRect,
                                         const std::vector<std::vector<uint8_t>>& validMasks,
                                         const OccluderSet& occ,
                                         const std::vector<BrushSolid>& repairSolids,
                                         const LightBakeSettings& settings,
                                         float skyTraceDistance,
                                         std::vector<LightmapPage>& pages)
{
    if (g_aaGrid <= 1) {
        return;
    }

    static const std::vector<uint32_t> kEmptyLightIndices;
    const auto rectsBySourcePoly = GroupRectsBySourcePoly(rects);
    for (const auto& [sourcePolyIndex, rectGroup] : rectsBySourcePoly) {
        (void)sourcePolyIndex;
        StitchedSourceFaceCanvas hiCanvas;
        if (!InitializeStitchedSourceFaceCanvas(rects, rectGroup, g_aaGrid, &hiCanvas)) {
            continue;
        }

        const size_t hiPixelCount = (size_t)hiCanvas.width * (size_t)hiCanvas.height;
        std::vector<uint16_t> sampleCounts(hiPixelCount, 0);
        for (size_t rectIndex : rectGroup) {
            if (rectIndex >= rects.size()) {
                continue;
            }
            const FaceRect& rect = rects[rectIndex];
            const std::vector<uint32_t>& rectLightIndices = lights.empty()
                ? kEmptyLightIndices
                : (lightIndicesByRect ? (*lightIndicesByRect)[rectIndex] : rect.lightIndices);
            const std::vector<uint32_t>& rectSurfaceEmitterIndices = surfaceEmitterIndicesByRect
                ? (*surfaceEmitterIndicesByRect)[rectIndex]
                : rect.surfaceEmitterIndices;

            OversampledRectBuffer rectBuffer;
            ShadeRectOversampled(rect,
                                 sourcePhongs,
                                 repairPolys,
                                 lights,
                                 surfaceEmitters,
                                 rectLightIndices,
                                 rectSurfaceEmitterIndices,
                                 occ,
                                 repairSolids,
                                 settings,
                                 skyTraceDistance,
                                 &rectBuffer);

            for (int ly = 0; ly < rect.gpu.h; ++ly) {
                for (int lx = 0; lx < rect.gpu.w; ++lx) {
                    for (int sy = 0; sy < g_aaGrid; ++sy) {
                        for (int sx = 0; sx < g_aaGrid; ++sx) {
                            const int hiX = lx * g_aaGrid + sx;
                            const int hiY = ly * g_aaGrid + sy;
                            const size_t rectHiIndex = (size_t)hiY * (size_t)rectBuffer.width + (size_t)hiX;
                            if (rectHiIndex >= rectBuffer.opaque.size() || !rectBuffer.opaque[rectHiIndex]) {
                                continue;
                            }

                            int stitchedX = 0;
                            int stitchedY = 0;
                            if (!RectLocalSubsampleToStitchedIndex(rect,
                                                                   hiCanvas.minU,
                                                                   hiCanvas.minV,
                                                                   hiCanvas.luxelSize,
                                                                   hiCanvas.width,
                                                                   hiCanvas.height,
                                                                   lx,
                                                                   ly,
                                                                   sx,
                                                                   sy,
                                                                   g_aaGrid,
                                                                   &stitchedX,
                                                                   &stitchedY)) {
                                continue;
                            }

                            const size_t stitchedIndex = (size_t)stitchedY * (size_t)hiCanvas.width + (size_t)stitchedX;
                            if (stitchedIndex >= hiPixelCount) {
                                continue;
                            }
                            hiCanvas.pixelsR[stitchedIndex] += rectBuffer.pixelsR[rectHiIndex];
                            hiCanvas.pixelsG[stitchedIndex] += rectBuffer.pixelsG[rectHiIndex];
                            hiCanvas.pixelsB[stitchedIndex] += rectBuffer.pixelsB[rectHiIndex];
                            sampleCounts[stitchedIndex] += 1;
                        }
                    }
                }
            }
        }

        bool anyValid = false;
        for (size_t i = 0; i < hiPixelCount; ++i) {
            if (sampleCounts[i] == 0) {
                continue;
            }
            const float invCount = 1.0f / (float)sampleCounts[i];
            hiCanvas.pixelsR[i] *= invCount;
            hiCanvas.pixelsG[i] *= invCount;
            hiCanvas.pixelsB[i] *= invCount;
            hiCanvas.valid[i] = 1;
            anyValid = true;
        }
        if (!anyValid) {
            continue;
        }

        FloodFillTransparentCanvas(hiCanvas.width, hiCanvas.height, hiCanvas.valid, hiCanvas.pixelsR, hiCanvas.pixelsG, hiCanvas.pixelsB);

        StitchedSourceFaceCanvas lowCanvas;
        if (!ResolveStitchedOversampledCanvas(hiCanvas, g_aaGrid, &lowCanvas)) {
            continue;
        }
        WriteStitchedSourceFaceCanvas(lowCanvas, rects, validMasks, rectGroup, pages);
    }
}

static void BakeLightmapCPUPage(const std::vector<BakePatch>& patches,
                                const std::vector<PhongSourcePoly>& sourcePhongs,
                                const std::vector<RepairSourcePoly>& repairPolys,
                                const std::vector<PointLight>& lights,
                                const std::vector<SurfaceLightEmitter>* surfaceEmitters,
                                const std::vector<FaceRect>& rects,
                                const std::vector<std::vector<uint32_t>>* lightIndicesByRect,
                                const std::vector<std::vector<uint32_t>>* surfaceEmitterIndicesByRect,
                                uint32_t pageIndex,
                                const OccluderSet& occ,
                                const std::vector<BrushSolid>& repairSolids,
                                const LightBakeSettings& settings,
                                float skyTraceDistance,
                                LightmapPage& page)
{
    (void)patches;
    static const std::vector<uint32_t> kEmptyLightIndices;

    for (size_t i = 0; i < rects.size(); ++i) {
        const FaceRect& r = rects[i];
        if (r.page != pageIndex) {
            continue;
        }

        const std::vector<uint32_t>& rectLightIndices = lights.empty()
            ? kEmptyLightIndices
            : (lightIndicesByRect ? (*lightIndicesByRect)[i] : r.lightIndices);
        const std::vector<uint32_t>& rectSurfaceEmitterIndices = surfaceEmitterIndicesByRect
            ? (*surfaceEmitterIndicesByRect)[i]
            : r.surfaceEmitterIndices;

        OversampledRectBuffer buffer;
        ShadeRectOversampled(r,
                             sourcePhongs,
                             repairPolys,
                             lights,
                             surfaceEmitters,
                             rectLightIndices,
                             rectSurfaceEmitterIndices,
                             occ,
                             repairSolids,
                             settings,
                             skyTraceDistance,
                             &buffer);
        if (g_aaGrid > 1) {
            FloodFillTransparentCanvas(buffer.width, buffer.height, buffer.opaque, buffer.pixelsR, buffer.pixelsG, buffer.pixelsB);
        }
        ResolveOversampledBufferToPage(r, buffer, page);
    }
}

// ---------------------------------------------------------------------------
//  Post-process box-filter "soften" pass for `_soften`.
//
//  For each stitched source-face canvas, first flood-fill uncovered cells from
//  neighbouring covered cells, then replace each original covered luxel with
//  the average of a clamped (2n+1) x (2n+1) neighbourhood. This keeps the blur
//  continuous across split bake patches while behaving more like ericw's
//  flood-fill + box-blur output path near face borders.
// ---------------------------------------------------------------------------
static void ApplyLightmapSoften(std::vector<LightmapPage>& pages,
                                const std::vector<FaceRect>& rects,
                                const std::vector<std::vector<uint8_t>>& validMasks,
                                int soften) {
    if (soften <= 0) return;
    const int n = std::min(4, soften);  // kernel radius, 1..4

    const auto rectsBySourcePoly = GroupRectsBySourcePoly(rects);
    for (const auto& [sourcePolyIndex, rectGroup] : rectsBySourcePoly) {
        (void)sourcePolyIndex;
        StitchedSourceFaceCanvas canvas;
        if (!BuildStitchedSourceFaceCanvas(rects, pages, validMasks, rectGroup, &canvas)) {
            continue;
        }

        const int W = canvas.width;
        const int H = canvas.height;
        const std::vector<uint8_t> originalValid = canvas.valid;
        std::vector<uint8_t> filteredValid = canvas.valid;
        std::vector<float> filteredR = canvas.pixelsR;
        std::vector<float> filteredG = canvas.pixelsG;
        std::vector<float> filteredB = canvas.pixelsB;
        FloodFillTransparentCanvas(W, H, filteredValid, filteredR, filteredG, filteredB);

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * (size_t)W + (size_t)x;
                if (!originalValid[idx]) {
                    continue;
                }

                float sumR = 0.0f;
                float sumG = 0.0f;
                float sumB = 0.0f;
                int count = 0;
                float sumRIgnoringCoverage = 0.0f;
                float sumGIgnoringCoverage = 0.0f;
                float sumBIgnoringCoverage = 0.0f;
                int countIgnoringCoverage = 0;
                for (int oy = -n; oy <= n; ++oy) {
                    const int ny = std::clamp(y + oy, 0, H - 1);
                    for (int ox = -n; ox <= n; ++ox) {
                        const int nx = std::clamp(x + ox, 0, W - 1);
                        const size_t ni = (size_t)ny * (size_t)W + (size_t)nx;
                        sumRIgnoringCoverage += filteredR[ni];
                        sumGIgnoringCoverage += filteredG[ni];
                        sumBIgnoringCoverage += filteredB[ni];
                        ++countIgnoringCoverage;
                        if (filteredValid[ni]) {
                            sumR += filteredR[ni];
                            sumG += filteredG[ni];
                            sumB += filteredB[ni];
                            ++count;
                        }
                    }
                }
                if (count > 0) {
                    const float invCount = 1.0f / (float)count;
                    canvas.pixelsR[idx] = sumR * invCount;
                    canvas.pixelsG[idx] = sumG * invCount;
                    canvas.pixelsB[idx] = sumB * invCount;
                } else if (countIgnoringCoverage > 0) {
                    const float invCount = 1.0f / (float)countIgnoringCoverage;
                    canvas.pixelsR[idx] = sumRIgnoringCoverage * invCount;
                    canvas.pixelsG[idx] = sumGIgnoringCoverage * invCount;
                    canvas.pixelsB[idx] = sumBIgnoringCoverage * invCount;
                }
            }
        }

        WriteStitchedSourceFaceCanvas(canvas, rects, validMasks, rectGroup, pages);
    }
}

// --------------------------------------------------------------------------
LightmapAtlas BakeLightmap(const std::vector<MapPolygon>& polys,
                           const std::vector<MapPolygon>& occluderPolys,
                           const std::vector<MapPolygon>& solidPolys,
                           const std::vector<PointLight>& lights,
                           const std::vector<SurfaceLightTemplate>& surfaceLights,
                           const std::unordered_map<std::string, Vector3>& textureBounceColors,
                           const LightBakeSettings& settings,
                           LightmapBakeBackendMode backendMode)
{
    LightmapAtlas atlas;
    const float luxelSize = std::max(0.125f, settings.luxelSize);
    const bool forceCpuFromEnv = (backendMode == LIGHTMAP_BAKE_BACKEND_AUTO) && ForceLightmapCPUFromEnv();
    const bool forceCpuBake = (backendMode == LIGHTMAP_BAKE_BACKEND_FORCE_CPU) || forceCpuFromEnv;
    const char* forceCpuReason = (backendMode == LIGHTMAP_BAKE_BACKEND_FORCE_CPU)
        ? "forced CPU reference bake via -cpu"
        : "forced CPU reference bake via WARPED_LIGHTMAP_FORCE_CPU";
    if (forceCpuBake) {
        printf("[Lightmap] %s.\n", forceCpuReason);
        fflush(stdout);
    } else if (backendMode == LIGHTMAP_BAKE_BACKEND_PREFER_GPU) {
        printf("[Lightmap] preferring GPU compute bake via -gpu; unsupported pages can still fall back to CPU.\n");
        fflush(stdout);
    }

    // Pick the super-sampling grid size from the worldspawn `_extra_samples`
    // key. 0 -> 1x1 (off), 2 -> 2x2, 4 -> 4x4 (historical default). This value
    // is consulted by BakeLightmapCPUPage, BuildCoverageMask, and every
    // coverage-threshold check in this TU via the file-scope g_aaGrid.
    g_aaGrid = (settings.extraSamples <= 0) ? 1
             : (settings.extraSamples <= 2) ? 2 : 4;
    // CSG union is responsible for removing true internal/contact geometry.
    // The lightmap visibility heuristic can false-positive on valid clipped
    // faces; because compile_map emits render geometry from atlas.patches, do
    // not let the baker hide source polygons from the final mesh.
    const std::vector<MapPolygon>& visiblePolys = polys;
    const std::vector<PhongSourcePoly> sourcePhongs = BuildPhongSourcePolys(visiblePolys);
    const std::vector<RepairSourcePoly> repairPolys = BuildRepairSourcePolys(visiblePolys, sourcePhongs);
    const std::vector<BrushSolid> repairSolids = BuildBrushSolids(solidPolys.empty() ? polys : solidPolys);
    const AABB visibleBounds = ComputeMapBounds(visiblePolys);
    const Vector3 mapCenter = Vector3Scale(Vector3Add(visibleBounds.min, visibleBounds.max), 0.5f);
    const float skyTraceDistance = std::max(2048.0f, sqrtf(Vector3LengthSq(Vector3Subtract(visibleBounds.max, visibleBounds.min))) * 2.0f + 1024.0f);
    const std::vector<PointLight> directPointLights = BuildDirectPointLights(lights, settings, mapCenter, skyTraceDistance);
    const std::vector<SurfaceLightEmitter> surfaceEmitters = BuildSurfaceEmitters(visiblePolys, surfaceLights, repairSolids, settings);
    std::vector<BakePatch> patches = SubdivideLightmappedPolygons(visiblePolys, luxelSize);
    std::vector<FaceRect> rects = BuildFaceRects(patches, directPointLights, repairPolys, sourcePhongs, luxelSize);
    BuildRectSurfaceEmitterIndices(rects, surfaceEmitters);

    std::vector<LightmapPageLayout> layouts = PackLightmapPages(rects);
    if (layouts.empty() && !rects.empty()) {
        printf("[Lightmap] failed to pack lightmap pages.\n");
        return atlas;
    }

    atlas.pages.resize(layouts.size());
    for (size_t i = 0; i < layouts.size(); ++i) {
        atlas.pages[i].width = LIGHTMAP_PAGE_SIZE;
        atlas.pages[i].height = layouts[i].usedHeight;
        atlas.pages[i].pixels.assign((size_t)atlas.pages[i].width * (size_t)atlas.pages[i].height * 4, 0.0f);
    }

    FillPatchUVs(patches, rects, atlas.pages, atlas);

    size_t totalLuxels = 0;
    for (const FaceRect& r : rects) {
        totalLuxels += (size_t)r.gpu.w * (size_t)r.gpu.h;
    }

    OccluderSet occ = BuildOccluders(occluderPolys.empty() ? visiblePolys : occluderPolys);
    const std::vector<LightmapComputeOccluderTri> computeOccluders = BuildLightmapComputeOccluders(occ);
    LightmapComputeBvh computeBvh;
    if (!forceCpuBake) {
        std::string computeBvhError;
        if (BuildLightmapComputeBvh(computeOccluders, &computeBvh, &computeBvhError)) {
            if (!computeBvh.nodes.empty()) {
                printf("[Lightmap] Embree-built GPU BVH ready (%zu nodes, %zu tri refs).\n",
                       computeBvh.nodes.size(),
                       computeBvh.triIndices.size());
                fflush(stdout);
            }
        } else if (!computeBvhError.empty()) {
            printf("[Lightmap] GPU BVH unavailable; compute tracing will scan triangles: %s\n",
                   computeBvhError.c_str());
            fflush(stdout);
        }
    }
    std::vector<LightmapComputeBrushSolid> computeBrushSolids;
    std::vector<LightmapComputeSolidPlane> computeSolidPlanes;
    std::vector<LightmapComputeRepairSourcePoly> computeRepairSourcePolys;
    std::vector<LightmapComputeRepairSourceNeighbor> computeRepairSourceNeighbors;
    std::vector<LightmapComputePhongSourcePoly> computePhongSourcePolys;
    std::vector<LightmapComputePhongNeighbor> computePhongNeighbors;
    std::vector<LightmapComputeSurfaceEmitter> computeSurfaceEmitters;
    std::vector<LightmapComputeSurfaceEmitterSample> computeSurfaceEmitterSamples;
    BuildComputeBrushSolids(repairSolids, &computeBrushSolids, &computeSolidPlanes);
    BuildComputeRepairSourceGraph(repairPolys, &computeRepairSourcePolys, &computeRepairSourceNeighbors);
    BuildComputePhongGraph(sourcePhongs, &computePhongSourcePolys, &computePhongNeighbors);
    BuildComputeSurfaceEmitterPayload(surfaceEmitters, &computeSurfaceEmitters, &computeSurfaceEmitterSamples);
    const bool repairGraphExceedsComputeVerts = std::any_of(
        repairPolys.begin(),
        repairPolys.end(),
        [](const RepairSourcePoly& poly) {
            return poly.poly2d.size() > LIGHTMAP_COMPUTE_MAX_POLY_VERTS;
        });
    if (repairGraphExceedsComputeVerts) {
        for (FaceRect& rect : rects) {
            rect.computeCompatible = false;
            if (rect.computeFallbackReason.empty()) {
                rect.computeFallbackReason = "page contains source polygon(s) exceeding compute repair vertex limit";
            }
        }
    }
    printf("[Lightmap] %zu source faces -> %zu bake patches across %zu pages of up to %dx%d, %zu direct point lights, %zu grouped surface emitters, %zu tris, %zu luxels x %d samples = %zu rays/light (luxel=%.3f, ambient=%.2f/%.2f/%.2f, bounces=%d, bounceScale=%.2f)\n",
           visiblePolys.size(), patches.size(), atlas.pages.size(), LIGHTMAP_PAGE_SIZE, LIGHTMAP_PAGE_SIZE,
           directPointLights.size(), surfaceEmitters.size(), occ.tris.size(),
           totalLuxels, g_aaGrid * g_aaGrid, totalLuxels * (size_t)(g_aaGrid * g_aaGrid),
           luxelSize,
           settings.ambientColor.x, settings.ambientColor.y, settings.ambientColor.z,
           settings.bounceCount, settings.bounceScale);
    fflush(stdout);

    std::vector<std::vector<uint8_t>> coverageMasks(atlas.pages.size());
    std::vector<std::vector<uint8_t>> baseValidMasks(atlas.pages.size());
    std::vector<std::vector<uint8_t>> strictInteriorMasks(atlas.pages.size());
    for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
        const LightmapPage& page = atlas.pages[pageIndex];
        coverageMasks[pageIndex] = BuildCoverageMask(rects, pageIndex, page.width, page.height);
        baseValidMasks[pageIndex] = CoverageToValidMask(coverageMasks[pageIndex]);
        strictInteriorMasks[pageIndex] = BuildStrictInteriorMask(rects, pageIndex, page.width, page.height);
    }

    const bool useStitchedExtraResolve = (g_aaGrid > 1);
    bool directUseComputeStitchedResolve = false;
    std::string directComputeStitchedError;
    if (useStitchedExtraResolve && !forceCpuBake) {
        directUseComputeStitchedResolve = true;
        for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
            std::vector<PointLight> pageLights = GatherPageLights(rects, pageIndex, directPointLights);
            for (const FaceRect& rect : rects) {
                if (rect.page != pageIndex || rect.computeCompatible) {
                    continue;
                }
                directUseComputeStitchedResolve = false;
                directComputeStitchedError = rect.computeFallbackReason.empty()
                    ? "page contains rects that the compute baker does not support"
                    : rect.computeFallbackReason;
                break;
            }
            if (!directUseComputeStitchedResolve) {
                break;
            }
            if (PageUsesCPUOnlyLightingFeatures(pageIndex, rects, sourcePhongs, pageLights, false, true, settings, &directComputeStitchedError)) {
                directUseComputeStitchedResolve = false;
                break;
            }
        }

        if (directUseComputeStitchedResolve &&
            !BakeLightmapComputeStitchedExtra(rects,
                                              baseValidMasks,
                                              computeOccluders,
                                              computeBvh,
                                              computeBrushSolids,
                                              computeSolidPlanes,
                                              computeRepairSourcePolys,
                                              computeRepairSourceNeighbors,
                                              computePhongSourcePolys,
                                              computePhongNeighbors,
                                              directPointLights,
                                              computeSurfaceEmitters,
                                              computeSurfaceEmitterSamples,
                                              settings,
                                              skyTraceDistance,
                                              atlas.pages,
                                              &directComputeStitchedError)) {
            directUseComputeStitchedResolve = false;
        }
    }

    bool directStitchedCpuBaked = false;
    for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
        LightmapPage& page = atlas.pages[pageIndex];
        printf("[Lightmap] page %u/%zu begin (%dx%d)\n",
               pageIndex + 1, atlas.pages.size(), page.width, page.height);
        fflush(stdout);
        std::vector<uint8_t>& coverage = coverageMasks[pageIndex];
        std::vector<uint8_t>& valid = baseValidMasks[pageIndex];
        std::vector<PointLight> pageLights = GatherPageLights(rects, pageIndex, directPointLights);
        const size_t pageSurfaceEmitterCount = CountPageSurfaceEmitters(rects, pageIndex, surfaceEmitters.size());
        size_t pageLuxels = 0;

        std::vector<LightmapComputeFaceRect> pageRects;
        std::vector<size_t> pageRectIndices;
        pageRects.reserve(rects.size());
        pageRectIndices.reserve(rects.size());
        bool pageRequiresCPU = forceCpuBake ||
            (useStitchedExtraResolve && (!directUseComputeStitchedResolve || directStitchedCpuBaked));
        std::string computeError = forceCpuBake
            ? forceCpuReason
            : ((useStitchedExtraResolve && directStitchedCpuBaked)
                ? "stitched direct bake already fell back to CPU earlier in this atlas"
                : ((useStitchedExtraResolve && !directUseComputeStitchedResolve) ? directComputeStitchedError : std::string()));
        for (size_t rectIndex = 0; rectIndex < rects.size(); ++rectIndex) {
            const FaceRect& r = rects[rectIndex];
            if (r.page == pageIndex) {
                pageRects.push_back(r.gpu);
                pageRectIndices.push_back(rectIndex);
                pageLuxels += (size_t)r.gpu.w * (size_t)r.gpu.h;
                if (!useStitchedExtraResolve && !r.computeCompatible) {
                    pageRequiresCPU = true;
                    if (computeError.empty()) {
                        computeError = r.computeFallbackReason;
                    }
                }
            }
        }
        printf("[Lightmap] page %u stats: %zu rects, %zu direct point lights, %zu grouped surface emitters%s%s, %zu luxels\n",
               pageIndex,
               pageRects.size(),
               pageLights.size(),
               pageSurfaceEmitterCount,
               (settings.sunlight2Intensity > 0.0f) ? ", + upper skylight" : "",
               (settings.sunlight3Intensity > 0.0f) ? ", + lower skylight" : "",
               pageLuxels);
        fflush(stdout);

        bool usedCPUFallback = false;
        if (!pageRequiresCPU &&
            !directUseComputeStitchedResolve &&
            PageUsesCPUOnlyLightingFeatures(pageIndex, rects, sourcePhongs, pageLights, !surfaceEmitters.empty(), true, settings, &computeError)) {
            pageRequiresCPU = true;
        }
        std::vector<LightmapComputeRectSurfaceEmitterRange> pageRectSurfaceEmitterRanges;
        std::vector<uint32_t> pageRectSurfaceEmitterIndices;
        BuildComputeRectSurfaceEmitterDispatchData(rects,
                                                  pageRectIndices,
                                                  &pageRectSurfaceEmitterRanges,
                                                  &pageRectSurfaceEmitterIndices);
        const bool usedPrecomputedComputeResult = directUseComputeStitchedResolve && !directStitchedCpuBaked;
        if (!usedPrecomputedComputeResult &&
            (pageRequiresCPU || !BakeLightmapCompute(pageRects,
                                                     computeOccluders,
                                                     computeBvh,
                                                     computeBrushSolids,
                                                     computeSolidPlanes,
                                                     computeRepairSourcePolys,
                                                     computeRepairSourceNeighbors,
                                                     computePhongSourcePolys,
                                                     computePhongNeighbors,
                                                     pageLights,
                                                     computeSurfaceEmitters,
                                                     computeSurfaceEmitterSamples,
                                                     pageRectSurfaceEmitterRanges,
                                                     pageRectSurfaceEmitterIndices,
                                                     settings,
                                                     skyTraceDistance,
                                                     page.width,
                                                     page.height,
                                                     false,
                                                     page.pixels,
                                                     &computeError))) {
            if (pageRequiresCPU) {
                printf("[Lightmap] page %u CPU bake selected: %s\n",
                       pageIndex, computeError.c_str());
            } else {
                printf("[Lightmap] page %u compute bake unavailable, falling back to CPU: %s\n",
                       pageIndex, computeError.c_str());
            }
            fflush(stdout);
            if (useStitchedExtraResolve) {
                if (!directStitchedCpuBaked) {
                    BakeLightmapCPUStitchedExtra(sourcePhongs, repairPolys, directPointLights, &surfaceEmitters, rects, nullptr, nullptr, baseValidMasks, occ, repairSolids, settings, skyTraceDistance, atlas.pages);
                    directStitchedCpuBaked = true;
                }
            } else {
                BakeLightmapCPUPage(patches, sourcePhongs, repairPolys, directPointLights, &surfaceEmitters, rects, nullptr, nullptr, pageIndex, occ, repairSolids, settings, skyTraceDistance, page);
            }
            usedCPUFallback = true;
            printf("[Lightmap] page %u CPU bake complete\n", pageIndex);
            fflush(stdout);
        } else if (usedPrecomputedComputeResult) {
            printf("[Lightmap] page %u stitched compute bake complete\n", pageIndex);
            fflush(stdout);
        }

        bool edgeTexelsStabilized = false;
        if (!usedCPUFallback) {
            // GPU edge samples can differ slightly from CPU coverage on partial
            // texels; stabilize those borders before treating dark texels as a
            // hard compute failure.
            printf("[Lightmap] page %u stabilizing edge texels\n", pageIndex);
            fflush(stdout);
            StabilizeEdgeTexels(page, coverage);
            edgeTexelsStabilized = true;

            const DarkLuxelStats darkStats = GatherDarkLuxelStats(page, valid, coverage, strictInteriorMasks[pageIndex]);
            if (HasInvalidDarkValidLuxels(darkStats)) {
                printf("[Lightmap] page %u compute bake produced invalid dark valid texels "
                       "(dark valid=%zu/%zu, dark full=%zu/%zu, dark strict interior=%zu/%zu), falling back to CPU.\n",
                       pageIndex,
                       darkStats.darkValidCount, darkStats.validCount,
                       darkStats.darkFullCoverageCount, darkStats.fullCoverageCount,
                       darkStats.darkStrictInteriorCount, darkStats.strictInteriorCount);
                fflush(stdout);
                if (useStitchedExtraResolve) {
                    if (!directStitchedCpuBaked) {
                        BakeLightmapCPUStitchedExtra(sourcePhongs, repairPolys, directPointLights, &surfaceEmitters, rects, nullptr, nullptr, baseValidMasks, occ, repairSolids, settings, skyTraceDistance, atlas.pages);
                        directStitchedCpuBaked = true;
                    }
                } else {
                    BakeLightmapCPUPage(patches, sourcePhongs, repairPolys, directPointLights, &surfaceEmitters, rects, nullptr, nullptr, pageIndex, occ, repairSolids, settings, skyTraceDistance, page);
                }
                usedCPUFallback = true;
                edgeTexelsStabilized = false;
                printf("[Lightmap] page %u CPU re-bake complete\n", pageIndex);
                fflush(stdout);
            }
        }

        if (!edgeTexelsStabilized) {
            printf("[Lightmap] page %u stabilizing edge texels\n", pageIndex);
            fflush(stdout);
            StabilizeEdgeTexels(page, coverage);
        }
        printf("[Lightmap] page %u direct complete\n", pageIndex);
        fflush(stdout);
    }

    const size_t directSeamWelded = WeldSiblingPatchSeams(rects, atlas.pages, coverageMasks, luxelSize);
    if (directSeamWelded > 0) {
        printf("[Lightmap] welded %zu direct seam-adjacent luxel pairs across split patches.\n", directSeamWelded);
        fflush(stdout);
    }

    std::vector<LightmapPage> bounceSourcePages = atlas.pages;
    Vector3 bounceAmbient = settings.ambientColor;
    for (int bouncePass = 0; bouncePass < settings.bounceCount; ++bouncePass) {
        const std::vector<SurfaceLightEmitter> indirectEmitters = BuildIndirectBounceEmitters(
            visiblePolys, rects, bounceSourcePages, coverageMasks, surfaceLights, textureBounceColors, bounceAmbient, repairSolids, settings, bouncePass + 1);
        if (indirectEmitters.empty()) {
            if (bouncePass == 0) {
                printf("[Lightmap] indirect bounce emitters: 0\n");
                fflush(stdout);
            }
            break;
        }

        printf("[Lightmap] bounce pass %d/%d emitters: %zu\n",
               bouncePass + 1, settings.bounceCount, indirectEmitters.size());
        fflush(stdout);
        const std::vector<std::vector<uint32_t>> indirectEmitterIndices = BuildSurfaceEmitterIndicesByRect(rects, indirectEmitters);
        std::vector<LightmapPage> bouncedPages = atlas.pages;
        for (LightmapPage& bouncedPage : bouncedPages) {
            bouncedPage = MakeBlankPageLike(bouncedPage);
        }
        const std::vector<PointLight> noIndirectPointLights;
        bool bounceStitchedCpuBaked = false;

        for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
            LightmapPage& page = atlas.pages[pageIndex];
            LightmapPage& bouncedPage = bouncedPages[pageIndex];
            const size_t pageIndirectEmitterCount = CountPageSurfaceEmitters(rects, pageIndex, indirectEmitters.size(), &indirectEmitterIndices);
            if (pageIndirectEmitterCount == 0) {
                continue;
            }

            printf("[Lightmap] page %u bounce %d begin (%zu bounce emitters)\n",
                   pageIndex, bouncePass + 1, pageIndirectEmitterCount);
            fflush(stdout);
            LightBakeSettings bounceSettings = settings;
            bounceSettings.ambientColor = Vector3Zero();
            bounceSettings.sunlight2Intensity = 0.0f;
            bounceSettings.sunlight3Intensity = 0.0f;
            if (useStitchedExtraResolve) {
                if (!bounceStitchedCpuBaked) {
                    BakeLightmapCPUStitchedExtra(sourcePhongs, repairPolys, noIndirectPointLights, &indirectEmitters, rects, nullptr, &indirectEmitterIndices, baseValidMasks, occ, repairSolids, bounceSettings, skyTraceDistance, bouncedPages);
                    bounceStitchedCpuBaked = true;
                }
            } else {
                BakeLightmapCPUPage(patches, sourcePhongs, repairPolys, noIndirectPointLights, &indirectEmitters, rects, nullptr, &indirectEmitterIndices, pageIndex, occ, repairSolids, bounceSettings, skyTraceDistance, bouncedPage);
            }

            StabilizeEdgeTexels(bouncedPage, coverageMasks[pageIndex]);
            AddPagePixels(page, bouncedPage);
            printf("[Lightmap] page %u bounce %d complete\n", pageIndex, bouncePass + 1);
            fflush(stdout);
        }

        const size_t indirectSeamWelded = WeldSiblingPatchSeams(rects, atlas.pages, coverageMasks, luxelSize);
        if (indirectSeamWelded > 0) {
            printf("[Lightmap] welded %zu bounce %d seam-adjacent luxel pairs across split patches.\n",
                   indirectSeamWelded, bouncePass + 1);
            fflush(stdout);
        }

        WeldSiblingPatchSeams(rects, bouncedPages, coverageMasks, luxelSize);

        bounceSourcePages = std::move(bouncedPages);
        bounceAmbient = Vector3Zero();
    }

    const float outputRangeScale = EffectiveOutputRangeScale(settings);
    const float outputGamma = EffectiveOutputGamma(settings);
    if (settings.maxLight > 0.0f ||
        fabsf(outputRangeScale - 1.0f) > 1e-6f ||
        fabsf(outputGamma - 1.0f) > 1e-6f) {
        printf("[Lightmap] applying output conditioning (_maxlight=%.3f, _range=%.2f -> scale=%.3f, _gamma=%.2f)\n",
               settings.maxLight, settings.rangeScale, outputRangeScale, settings.lightmapGamma);
        fflush(stdout);
        ApplyLightmapOutputConditioning(atlas.pages, baseValidMasks, settings);
    }

    if (settings.lmAAScale > 0) {
        printf("[Lightmap] applying post-bake AA (scale=%d, sigma=%.1f)\n", settings.lmAAScale, settings.lmAAScale / 2.0f);
        fflush(stdout);
        ApplyLightmapAA(atlas.pages, rects, baseValidMasks, settings.lmAAScale);

        // The stitched-face AA pass now filters across split patch seams for a
        // source face directly, but keeping the final seam weld here still
        // helps clamp any residual quantization mismatch between independently
        // baked sibling patches before DilatePage propagates the edge values.
        const size_t postAASeamWelded = WeldSiblingPatchSeams(rects, atlas.pages, coverageMasks, luxelSize);
        if (postAASeamWelded > 0) {
            printf("[Lightmap] welded %zu post-AA seam-adjacent luxel pairs across split patches.\n", postAASeamWelded);
            fflush(stdout);
        }
    }

    // Post-process box-filter softening from the worldspawn `_soften` key. Runs
    // after AA so the gaussian and the box filter compose cleanly, and re-welds
    // sibling seams afterwards for the same residual quantization guard.
    if (settings.soften > 0) {
        printf("[Lightmap] applying _soften post-process (n=%d, %dx%d window)\n",
               settings.soften, settings.soften * 2 + 1, settings.soften * 2 + 1);
        fflush(stdout);
        ApplyLightmapSoften(atlas.pages, rects, baseValidMasks, settings.soften);

        const size_t postSoftenSeamWelded = WeldSiblingPatchSeams(rects, atlas.pages, coverageMasks, luxelSize);
        if (postSoftenSeamWelded > 0) {
            printf("[Lightmap] welded %zu post-soften seam-adjacent luxel pairs across split patches.\n", postSoftenSeamWelded);
            fflush(stdout);
        }
    }

    for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
        LightmapPage& page = atlas.pages[pageIndex];
        std::vector<uint8_t> valid = baseValidMasks[pageIndex];
        printf("[Lightmap] page %u dilating borders\n", pageIndex);
        fflush(stdout);
        DilatePage(page, valid);
        printf("[Lightmap] page %u complete\n", pageIndex);
        fflush(stdout);
    }

    return atlas;
}
