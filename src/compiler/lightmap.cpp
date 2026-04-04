// lightmap.cpp  —  offline lightmap baker with optional GPU compute path.
//
// Oversized faces are subdivided before baking so we can preserve 1:1 luxel
// density while packing into multiple smaller lightmap pages instead of one
// extremely tall atlas.

#include "lightmap.h"
#include "lightmap_compute.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>

// --------------------------------------------------------------------------
//  Tunables
// --------------------------------------------------------------------------
static constexpr int   LM_PAD             = 2;      // border luxels (filled by dilate)
static constexpr int   LIGHTMAP_PAGE_SIZE = 2048;
static constexpr int   FACE_MAX_LUXELS    = 127;    // Source-style per-face luxel cap (interior only)
static constexpr float SURFACE_SAMPLE_OFFSET = 1.0f;
static constexpr float SHADOW_BIAS        = 0.03125f;
static constexpr float SOLID_REPAIR_EPSILON = 0.05f;
static constexpr float SAMPLE_REPAIR_JITTER = 0.5f;
static constexpr float RAY_EPS            = 1e-4f;
static constexpr float OCCLUSION_NEAR_TMIN = SHADOW_BIAS;
static constexpr float EDGE_SEAM_GUARD_LUXELS = 0.75f;
static constexpr float EDGE_SEAM_TMIN_BOOST   = SHADOW_BIAS * 3.0f;
// The current indirect pass approximates bounced light with per-patch emitters.
// Keep it conservative so it lifts occluded regions without flattening direct
// shadow contrast into an overcast look.
static constexpr float INDIRECT_BOUNCE_REFLECTANCE = 0.18f;
static constexpr float INDIRECT_BOUNCE_THRESHOLD   = 0.08f;
static constexpr float INDIRECT_BOUNCE_RADIUS_BIAS = 32.0f;
static constexpr float INDIRECT_BOUNCE_RADIUS_SCALE = 2.0f;
static constexpr float INDIRECT_BOUNCE_RADIUS_MIN = 32.0f;
static constexpr float INDIRECT_BOUNCE_RADIUS_MAX = 160.0f;
static constexpr int   AA_GRID            = 4;      // AA_GRID² samples per luxel
static constexpr int   DILATE_PASSES      = 4;
static constexpr int   ERICW_SUNSAMPLES   = 100;    // matches ericw-tools default sunsamples
static constexpr int   DIRT_NUM_ANGLE_STEPS = 16;
static constexpr int   DIRT_NUM_ELEVATION_STEPS = 3;
static constexpr int   DIRT_RAY_COUNT = DIRT_NUM_ANGLE_STEPS * DIRT_NUM_ELEVATION_STEPS;

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
struct OccluderSet {
    std::vector<LightmapComputeOccluderTri> tris;
};

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
};

static bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py);

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

        for (size_t j = 0; j < polys.size(); ++j) {
            if (i == j) {
                continue;
            }
            const MapPolygon& other = polys[j];
            if (Vector3DotProduct(poly.normal, other.normal) > -0.99f) {
                continue;
            }

            const float centroidPlaneDist = Vector3DotProduct(other.normal, Vector3Subtract(centroid, other.verts[0]));
            if (fabsf(centroidPlaneDist) > 0.05f) {
                continue;
            }

            const float frontPlaneDist = Vector3DotProduct(other.normal, Vector3Subtract(frontSample, other.verts[0]));
            if (frontPlaneDist > -0.05f) {
                continue;
            }

            const Vector3 projected = Vector3Subtract(frontSample, Vector3Scale(other.normal, frontPlaneDist));
            if (pointInsidePolygonProjected(other, projected)) {
                info.hidden[i] = 1;
                break;
            }
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

static OccluderSet BuildOccluders(const std::vector<MapPolygon>& polys) {
    OccluderSet o;
    const SurfaceVisibilityInfo visibility = AnalyzeSurfaceVisibility(polys);
    for (size_t i = 0; i < polys.size(); ++i) {
        if (visibility.emitterOnly[i]) {
            continue;
        }
        if (visibility.hidden[i]) {
            continue;
        }
        const MapPolygon& p = polys[i];
        for (size_t t = 1; t + 1 < p.verts.size(); ++t) {
            LightmapComputeOccluderTri tr{};
            tr.a = p.verts[0];
            tr.b = p.verts[t];
            tr.c = p.verts[t + 1];
            tr.bounds = AABBInvalid();
            tr.occluderGroup = p.occluderGroup;
            AABBExtend(&tr.bounds, tr.a);
            AABBExtend(&tr.bounds, tr.b);
            AABBExtend(&tr.bounds, tr.c);
            o.tris.push_back(tr);
        }
    }
    if (visibility.hiddenCount > 0) {
        printf("[Lightmap] skipped %d hidden/internal brush faces from occluders.\n", visibility.hiddenCount);
    }
    if (visibility.emitterOnlyCount > 0) {
        printf("[Lightmap] skipped %d light brush faces from occluders.\n", visibility.emitterOnlyCount);
    }
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

static bool RayTriDistance(const Vector3& ro, const Vector3& rd,
                           const LightmapComputeOccluderTri& tr, float tmin, float tmax, float* outT = nullptr)
{
    Vector3 e1 = Vector3Subtract(tr.b, tr.a);
    Vector3 e2 = Vector3Subtract(tr.c, tr.a);
    Vector3 p  = Vector3CrossProduct(rd, e2);
    float det  = Vector3DotProduct(e1, p);
    if (fabsf(det) < RAY_EPS) return false;
    float inv = 1.0f / det;
    Vector3 s = Vector3Subtract(ro, tr.a);
    float u = Vector3DotProduct(s, p) * inv;      if (u < 0 || u > 1) return false;
    Vector3 q = Vector3CrossProduct(s, e1);
    float v = Vector3DotProduct(rd, q) * inv;     if (v < 0 || u + v > 1) return false;
    float t = Vector3DotProduct(e2, q) * inv;
    if (t <= tmin || t >= tmax) {
        return false;
    }
    if (outT) {
        *outT = t;
    }
    return true;
}

static bool RayAABB(const Vector3& ro, const Vector3& inv_rd,
                    const AABB& b, float tmax)
{
    float t1, t2, tn = 0, tf = tmax;
    t1 = (b.min.x - ro.x) * inv_rd.x; t2 = (b.max.x - ro.x) * inv_rd.x;
    tn = std::max(tn, std::min(t1, t2)); tf = std::min(tf, std::max(t1, t2));
    t1 = (b.min.y - ro.y) * inv_rd.y; t2 = (b.max.y - ro.y) * inv_rd.y;
    tn = std::max(tn, std::min(t1, t2)); tf = std::min(tf, std::max(t1, t2));
    t1 = (b.min.z - ro.z) * inv_rd.z; t2 = (b.max.z - ro.z) * inv_rd.z;
    tn = std::max(tn, std::min(t1, t2)); tf = std::min(tf, std::max(t1, t2));
    return tf >= tn;
}

static bool Occluded(const OccluderSet& o, const Vector3& ro,
                     const Vector3& rd, float minHitT, float dist, int ignoreOccluderGroup)
{
    Vector3 inv = {
        1.0f / (fabsf(rd.x) > RAY_EPS ? rd.x : RAY_EPS),
        1.0f / (fabsf(rd.y) > RAY_EPS ? rd.y : RAY_EPS),
        1.0f / (fabsf(rd.z) > RAY_EPS ? rd.z : RAY_EPS)
    };
    for (const auto& tri : o.tris) {
        if ((ignoreOccluderGroup >= 0) && (tri.occluderGroup == ignoreOccluderGroup)) {
            continue;
        }
        if (!RayAABB(ro, inv, tri.bounds, dist)) continue;
        if (RayTriDistance(ro, rd, tri, minHitT, dist)) return true;
    }
    return false;
}

static float ClosestHitDistance(const OccluderSet& o,
                                const Vector3& ro,
                                const Vector3& rd,
                                float minHitT,
                                float dist,
                                int ignoreOccluderGroup)
{
    Vector3 inv = {
        1.0f / (fabsf(rd.x) > RAY_EPS ? rd.x : RAY_EPS),
        1.0f / (fabsf(rd.y) > RAY_EPS ? rd.y : RAY_EPS),
        1.0f / (fabsf(rd.z) > RAY_EPS ? rd.z : RAY_EPS)
    };

    float closest = dist;
    for (const auto& tri : o.tris) {
        if ((ignoreOccluderGroup >= 0) && (tri.occluderGroup == ignoreOccluderGroup)) {
            continue;
        }
        if (!RayAABB(ro, inv, tri.bounds, closest)) {
            continue;
        }
        float hitT = 0.0f;
        if (RayTriDistance(ro, rd, tri, minHitT, closest, &hitT)) {
            closest = hitT;
        }
    }
    return closest;
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
    uint32_t page = 0;
    uint32_t sourcePolyIndex = 0;
    float maxU = 0.0f;
    float maxV = 0.0f;
    bool computeCompatible = true;
    std::string computeFallbackReason;
};

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
        const int avg = ((int)aPage.pixels[aOff + c] + (int)bPage.pixels[bOff + c] + 1) / 2;
        aPage.pixels[aOff + c] = (uint8_t)avg;
        bPage.pixels[bOff + c] = (uint8_t)avg;
    }
    aPage.pixels[aOff + 3] = 255;
    bPage.pixels[bOff + 3] = 255;
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
            atlas.patches[i].uv.push_back({
                (r.gpu.x + LM_PAD + u + 0.5f) / page.width,
                (r.gpu.y + LM_PAD + v + 0.5f) / page.height
            });
        }
    }
}

static std::vector<uint8_t> BuildCoverageMask(const std::vector<FaceRect>& rects,
                                              uint32_t pageIndex,
                                              int W, int H)
{
    std::vector<uint8_t> coverage((size_t)W * (size_t)H, 0);
    const float invGrid = 1.0f / (float)AA_GRID;
    for (const FaceRect& r : rects) {
        if (r.page != pageIndex) {
            continue;
        }
        for (int ly = 0; ly < r.gpu.h; ++ly) {
            for (int lx = 0; lx < r.gpu.w; ++lx) {
                uint8_t coveredSamples = 0;
                for (int sy = 0; sy < AA_GRID; ++sy) {
                    for (int sx = 0; sx < AA_GRID; ++sx) {
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

static void StabilizeEdgeTexels(LightmapPage& page,
                                const std::vector<uint8_t>& coverage)
{
    const int W = page.width;
    const int H = page.height;
    const uint8_t kFullCoverage = (uint8_t)(AA_GRID * AA_GRID);
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

    for (int pass = 0; pass < AA_GRID; ++pass) {
        bool changed = false;
        std::vector<uint8_t> nextStable = stable;
        std::vector<uint8_t> nextPixels = page.pixels;

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * W + x;
                if (coverage[idx] == 0 || stable[idx]) {
                    continue;
                }

                int sr = 0, sg = 0, sb = 0, n = 0;
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
                    nextPixels[idx * 4 + 0] = (uint8_t)(sr / n);
                    nextPixels[idx * 4 + 1] = (uint8_t)(sg / n);
                    nextPixels[idx * 4 + 2] = (uint8_t)(sb / n);
                    nextPixels[idx * 4 + 3] = 255;
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

static void DilatePage(LightmapPage& page, std::vector<uint8_t>& valid) {
    const int W = page.width;
    const int H = page.height;
    for (int pass = 0; pass < DILATE_PASSES; ++pass) {
        std::vector<uint8_t> nextValid = valid;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * W + x;
                if (valid[idx]) continue;
                int sr = 0, sg = 0, sb = 0, n = 0;
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
                    page.pixels[idx * 4 + 0] = (uint8_t)(sr / n);
                    page.pixels[idx * 4 + 1] = (uint8_t)(sg / n);
                    page.pixels[idx * 4 + 2] = (uint8_t)(sb / n);
                    page.pixels[idx * 4 + 3] = 255;
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
};

static DarkLuxelStats GatherDarkLuxelStats(const LightmapPage& page,
                                           const std::vector<uint8_t>& valid,
                                           const std::vector<uint8_t>& coverage)
{
    const size_t pixelCount = (size_t)page.width * (size_t)page.height;
    DarkLuxelStats stats;
    if (valid.size() < pixelCount || coverage.size() < pixelCount || page.pixels.size() < pixelCount * 4) {
        stats.darkValidCount = 1;
        return stats;
    }

    const uint8_t kFullCoverage = (uint8_t)(AA_GRID * AA_GRID);
    for (size_t i = 0; i < pixelCount; ++i) {
        if (!valid[i]) {
            continue;
        }
        ++stats.validCount;
        const uint8_t r = page.pixels[i * 4 + 0];
        const uint8_t g = page.pixels[i * 4 + 1];
        const uint8_t b = page.pixels[i * 4 + 2];
        const bool dark = r < 8 && g < 8 && b < 8;
        if (dark) {
            ++stats.darkValidCount;
        }
        if (coverage[i] == kFullCoverage) {
            ++stats.fullCoverageCount;
            if (dark) {
                ++stats.darkFullCoverageCount;
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

    // Full-coverage interior luxels are the reliable signal for a broken GPU bake.
    if (stats.fullCoverageCount > 0) {
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
    light.attenuationMode = POINT_LIGHT_ATTEN_NONE;
    light.angleScale = angleScale;
    light.dirt = (int8_t)dirtOverride;
    return light;
}

static std::vector<Vector3> GenerateSurfaceEmitterSamples(const MapPolygon& poly,
                                                          float sampleSpacing,
                                                          float offsetAlongNormal)
{
    Vector3 axisU{};
    Vector3 axisV{};
    FaceBasis(poly.normal, axisU, axisV);

    float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
    std::vector<Vector2> poly2d;
    poly2d.reserve(poly.verts.size());
    const Vector3 anchor = poly.verts[0];
    const float anchorU = Vector3DotProduct(anchor, axisU);
    const float anchorV = Vector3DotProduct(anchor, axisV);
    for (const Vector3& vert : poly.verts) {
        const float u = Vector3DotProduct(vert, axisU);
        const float v = Vector3DotProduct(vert, axisV);
        minU = std::min(minU, u);
        maxU = std::max(maxU, u);
        minV = std::min(minV, v);
        maxV = std::max(maxV, v);
        poly2d.push_back({u, v});
    }

    const float safeSpacing = std::max(16.0f, sampleSpacing);
    const float extentU = std::max(0.0f, maxU - minU);
    const float extentV = std::max(0.0f, maxV - minV);
    const int samplesU = std::max(1, std::min(32, (int)ceilf(extentU / safeSpacing)));
    const int samplesV = std::max(1, std::min(32, (int)ceilf(extentV / safeSpacing)));
    const float stepU = (samplesU > 0) ? (extentU / (float)samplesU) : 0.0f;
    const float stepV = (samplesV > 0) ? (extentV / (float)samplesV) : 0.0f;

    std::vector<Vector3> samples;
    samples.reserve((size_t)samplesU * (size_t)samplesV);
    for (int y = 0; y < samplesV; ++y) {
        for (int x = 0; x < samplesU; ++x) {
            const float u = minU + ((float)x + 0.5f) * stepU;
            const float v = minV + ((float)y + 0.5f) * stepV;
            if (!InsidePoly2D(poly2d, u, v)) {
                continue;
            }
            Vector3 p = anchor;
            p = Vector3Add(p, Vector3Scale(axisU, u - anchorU));
            p = Vector3Add(p, Vector3Scale(axisV, v - anchorV));
            p = Vector3Add(p, Vector3Scale(poly.normal, offsetAlongNormal));
            samples.push_back(p);
        }
    }

    if (samples.empty()) {
        samples.push_back(Vector3Add(PolygonCentroid(poly.verts), Vector3Scale(poly.normal, offsetAlongNormal)));
    }
    return samples;
}

static std::vector<PointLight> BuildExpandedLights(const std::vector<MapPolygon>& polys,
                                                   const std::vector<PointLight>& explicitLights,
                                                   const std::vector<SurfaceLightTemplate>& surfaceLights,
                                                   const LightBakeSettings& settings,
                                                   const Vector3& mapCenter,
                                                   float sunDistance)
{
    std::vector<PointLight> lights = explicitLights;

    if (settings.sunlightIntensity > 0.0f) {
        const int sunSamples = (settings.sunlightPenumbra > 0.1f) ? ERICW_SUNSAMPLES : 1;
        const Vector3 sampleColor = Vector3Scale(settings.sunlightColor, settings.sunlightIntensity / (300.0f * (float)sunSamples));
        const int dirtOverride = (settings.sunlightDirt == -2) ? settings.dirt : settings.sunlightDirt;
        for (int i = 0; i < sunSamples; ++i) {
            const Vector3 toLight = SampleConeDirection(settings.sunlightDirection, settings.sunlightPenumbra, i, sunSamples);
            lights.push_back(BuildDirectionalSunLight(toLight, sampleColor, settings.sunlightAngleScale, dirtOverride, mapCenter, sunDistance));
        }
    }

    int surfaceTemplateMatches = 0;
    for (const SurfaceLightTemplate& templ : surfaceLights) {
        for (const MapPolygon& poly : polys) {
            if (poly.texture != templ.texture) {
                continue;
            }
            if (templ.surfaceLightGroup != 0 && poly.surfaceLightGroup != templ.surfaceLightGroup) {
                continue;
            }

            const std::vector<Vector3> samples = GenerateSurfaceEmitterSamples(poly, 128.0f, std::max(0.0f, templ.surfaceOffset));
            for (const Vector3& samplePos : samples) {
                PointLight emitted = templ.light;
                emitted.position = samplePos;
                if (templ.surfaceSpotlight) {
                    emitted.spotDirection = poly.normal;
                    if (emitted.spotOuterCos <= -1.5f) {
                        emitted.spotOuterCos = cosf(20.0f * DEG2RAD);
                        emitted.spotInnerCos = emitted.spotOuterCos;
                    }
                }
                AppendDeviatedLights(emitted, templ.deviance, templ.devianceSamples, ++surfaceTemplateMatches, lights);
            }
        }
    }

    if (!surfaceLights.empty()) {
        printf("[Lightmap] expanded %zu surface-light templates into %zu total lights.\n", surfaceLights.size(), lights.size());
        fflush(stdout);
    }
    return lights;
}

static bool IsLightBrushTextureName(const std::string& name) {
    return name.rfind("__light_brush_", 0) == 0;
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

    const uint8_t kFullCoverage = (uint8_t)(AA_GRID * AA_GRID);
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
                page.pixels[off + 0] / 255.0f,
                page.pixels[off + 1] / 255.0f,
                page.pixels[off + 2] / 255.0f
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

static std::vector<PointLight> BuildIndirectBounceLights(const std::vector<BakePatch>& patches,
                                                         const std::vector<FaceRect>& rects,
                                                         const std::vector<LightmapPage>& pages,
                                                         const std::vector<std::vector<uint8_t>>& coverageMasks,
                                                         const Vector3& ambientColor,
                                                         float bounceScale)
{
    std::vector<PointLight> bounceLights;
    if (bounceScale <= 0.0f) {
        return bounceLights;
    }
    bounceLights.reserve(patches.size());
    for (size_t i = 0; i < patches.size(); ++i) {
        const BakePatch& patch = patches[i];
        const FaceRect& rect = rects[i];
        if (rect.page >= pages.size() || rect.page >= coverageMasks.size()) {
            continue;
        }
        if (IsLightBrushTextureName(patch.poly.texture)) {
            continue;
        }

        const Vector3 avgDirect = AverageRectLighting(pages[rect.page], rect, coverageMasks[rect.page], ambientColor);
        const float maxChannel = std::max(avgDirect.x, std::max(avgDirect.y, avgDirect.z));
        if (maxChannel < INDIRECT_BOUNCE_THRESHOLD) {
            continue;
        }

        const float area = std::max(1.0f, PolygonArea(patch.poly.verts));
        const float radius = std::clamp(
            INDIRECT_BOUNCE_RADIUS_BIAS + sqrtf(area) * INDIRECT_BOUNCE_RADIUS_SCALE,
            INDIRECT_BOUNCE_RADIUS_MIN,
            INDIRECT_BOUNCE_RADIUS_MAX);

        PointLight bounce{};
        bounce.position = Vector3Add(PolygonCentroid(patch.poly.verts), Vector3Scale(patch.poly.normal, 1.0f));
        bounce.color = Vector3Scale(avgDirect, INDIRECT_BOUNCE_REFLECTANCE * bounceScale);
        bounce.intensity = radius;
        bounce.emissionNormal = patch.poly.normal;
        bounce.directional = 1;
        bounce.ignoreOccluderGroup = -1;
        bounce.attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
        bounce.dirt = -1;
        bounceLights.push_back(bounce);
    }
    return bounceLights;
}

static LightmapPage MakeBlankPageLike(const LightmapPage& src) {
    LightmapPage page;
    page.width = src.width;
    page.height = src.height;
    page.pixels.assign((size_t)page.width * (size_t)page.height * 4, 0);
    return page;
}

static void AddPagePixels(LightmapPage& dst, const LightmapPage& src) {
    if (dst.pixels.size() != src.pixels.size()) {
        return;
    }
    for (size_t i = 0; i + 3 < dst.pixels.size(); i += 4) {
        dst.pixels[i + 0] = (uint8_t)std::min(255, (int)dst.pixels[i + 0] + (int)src.pixels[i + 0]);
        dst.pixels[i + 1] = (uint8_t)std::min(255, (int)dst.pixels[i + 1] + (int)src.pixels[i + 1]);
        dst.pixels[i + 2] = (uint8_t)std::min(255, (int)dst.pixels[i + 2] + (int)src.pixels[i + 2]);
        dst.pixels[i + 3] = 255;
    }
}

static Vector3 ComputeLuxelPlanePoint(const FaceRect& rect, float ju, float jv) {
    return Vector3Add(
        rect.gpu.origin,
        Vector3Add(Vector3Scale(rect.gpu.axisU, ju * rect.gpu.luxelSize),
                   Vector3Scale(rect.gpu.axisV, jv * rect.gpu.luxelSize)));
}

static Vector3 OffsetSamplePointOffSurface(const Vector3& planePoint, const Vector3& faceNormal) {
    return Vector3Add(planePoint, Vector3Scale(Vector3Normalize(faceNormal), SURFACE_SAMPLE_OFFSET));
}

static float ComputeDirtAttenuation(float occlusionRatio, float dirtScale, float dirtGain) {
    const float scaled = std::clamp(occlusionRatio * std::max(0.0f, dirtScale), 0.0f, 1.0f);
    const float gained = powf(scaled, std::max(0.01f, dirtGain));
    return std::clamp(1.0f - gained, 0.0f, 1.0f);
}

static float ComputeDirtOcclusionRatio(const OccluderSet& occ,
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
        const float hitDistance = ClosestHitDistance(
            occ,
            Vector3Add(samplePoint, Vector3Scale(dir, SHADOW_BIAS)),
            dir,
            OCCLUSION_NEAR_TMIN,
            depth,
            -1);
        accumulatedDistance += std::min(depth, hitDistance);
    }
    const float avgHitDistance = accumulatedDistance / (float)DIRT_RAY_COUNT;
    return std::clamp(1.0f - (avgHitDistance / depth), 0.0f, 1.0f);
}

static bool TryRepairSampleCandidate(const std::vector<BrushSolid>& solids,
                                     const Vector3& planePoint,
                                     const Vector3& faceNormal,
                                     RepairedSamplePoint* ioSample)
{
    if (!ioSample) {
        return false;
    }

    ioSample->planePoint = planePoint;
    ioSample->samplePoint = OffsetSamplePointOffSurface(planePoint, faceNormal);
    if (!PointInsideAnySolid(solids, ioSample->samplePoint, SOLID_REPAIR_EPSILON)) {
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
                    return true;
                }
            }
        }
    }

    return false;
}

static bool SourcePolyVisited(const std::vector<uint32_t>& visitedPath, uint32_t sourcePolyIndex) {
    return std::find(visitedPath.begin(), visitedPath.end(), sourcePolyIndex) != visitedPath.end();
}

static bool TryRecursiveRepairWalk(const std::vector<RepairSourcePoly>& repairPolys,
                                   const std::vector<BrushSolid>& solids,
                                   uint32_t sourcePolyIndex,
                                   const Vector3& seedPoint,
                                   float luxelSize,
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
    const Vector3 faceSeedPoint = OffsetSamplePointOffSurface(projected, poly.normal);
    const bool insideFace = InsidePoly2D(poly.poly2d, projectedUv.x, projectedUv.y);
    if (insideFace) {
        RepairedSamplePoint candidate{};
        candidate.sourcePolyIndex = sourcePolyIndex;
        if (TryRepairSampleCandidate(solids, projected, poly.normal, &candidate)) {
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
        boundarySample.sourcePolyIndex = sourcePolyIndex;
        if (TryRepairSampleCandidate(solids, snappedPoint, poly.normal, &boundarySample)) {
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
                                             float luxelSize)
{
    RepairedSamplePoint repaired{};
    repaired.sourcePolyIndex = rect.sourcePolyIndex;
    repaired.planePoint = planePoint;
    const Vector3 baseNormal = (rect.sourcePolyIndex < repairPolys.size() &&
                                Vector3LengthSq(repairPolys[rect.sourcePolyIndex].normal) > 1e-8f)
        ? repairPolys[rect.sourcePolyIndex].normal
        : rect.gpu.normal;
    repaired.samplePoint = OffsetSamplePointOffSurface(planePoint, baseNormal);
    if (!PointInsideAnySolid(solids, repaired.samplePoint, SOLID_REPAIR_EPSILON)) {
        return repaired;
    }

    if (TryRepairSampleCandidate(solids, planePoint, baseNormal, &repaired)) {
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
            const Vector3 ro = Vector3Add(samplePoint, Vector3Scale(dir, SHADOW_BIAS));
            if (!Occluded(occ, ro, dir, nearHitT, skyTraceDistance, -1)) {
                const float incidence = EvaluateAngleScale(settings.sunlightAngleScale, Vector3DotProduct(sampleNormal, dir));
                const float scale = upperPerSample * incidence * dirtScale;
                contrib = Vector3Add(contrib, Vector3Scale(settings.sunlight2Color, scale));
            }
        }
        if (lowerPerSample > 0.0f) {
            const Vector3 dir = EricwSkyDomeDirection(false, i, lowerRotation);
            const Vector3 ro = Vector3Add(samplePoint, Vector3Scale(dir, SHADOW_BIAS));
            if (!Occluded(occ, ro, dir, nearHitT, skyTraceDistance, -1)) {
                const float incidence = EvaluateAngleScale(settings.sunlightAngleScale, Vector3DotProduct(sampleNormal, dir));
                const float scale = lowerPerSample * incidence * dirtScale;
                contrib = Vector3Add(contrib, Vector3Scale(settings.sunlight3Color, scale));
            }
        }
    }

    return contrib;
}

static bool PageUsesCPUOnlyLightingFeatures(uint32_t pageIndex,
                                            const std::vector<FaceRect>& rects,
                                            const std::vector<PhongSourcePoly>& sourcePhongs,
                                            const std::vector<PointLight>& pageLights,
                                            const LightBakeSettings& settings,
                                            std::string* reason)
{
    (void)pageIndex;
    (void)rects;
    (void)sourcePhongs;
    (void)pageLights;
    (void)settings;
    if (reason) {
        reason->clear();
    }
    return false;
}

static void BakeLightmapCPUPage(const std::vector<BakePatch>& patches,
                                const std::vector<PhongSourcePoly>& sourcePhongs,
                                const std::vector<RepairSourcePoly>& repairPolys,
                                const std::vector<PointLight>& lights,
                                const std::vector<FaceRect>& rects,
                                const std::vector<std::vector<uint32_t>>* lightIndicesByRect,
                                uint32_t pageIndex,
                                const OccluderSet& occ,
                                const std::vector<BrushSolid>& repairSolids,
                                const LightBakeSettings& settings,
                                float skyTraceDistance,
                                LightmapPage& page)
{
    const int W = page.width;
    const int SAMPLES = AA_GRID * AA_GRID;
    const float invG = 1.0f / (float)AA_GRID;

    for (size_t i = 0; i < patches.size(); ++i) {
        const MapPolygon& p = patches[i].poly;
        const FaceRect& r = rects[i];
        if (r.page != pageIndex) {
            continue;
        }

        for (int ly = 0; ly < r.gpu.h; ++ly) {
            for (int lx = 0; lx < r.gpu.w; ++lx) {
                float ar = 0, ag = 0, ab = 0;
                int usedSamples = 0;

                for (int sy = 0; sy < AA_GRID; ++sy) {
                    for (int sx = 0; sx < AA_GRID; ++sx) {
                        const float ju = (lx - LM_PAD) + (sx + 0.5f) * invG;
                        const float jv = (ly - LM_PAD) + (sy + 0.5f) * invG;
                        if (!InsidePoly2D(r.poly2d, ju, jv)) {
                            continue;
                        }
                        const Vector3 planePoint = ComputeLuxelPlanePoint(r, ju, jv);
                        const RepairedSamplePoint repairedSample = RepairSamplePoint(r, repairPolys, repairSolids, planePoint, r.gpu.luxelSize);
                        Vector3 sampleNormal = EvaluatePhongNormal(sourcePhongs, repairedSample.sourcePolyIndex, repairedSample.planePoint, r.gpu.luxelSize);
                        if (Vector3LengthSq(sampleNormal) <= 1e-8f) {
                            sampleNormal = r.gpu.normal;
                        }
                        const Vector3 samplePoint = repairedSample.samplePoint;
                        const float edgeDistLuxels = MinDistToPolyEdge2D(r.poly2d, ju, jv);
                        const float edgeFactor = std::clamp(
                            1.0f - edgeDistLuxels / EDGE_SEAM_GUARD_LUXELS,
                            0.0f,
                            1.0f);
                        const float nearHitT = OCCLUSION_NEAR_TMIN + EDGE_SEAM_TMIN_BOOST * edgeFactor;

                        float cr = settings.ambientColor.x;
                        float cg = settings.ambientColor.y;
                        float cb = settings.ambientColor.z;
                        const std::vector<uint32_t>& rectLightIndices = lightIndicesByRect ? (*lightIndicesByRect)[i] : r.lightIndices;
                        bool usesDirt = false;
                        for (uint32_t lightIndex : rectLightIndices) {
                            if (LightUsesDirt(lights[lightIndex], settings)) {
                                usesDirt = true;
                                break;
                                }
                        }
                        usesDirt = usesDirt || SkyDomeUsesDirt(settings);
                        const float dirtOcclusion = usesDirt ? ComputeDirtOcclusionRatio(occ, samplePoint, sampleNormal, settings) : 0.0f;
                        for (uint32_t lightIndex : rectLightIndices) {
                            const PointLight& L = lights[lightIndex];
                            Vector3 dir{};
                            float dist = 0.0f;
                            float att = 1.0f;
                            if (IsParallelLight(L)) {
                                dir = Vector3Normalize(L.parallelDirection);
                                dist = std::max(1.0f, L.intensity);
                            } else {
                                const Vector3 toL = Vector3Subtract(L.position, samplePoint);
                                dist = Vector3Length(toL);
                                if (dist > L.intensity || dist < 1e-3f) continue;
                                dir = Vector3Scale(toL, 1.f / dist);
                                att = EvaluateLightAttenuation(L, dist);
                                if (att <= 0.0f) continue;
                            }
                            const Vector3 ro = Vector3Add(samplePoint, Vector3Scale(dir, SHADOW_BIAS));
                            float emit = 1.0f;
                            if (L.directional) {
                                const Vector3 lightToSurface = Vector3Scale(dir, -1.0f);
                                emit = Vector3DotProduct(L.emissionNormal, lightToSurface);
                                if (emit <= 0.0f) continue;
                            }
                            const float spot = EvaluateSpotlightFactor(L, dir);
                            if (spot <= 0.0f) continue;
                            const float ndl = Vector3DotProduct(sampleNormal, dir);
                            const float incidence = EvaluateIncidenceScale(L, ndl);
                            if (incidence <= 0.0f) continue;
                            if (Occluded(occ, ro, dir, nearHitT, std::max(0.0f, dist - (SHADOW_BIAS * 2.0f)), L.ignoreOccluderGroup)) continue;
                            const float dirt = LightUsesDirt(L, settings)
                                ? ComputeDirtAttenuation(dirtOcclusion, EffectiveLightDirtScale(L, settings), EffectiveLightDirtGain(L, settings))
                                : 1.0f;
                            const float contrib = emit * spot * incidence * att * dirt;
                            cr += L.color.x * contrib;
                            cg += L.color.y * contrib;
                            cb += L.color.z * contrib;
                        }
                        const Vector3 skyContrib = ComputeSkyDomeContribution(
                            occ, samplePoint, sampleNormal, nearHitT, skyTraceDistance, dirtOcclusion, settings);
                        cr += skyContrib.x;
                        cg += skyContrib.y;
                        cb += skyContrib.z;
                        ar += cr; ag += cg; ab += cb;
                        ++usedSamples;
                    }
                }

                if (usedSamples > 0) {
                    const float invSamples = 1.0f / (float)usedSamples;
                    ar *= invSamples;
                    ag *= invSamples;
                    ab *= invSamples;
                }
                const size_t off = ((size_t)(r.gpu.y + ly) * W + (r.gpu.x + lx)) * 4;
                auto C = [](float v) { return (uint8_t)std::min(255, (int)(v * 255.f)); };
                page.pixels[off + 0] = C(ar);
                page.pixels[off + 1] = C(ag);
                page.pixels[off + 2] = C(ab);
                page.pixels[off + 3] = 255;
            }
        }
    }
}

// --------------------------------------------------------------------------
LightmapAtlas BakeLightmap(const std::vector<MapPolygon>& polys,
                           const std::vector<MapPolygon>& occluderPolys,
                           const std::vector<PointLight>& lights,
                           const std::vector<SurfaceLightTemplate>& surfaceLights,
                           const LightBakeSettings& settings)
{
    LightmapAtlas atlas;
    const float luxelSize = std::max(0.125f, settings.luxelSize);
    const SurfaceVisibilityInfo sourceVisibility = AnalyzeSurfaceVisibility(polys);
    const std::vector<MapPolygon> visiblePolys = FilterVisibleBakePolygons(polys, sourceVisibility);
    if (sourceVisibility.hiddenCount > 0) {
        printf("[Lightmap] skipped %d hidden/internal brush faces from bake surfaces.\n",
               sourceVisibility.hiddenCount);
        fflush(stdout);
    }
    const std::vector<PhongSourcePoly> sourcePhongs = BuildPhongSourcePolys(visiblePolys);
    const std::vector<RepairSourcePoly> repairPolys = BuildRepairSourcePolys(visiblePolys, sourcePhongs);
    const AABB visibleBounds = ComputeMapBounds(visiblePolys);
    const Vector3 mapCenter = Vector3Scale(Vector3Add(visibleBounds.min, visibleBounds.max), 0.5f);
    const float skyTraceDistance = std::max(2048.0f, sqrtf(Vector3LengthSq(Vector3Subtract(visibleBounds.max, visibleBounds.min))) * 2.0f + 1024.0f);
    const std::vector<PointLight> allLights = BuildExpandedLights(visiblePolys, lights, surfaceLights, settings, mapCenter, skyTraceDistance);
    std::vector<BakePatch> patches = SubdivideLightmappedPolygons(visiblePolys, luxelSize);
    std::vector<FaceRect> rects = BuildFaceRects(patches, allLights, repairPolys, sourcePhongs, luxelSize);
    const std::vector<BrushSolid> repairSolids = BuildBrushSolids(polys);

    std::vector<LightmapPageLayout> layouts = PackLightmapPages(rects);
    if (layouts.empty() && !rects.empty()) {
        printf("[Lightmap] failed to pack lightmap pages.\n");
        return atlas;
    }

    atlas.pages.resize(layouts.size());
    for (size_t i = 0; i < layouts.size(); ++i) {
        atlas.pages[i].width = LIGHTMAP_PAGE_SIZE;
        atlas.pages[i].height = layouts[i].usedHeight;
        atlas.pages[i].pixels.assign((size_t)atlas.pages[i].width * (size_t)atlas.pages[i].height * 4, 0);
    }

    FillPatchUVs(patches, rects, atlas.pages, atlas);

    size_t totalLuxels = 0;
    for (const FaceRect& r : rects) {
        totalLuxels += (size_t)r.gpu.w * (size_t)r.gpu.h;
    }

    OccluderSet occ = BuildOccluders(occluderPolys.empty() ? visiblePolys : occluderPolys);
    std::vector<LightmapComputeBrushSolid> computeBrushSolids;
    std::vector<LightmapComputeSolidPlane> computeSolidPlanes;
    std::vector<LightmapComputeRepairSourcePoly> computeRepairSourcePolys;
    std::vector<LightmapComputeRepairSourceNeighbor> computeRepairSourceNeighbors;
    BuildComputeBrushSolids(repairSolids, &computeBrushSolids, &computeSolidPlanes);
    BuildComputeRepairSourceGraph(repairPolys, &computeRepairSourcePolys, &computeRepairSourceNeighbors);
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
    printf("[Lightmap] %zu source faces -> %zu bake patches across %zu pages of up to %dx%d, %zu explicit/direct lights, %zu tris, %zu luxels x %d samples = %zu rays/light (luxel=%.3f, ambient=%.2f/%.2f/%.2f, bounces=%d, bounceScale=%.2f)\n",
           visiblePolys.size(), patches.size(), atlas.pages.size(), LIGHTMAP_PAGE_SIZE, LIGHTMAP_PAGE_SIZE,
           allLights.size(), occ.tris.size(),
           totalLuxels, AA_GRID * AA_GRID, totalLuxels * (size_t)(AA_GRID * AA_GRID),
           luxelSize,
           settings.ambientColor.x, settings.ambientColor.y, settings.ambientColor.z,
           settings.bounceCount, settings.bounceScale);
    fflush(stdout);

    std::vector<std::vector<uint8_t>> coverageMasks(atlas.pages.size());
    std::vector<std::vector<uint8_t>> baseValidMasks(atlas.pages.size());
    for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
        LightmapPage& page = atlas.pages[pageIndex];
        printf("[Lightmap] page %u/%zu begin (%dx%d)\n",
               pageIndex + 1, atlas.pages.size(), page.width, page.height);
        fflush(stdout);
        std::vector<uint8_t>& coverage = coverageMasks[pageIndex];
        coverage = BuildCoverageMask(rects, pageIndex, page.width, page.height);
        std::vector<uint8_t>& valid = baseValidMasks[pageIndex];
        valid = CoverageToValidMask(coverage);
        std::vector<PointLight> pageLights = GatherPageLights(rects, pageIndex, allLights);
        size_t pageLuxels = 0;

        std::vector<LightmapComputeFaceRect> pageRects;
        pageRects.reserve(rects.size());
        bool pageRequiresCPU = false;
        std::string computeError;
        for (const FaceRect& r : rects) {
            if (r.page == pageIndex) {
                pageRects.push_back(r.gpu);
                pageLuxels += (size_t)r.gpu.w * (size_t)r.gpu.h;
                if (!r.computeCompatible) {
                    pageRequiresCPU = true;
                    if (computeError.empty()) {
                        computeError = r.computeFallbackReason;
                    }
                }
            }
        }
        printf("[Lightmap] page %u stats: %zu rects, %zu explicit/direct lights%s%s, %zu luxels\n",
               pageIndex,
               pageRects.size(),
               pageLights.size(),
               (settings.sunlight2Intensity > 0.0f) ? ", + upper skylight" : "",
               (settings.sunlight3Intensity > 0.0f) ? ", + lower skylight" : "",
               pageLuxels);
        fflush(stdout);

        bool usedCPUFallback = false;
        if (!pageRequiresCPU && PageUsesCPUOnlyLightingFeatures(pageIndex, rects, sourcePhongs, pageLights, settings, &computeError)) {
            pageRequiresCPU = true;
        }
        if (pageRequiresCPU || !BakeLightmapCompute(pageRects, occ.tris, computeBrushSolids, computeSolidPlanes, computeRepairSourcePolys, computeRepairSourceNeighbors, pageLights, settings, skyTraceDistance, page.width, page.height, page.pixels, &computeError)) {
            printf("[Lightmap] page %u compute bake unavailable, falling back to CPU: %s\n",
                   pageIndex, computeError.c_str());
            fflush(stdout);
            BakeLightmapCPUPage(patches, sourcePhongs, repairPolys, allLights, rects, nullptr, pageIndex, occ, repairSolids, settings, skyTraceDistance, page);
            usedCPUFallback = true;
            printf("[Lightmap] page %u CPU bake complete\n", pageIndex);
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

            const DarkLuxelStats darkStats = GatherDarkLuxelStats(page, valid, coverage);
            if (HasInvalidDarkValidLuxels(darkStats)) {
                printf("[Lightmap] page %u compute bake produced invalid dark valid texels "
                       "(dark valid=%zu/%zu, dark full=%zu/%zu), falling back to CPU.\n",
                       pageIndex,
                       darkStats.darkValidCount, darkStats.validCount,
                       darkStats.darkFullCoverageCount, darkStats.fullCoverageCount);
                fflush(stdout);
                BakeLightmapCPUPage(patches, sourcePhongs, repairPolys, allLights, rects, nullptr, pageIndex, occ, repairSolids, settings, skyTraceDistance, page);
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
        const std::vector<PointLight> indirectLights = BuildIndirectBounceLights(
            patches, rects, bounceSourcePages, coverageMasks, bounceAmbient, settings.bounceScale);
        if (indirectLights.empty()) {
            if (bouncePass == 0) {
                printf("[Lightmap] indirect bounce emitters: 0\n");
                fflush(stdout);
            }
            break;
        }

        printf("[Lightmap] bounce pass %d/%d emitters: %zu\n",
               bouncePass + 1, settings.bounceCount, indirectLights.size());
        fflush(stdout);
        const std::vector<std::vector<uint32_t>> indirectLightIndices = BuildRectLightIndices(rects, indirectLights);
        std::vector<LightmapPage> bouncedPages = atlas.pages;
        for (LightmapPage& bouncedPage : bouncedPages) {
            bouncedPage = MakeBlankPageLike(bouncedPage);
        }

        for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
            LightmapPage& page = atlas.pages[pageIndex];
            LightmapPage& bouncedPage = bouncedPages[pageIndex];
            const std::vector<PointLight> pageIndirectLights = GatherPageLights(rects, pageIndex, indirectLights, &indirectLightIndices);
            if (pageIndirectLights.empty()) {
                continue;
            }

            std::vector<LightmapComputeFaceRect> pageRects;
            pageRects.reserve(rects.size());
            bool pageRequiresCPU = false;
            std::string computeError;
            for (const FaceRect& r : rects) {
                if (r.page == pageIndex) {
                    pageRects.push_back(r.gpu);
                    if (!r.computeCompatible) {
                        pageRequiresCPU = true;
                        if (computeError.empty()) {
                            computeError = r.computeFallbackReason;
                        }
                    }
                }
            }

            printf("[Lightmap] page %u bounce %d begin (%zu bounce lights)\n",
                   pageIndex, bouncePass + 1, pageIndirectLights.size());
            fflush(stdout);
            if (!pageRequiresCPU && PageUsesCPUOnlyLightingFeatures(pageIndex, rects, sourcePhongs, pageIndirectLights, settings, &computeError)) {
                pageRequiresCPU = true;
            }
            LightBakeSettings bounceSettings = settings;
            bounceSettings.ambientColor = Vector3Zero();
            bounceSettings.sunlight2Intensity = 0.0f;
            bounceSettings.sunlight3Intensity = 0.0f;
            if (pageRequiresCPU || !BakeLightmapCompute(pageRects, occ.tris, computeBrushSolids, computeSolidPlanes, computeRepairSourcePolys, computeRepairSourceNeighbors, pageIndirectLights, bounceSettings, skyTraceDistance, page.width, page.height, bouncedPage.pixels, &computeError)) {
                printf("[Lightmap] page %u bounce %d compute unavailable, falling back to CPU: %s\n",
                       pageIndex, bouncePass + 1, computeError.c_str());
                fflush(stdout);
                BakeLightmapCPUPage(patches, sourcePhongs, repairPolys, indirectLights, rects, &indirectLightIndices, pageIndex, occ, repairSolids, bounceSettings, skyTraceDistance, bouncedPage);
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

        bounceSourcePages = std::move(bouncedPages);
        bounceAmbient = Vector3Zero();
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
