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
static constexpr float DEFAULT_LUXEL_SIZE = 16.0f;   // world units per luxel
static constexpr int   LM_PAD             = 2;      // border luxels (filled by dilate)
static constexpr int   LIGHTMAP_PAGE_SIZE = 2048;
static constexpr int   FACE_MAX_LUXELS    = LIGHTMAP_PAGE_SIZE - LM_PAD * 2 - 4;
static constexpr float AMBIENT            = 0.12f;
static constexpr float SHADOW_BIAS        = 0.25f;
static constexpr float RAY_EPS            = 1e-4f;
static constexpr float OCCLUSION_NEAR_TMIN = SHADOW_BIAS;
static constexpr float EDGE_SEAM_GUARD_LUXELS = 0.75f;
static constexpr float EDGE_SEAM_TMIN_BOOST   = SHADOW_BIAS * 3.0f;
static constexpr int   AA_GRID            = 4;      // AA_GRID² samples per luxel
static constexpr int   DILATE_PASSES      = 4;

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

static Vector3 LerpVec3(const Vector3& a, const Vector3& b, float t) {
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
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
                                                          uint32_t sourcePolyIndex) {
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

    const float tileExtent = FACE_MAX_LUXELS * DEFAULT_LUXEL_SIZE;
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

static std::vector<BakePatch> SubdivideLightmappedPolygons(const std::vector<MapPolygon>& polys) {
    std::vector<BakePatch> out;
    out.reserve(polys.size());

    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& poly = polys[i];
        std::vector<BakePatch> parts = SubdivideLightmappedPolygon(poly, (uint32_t)i);
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

struct BrushSolidPlane {
    Vector3 point{};
    Vector3 normal{};
};

struct BrushSolid {
    int sourceBrushId = -1;
    std::vector<BrushSolidPlane> planes;
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
        solids[it->second].planes.push_back({ poly.verts[0], Vector3Normalize(poly.normal) });
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

static OccluderSet BuildOccluders(const std::vector<MapPolygon>& polys) {
    OccluderSet o;
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

    std::vector<uint8_t> hidden(polys.size(), 0);
    std::vector<uint8_t> emitterOnly(polys.size(), 0);
    for (size_t i = 0; i < polys.size(); ++i) {
        if (polys[i].occluderGroup >= 0) {
            emitterOnly[i] = 1;
        }
    }
    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& poly = polys[i];
        if (emitterOnly[i]) {
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
            hidden[i] = 1;
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
            hidden[i] = 1;
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
                hidden[i] = 1;
                break;
            }
        }
    }

    int hiddenCount = 0;
    int emitterOnlyCount = 0;
    for (size_t i = 0; i < polys.size(); ++i) {
        if (emitterOnly[i]) {
            ++emitterOnlyCount;
            continue;
        }
        if (hidden[i]) {
            ++hiddenCount;
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
    if (hiddenCount > 0) {
        printf("[Lightmap] skipped %d hidden/internal brush faces from occluders.\n", hiddenCount);
    }
    if (emitterOnlyCount > 0) {
        printf("[Lightmap] skipped %d light brush faces from occluders.\n", emitterOnlyCount);
    }
    return o;
}

static bool RayTri(const Vector3& ro, const Vector3& rd,
                   const LightmapComputeOccluderTri& tr, float tmin, float tmax)
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
    return t > tmin && t < tmax;
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
        if (RayTri(ro, rd, tri, minHitT, dist)) return true;
    }
    return false;
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

struct FaceRect {
    LightmapComputeFaceRect gpu;
    std::vector<Vector2> poly2d;
    AABB bounds{};
    std::vector<uint32_t> lightIndices;
    uint32_t page = 0;
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
                                            const std::vector<PointLight>& lights) {
    std::vector<FaceRect> rects(patches.size());
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

        r.gpu.w = std::max(2, (int)ceilf((maxU - minU) / DEFAULT_LUXEL_SIZE)) + LM_PAD * 2;
        r.gpu.h = std::max(2, (int)ceilf((maxV - minV) / DEFAULT_LUXEL_SIZE)) + LM_PAD * 2;
        r.gpu.luxelSize = DEFAULT_LUXEL_SIZE;
        r.gpu.minU = minU;
        r.gpu.minV = minV;
        r.gpu.axisU = U;
        r.gpu.axisV = V;
        r.gpu.normal = p.normal;
        const float d = Vector3DotProduct(p.verts[0], p.normal);
        r.gpu.origin = Vector3Add(
            Vector3Scale(p.normal, d),
            Vector3Add(Vector3Scale(U, minU), Vector3Scale(V, minV)));
        r.poly2d.reserve(p.verts.size());
        for (const Vector3& vv : p.verts) {
            r.poly2d.push_back({
                (Vector3DotProduct(vv, U) - minU) / DEFAULT_LUXEL_SIZE,
                (Vector3DotProduct(vv, V) - minV) / DEFAULT_LUXEL_SIZE
            });
        }
        if (r.poly2d.size() > LIGHTMAP_COMPUTE_MAX_POLY_VERTS) {
            r.gpu.polyCount = -1;
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
            const float radiusSq = light.intensity * light.intensity;
            if (DistSqPointAABB(light.position, r.bounds) <= radiusSq) {
                r.lightIndices.push_back(lightIndex);
            }
        }
    }
    return rects;
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

static std::vector<PointLight> GatherPageLights(const std::vector<FaceRect>& rects,
                                                uint32_t pageIndex,
                                                const std::vector<PointLight>& lights)
{
    if (lights.empty()) {
        return {};
    }

    std::vector<uint8_t> used(lights.size(), 0);
    for (const FaceRect& r : rects) {
        if (r.page != pageIndex) {
            continue;
        }
        for (uint32_t lightIndex : r.lightIndices) {
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

static void BakeLightmapCPUPage(const std::vector<BakePatch>& patches,
                                const std::vector<PointLight>& lights,
                                const std::vector<FaceRect>& rects,
                                uint32_t pageIndex,
                                const OccluderSet& occ,
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
                        const Vector3 wp = Vector3Add(
                            r.gpu.origin,
                            Vector3Add(Vector3Scale(r.gpu.axisU, ju * r.gpu.luxelSize),
                                       Vector3Scale(r.gpu.axisV, jv * r.gpu.luxelSize)));
                        const float edgeDistLuxels = MinDistToPolyEdge2D(r.poly2d, ju, jv);
                        const float edgeFactor = std::clamp(
                            1.0f - edgeDistLuxels / EDGE_SEAM_GUARD_LUXELS,
                            0.0f,
                            1.0f);
                        const float nearHitT = OCCLUSION_NEAR_TMIN + EDGE_SEAM_TMIN_BOOST * edgeFactor;

                        float cr = AMBIENT, cg = AMBIENT, cb = AMBIENT;
                        for (uint32_t lightIndex : r.lightIndices) {
                            const PointLight& L = lights[lightIndex];
                            const Vector3 toL = Vector3Subtract(L.position, wp);
                            const float dist = Vector3Length(toL);
                            if (dist > L.intensity || dist < 1e-3f) continue;
                            const Vector3 dir = Vector3Scale(toL, 1.f / dist);
                            const Vector3 ro = Vector3Add(
                                Vector3Add(wp, Vector3Scale(p.normal, SHADOW_BIAS)),
                                Vector3Scale(dir, SHADOW_BIAS));
                            float emit = 1.0f;
                            if (L.directional) {
                                const Vector3 lightToSurface = Vector3Scale(dir, -1.0f);
                                emit = Vector3DotProduct(L.emissionNormal, lightToSurface);
                                if (emit <= 0.0f) continue;
                            }
                            const float ndl = Vector3DotProduct(p.normal, dir);
                            if (ndl <= 0) continue;
                            if (Occluded(occ, ro, dir, nearHitT, std::max(0.0f, dist - (SHADOW_BIAS * 2.0f)), L.ignoreOccluderGroup)) continue;
                            float att = 1.f - dist / L.intensity;
                            att *= att;
                            const float contrib = emit * ndl * att;
                            cr += L.color.x * contrib;
                            cg += L.color.y * contrib;
                            cb += L.color.z * contrib;
                        }
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
                           const std::vector<PointLight>& lights)
{
    LightmapAtlas atlas;
    std::vector<BakePatch> patches = SubdivideLightmappedPolygons(polys);
    std::vector<FaceRect> rects = BuildFaceRects(patches, lights);

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

    OccluderSet occ = BuildOccluders(occluderPolys.empty() ? polys : occluderPolys);
    printf("[Lightmap] %zu source faces -> %zu bake patches across %zu pages of up to %dx%d, %zu lights, %zu tris, %zu luxels x %d samples = %zu rays/light\n",
           polys.size(), patches.size(), atlas.pages.size(), LIGHTMAP_PAGE_SIZE, LIGHTMAP_PAGE_SIZE,
           lights.size(), occ.tris.size(),
           totalLuxels, AA_GRID * AA_GRID, totalLuxels * (size_t)(AA_GRID * AA_GRID));
    fflush(stdout);

    for (uint32_t pageIndex = 0; pageIndex < atlas.pages.size(); ++pageIndex) {
        LightmapPage& page = atlas.pages[pageIndex];
        printf("[Lightmap] page %u/%zu begin (%dx%d)\n",
               pageIndex + 1, atlas.pages.size(), page.width, page.height);
        fflush(stdout);
        std::vector<uint8_t> coverage = BuildCoverageMask(rects, pageIndex, page.width, page.height);
        std::vector<uint8_t> valid = CoverageToValidMask(coverage);
        std::vector<PointLight> pageLights = GatherPageLights(rects, pageIndex, lights);
        size_t pageLuxels = 0;

        std::vector<LightmapComputeFaceRect> pageRects;
        pageRects.reserve(rects.size());
        bool pageRequiresCPU = false;
        for (const FaceRect& r : rects) {
            if (r.page == pageIndex) {
                pageRects.push_back(r.gpu);
                pageLuxels += (size_t)r.gpu.w * (size_t)r.gpu.h;
                if (r.gpu.polyCount < 0) {
                    pageRequiresCPU = true;
                }
            }
        }
        printf("[Lightmap] page %u stats: %zu rects, %zu lights, %zu luxels\n",
               pageIndex, pageRects.size(), pageLights.size(), pageLuxels);
        fflush(stdout);

        std::string computeError;
        bool usedCPUFallback = false;
        if (pageRequiresCPU) {
            computeError = "page contains polygon(s) exceeding compute vertex limit";
        }
        if (pageRequiresCPU || !BakeLightmapCompute(pageRects, occ.tris, pageLights, page.width, page.height, page.pixels, &computeError)) {
            printf("[Lightmap] page %u compute bake unavailable, falling back to CPU: %s\n",
                   pageIndex, computeError.c_str());
            fflush(stdout);
            BakeLightmapCPUPage(patches, lights, rects, pageIndex, occ, page);
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
                BakeLightmapCPUPage(patches, lights, rects, pageIndex, occ, page);
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
        printf("[Lightmap] page %u dilating borders\n", pageIndex);
        fflush(stdout);
        DilatePage(page, valid);
        printf("[Lightmap] page %u complete\n", pageIndex);
        fflush(stdout);
    }

    return atlas;
}
