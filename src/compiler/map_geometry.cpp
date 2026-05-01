#include "map_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>

// Calculate normal vector from 3 points
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
    Vector3 edge1 = Vector3Subtract(v2, v1);
    Vector3 edge2 = Vector3Subtract(v3, v1);
    Vector3 n     = Vector3CrossProduct(edge1, edge2);
    return Vector3Normalize(n);
}

// Remove duplicates in-place
void RemoveDuplicatePoints(std::vector<Vector3>& points, float eps) {
    std::vector<Vector3> unique;
    unique.reserve(points.size());

    for (auto &p : points) {
        bool found = false;
        for (auto &u : unique) {
            float dx = p.x - u.x;
            float dy = p.y - u.y;
            float dz = p.z - u.z;
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < eps*eps) {
                found = true;
                break;
            }
        }
        if (!found) unique.push_back(p);
    }
    points = std::move(unique);
}

// Sort polygon vertices by angle around centroid
void SortPolygonVertices(std::vector<Vector3>& poly, const Vector3& normal) {
    if (poly.size() < 3) return;

    // Compute centroid
    Vector3 centroid = {0,0,0};
    for (auto &p : poly) centroid = Vector3Add(centroid, p);
    centroid = Vector3Scale(centroid, 1.0f / (float)poly.size());

    // Create local axes in plane
    Vector3 arbitrary = {1,0,0};
    if (fabsf(Vector3DotProduct(normal, arbitrary)) > 0.9f) {
        arbitrary = (Vector3){0,1,0};
    }

    Vector3 U = Vector3Normalize(Vector3CrossProduct(normal, arbitrary));
    Vector3 V = Vector3CrossProduct(normal, U);

    struct AnglePoint {
        float   angle;
        Vector3 point;
    };

    std::vector<AnglePoint> ap;
    ap.reserve(poly.size());
    for (auto &p : poly) {
        Vector3 dir = Vector3Subtract(p, centroid);
        float u     = Vector3DotProduct(dir, U);
        float v     = Vector3DotProduct(dir, V);
        float angle = atan2f(v, u);
        ap.push_back({angle, p});
    }

    std::sort(ap.begin(), ap.end(), [](const AnglePoint &a, const AnglePoint &b){
        return a.angle < b.angle;
    });

    for (size_t i=0; i<ap.size(); i++) {
        poly[i] = ap[i].point;
    }
}

void CleanupClippedPolygon(std::vector<Vector3>& poly, const Vector3& expectedNormal) {
    RemoveDuplicatePoints(poly, CSG_POLYGON_CLASSIFY_EPS);
    if (poly.size() < 3) {
        poly.clear();
        return;
    }

    std::vector<Vector3> cleaned;
    cleaned.reserve(poly.size());
    const float collinearEpsSq = CSG_COLLINEAR_EPS * CSG_COLLINEAR_EPS;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& prev = poly[(i + poly.size() - 1) % poly.size()];
        const Vector3& curr = poly[i];
        const Vector3& next = poly[(i + 1) % poly.size()];
        const Vector3 e0 = Vector3Subtract(curr, prev);
        const Vector3 e1 = Vector3Subtract(next, curr);
        if (Vector3LengthSq(e0) <= collinearEpsSq || Vector3LengthSq(e1) <= collinearEpsSq) {
            continue;
        }
        const Vector3 cross = Vector3CrossProduct(e0, e1);
        if (Vector3LengthSq(cross) <= collinearEpsSq) {
            continue;
        }
        cleaned.push_back(curr);
    }

    RemoveDuplicatePoints(cleaned, CSG_POLYGON_CLASSIFY_EPS);
    if (cleaned.size() < 3) {
        poly.clear();
        return;
    }

    Vector3 clippedNormal = CalculateNormal(cleaned[0], cleaned[1], cleaned[2]);
    if (Vector3DotProduct(clippedNormal, expectedNormal) < 0.0f) {
        std::reverse(cleaned.begin(), cleaned.end());
    }
    poly.swap(cleaned);
}

namespace {

bool PointsNearlyEqual(const Vector3& a, const Vector3& b, float epsSq) {
    return Vector3LengthSq(Vector3Subtract(a, b)) <= epsSq;
}

bool TriangleIsUsable(const std::vector<Vector3>& poly,
                      uint32_t ia,
                      uint32_t ib,
                      uint32_t ic,
                      const Vector3& normal,
                      float epsSq) {
    const Vector3 ab = Vector3Subtract(poly[ib], poly[ia]);
    const Vector3 bc = Vector3Subtract(poly[ic], poly[ib]);
    const Vector3 cross = Vector3CrossProduct(ab, bc);
    if (Vector3LengthSq(cross) <= epsSq) {
        return false;
    }
    return Vector3DotProduct(cross, normal) > 0.0f;
}

bool PointInTriangle3D(const Vector3& p,
                       const Vector3& a,
                       const Vector3& b,
                       const Vector3& c,
                       float eps)
{
    const Vector3 v0 = Vector3Subtract(c, a);
    const Vector3 v1 = Vector3Subtract(b, a);
    const Vector3 v2 = Vector3Subtract(p, a);
    const float dot00 = Vector3DotProduct(v0, v0);
    const float dot01 = Vector3DotProduct(v0, v1);
    const float dot02 = Vector3DotProduct(v0, v2);
    const float dot11 = Vector3DotProduct(v1, v1);
    const float dot12 = Vector3DotProduct(v1, v2);
    const float denom = dot00 * dot11 - dot01 * dot01;
    if (fabsf(denom) <= eps * eps) {
        return false;
    }

    const float invDenom = 1.0f / denom;
    const float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    const float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    return u >= -eps && v >= -eps && (u + v) <= 1.0f + eps;
}

bool TriangleContainsAnyOtherVertex(const std::vector<Vector3>& poly,
                                    const std::vector<uint32_t>& remaining,
                                    uint32_t ia,
                                    uint32_t ib,
                                    uint32_t ic,
                                    float eps)
{
    const Vector3& a = poly[ia];
    const Vector3& b = poly[ib];
    const Vector3& c = poly[ic];
    const float epsSq = eps * eps;
    for (uint32_t index : remaining) {
        if (index == ia || index == ib || index == ic) {
            continue;
        }
        if (PointsNearlyEqual(poly[index], a, epsSq) ||
            PointsNearlyEqual(poly[index], b, epsSq) ||
            PointsNearlyEqual(poly[index], c, epsSq)) {
            continue;
        }
        if (PointInTriangle3D(poly[index], a, b, c, eps)) {
            return true;
        }
    }
    return false;
}

float DistSqPointSegment2D(const Vector2& p, const Vector2& a, const Vector2& b) {
    const float abx = b.x - a.x;
    const float aby = b.y - a.y;
    const float abLenSq = abx * abx + aby * aby;
    if (abLenSq <= 1e-8f) {
        const float dx = p.x - a.x;
        const float dy = p.y - a.y;
        return dx * dx + dy * dy;
    }

    const float t = std::clamp(((p.x - a.x) * abx + (p.y - a.y) * aby) / abLenSq, 0.0f, 1.0f);
    const float qx = a.x + abx * t;
    const float qy = a.y + aby * t;
    const float dx = p.x - qx;
    const float dy = p.y - qy;
    return dx * dx + dy * dy;
}

} // namespace

void HealTJunctions(std::vector<MapPolygon>& polys, float eps) {
    if (polys.size() < 2) {
        return;
    }

    const float epsSq = eps * eps;
    std::vector<std::vector<Vector3>> sourceVerts;
    sourceVerts.reserve(polys.size());
    for (const MapPolygon& poly : polys) {
        sourceVerts.push_back(poly.verts);
    }

    for (size_t polyIndex = 0; polyIndex < polys.size(); ++polyIndex) {
        MapPolygon& poly = polys[polyIndex];
        if (poly.verts.size() < 2) {
            continue;
        }

        std::vector<Vector3> healed;
        healed.reserve(poly.verts.size());

        for (size_t i = 0; i < poly.verts.size(); ++i) {
            const Vector3& v0 = poly.verts[i];
            const Vector3& v1 = poly.verts[(i + 1) % poly.verts.size()];
            const Vector3 edge = Vector3Subtract(v1, v0);
            const float edgeLenSq = Vector3LengthSq(edge);

            healed.push_back(v0);
            if (edgeLenSq <= epsSq) {
                continue;
            }

            std::vector<std::pair<float, Vector3>> onEdge;
            for (size_t otherIndex = 0; otherIndex < polys.size(); ++otherIndex) {
                if (otherIndex == polyIndex) {
                    continue;
                }
                for (const Vector3& p : sourceVerts[otherIndex]) {
                    if (PointsNearlyEqual(p, v0, epsSq) || PointsNearlyEqual(p, v1, epsSq)) {
                        continue;
                    }

                    const float t = Vector3DotProduct(Vector3Subtract(p, v0), edge) / edgeLenSq;
                    if (t <= 0.0f || t >= 1.0f) {
                        continue;
                    }

                    const Vector3 projected = Vector3Add(v0, Vector3Scale(edge, t));
                    if (Vector3LengthSq(Vector3Subtract(p, projected)) > epsSq) {
                        continue;
                    }

                    bool duplicate = false;
                    for (const auto& existing : onEdge) {
                        if (fabsf(existing.first - t) <= eps ||
                            PointsNearlyEqual(existing.second, p, epsSq)) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate) {
                        onEdge.push_back({ t, p });
                    }
                }
            }

            std::sort(onEdge.begin(), onEdge.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            for (const auto& inserted : onEdge) {
                healed.push_back(inserted.second);
            }
        }

        poly.verts.swap(healed);
    }
}

float PolygonArea3D(const std::vector<Vector3>& poly) {
    if (poly.size() < 3) {
        return 0.0f;
    }

    const Vector3 origin = poly[0];
    float area = 0.0f;
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        const Vector3 a = Vector3Subtract(poly[i], origin);
        const Vector3 b = Vector3Subtract(poly[i + 1], origin);
        area += 0.5f * Vector3Length(Vector3CrossProduct(a, b));
    }
    return area;
}

std::vector<uint32_t> TriangulatePolygonIndices(const std::vector<Vector3>& poly,
                                                const Vector3& normal,
                                                float eps) {
    std::vector<uint32_t> indices;
    if (poly.size() < 3) {
        return indices;
    }

    const float epsSq = eps * eps;
    std::vector<uint32_t> remaining(poly.size());
    std::iota(remaining.begin(), remaining.end(), 0u);
    indices.reserve((poly.size() - 2) * 3);

    size_t guard = poly.size() * poly.size();
    while (remaining.size() > 3 && guard-- > 0) {
        bool clipped = false;
        for (size_t i = 0; i < remaining.size(); ++i) {
            const uint32_t prev = remaining[(i + remaining.size() - 1) % remaining.size()];
            const uint32_t curr = remaining[i];
            const uint32_t next = remaining[(i + 1) % remaining.size()];
            if (!TriangleIsUsable(poly, prev, curr, next, normal, epsSq)) {
                continue;
            }
            if (TriangleContainsAnyOtherVertex(poly, remaining, prev, curr, next, eps)) {
                continue;
            }

            indices.push_back(prev);
            indices.push_back(curr);
            indices.push_back(next);
            remaining.erase(remaining.begin() + (std::ptrdiff_t)i);
            clipped = true;
            break;
        }

        if (!clipped) {
            indices.clear();
            for (size_t t = 1; t + 1 < poly.size(); ++t) {
                if (TriangleIsUsable(poly, 0u, (uint32_t)t, (uint32_t)t + 1u, normal, epsSq)) {
                    indices.push_back(0u);
                    indices.push_back((uint32_t)t);
                    indices.push_back((uint32_t)t + 1u);
                }
            }
            return indices;
        }
    }

    if (remaining.size() == 3 &&
        TriangleIsUsable(poly, remaining[0], remaining[1], remaining[2], normal, epsSq)) {
        indices.push_back(remaining[0]);
        indices.push_back(remaining[1]);
        indices.push_back(remaining[2]);
    }

    return indices;
}

PolygonPlaneSplit SplitPolygonByPlane(const std::vector<Vector3>& poly,
                                             const Vector3& planePoint,
                                             const Vector3& planeNormal) {
    PolygonPlaneSplit split;
    if (poly.size() < 3) {
        return split;
    }

    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& a = poly[i];
        const Vector3& b = poly[(i + 1) % poly.size()];
        const float da = Vector3DotProduct(planeNormal, Vector3Subtract(a, planePoint));
        const float db = Vector3DotProduct(planeNormal, Vector3Subtract(b, planePoint));
        const bool aFront = da > CSG_POINT_EPS;
        const bool aBack = da < -CSG_POINT_EPS;

        if (aFront) {
            split.hasStrictFront = true;
        }
        if (!aBack) {
            split.front.push_back(a);
        }
        if (!aFront) {
            split.back.push_back(a);
        }

        if ((da > CSG_POINT_EPS && db < -CSG_POINT_EPS) ||
            (da < -CSG_POINT_EPS && db > CSG_POINT_EPS)) {
            const float t = da / (da - db);
            const Vector3 hit = Vector3Add(a, Vector3Scale(Vector3Subtract(b, a), t));
            split.front.push_back(hit);
            split.back.push_back(hit);
            split.hasStrictFront = true;
        }
    }

    return split;
}

// Simple intersection of three planes (in TB coords)
// **Credit to Stefan Hajnoczi**
bool GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& out) {
    Vector3 cross23 = Vector3CrossProduct(p2.normal, p3.normal);
    float denom     = Vector3DotProduct(p1.normal, cross23);

    if (fabsf(denom) < (float)epsilon) {
        return false;
    }

    Vector3 term1 = Vector3Scale(cross23, -p1.d);
    Vector3 cross31 = Vector3CrossProduct(p3.normal, p1.normal);
    Vector3 term2 = Vector3Scale(cross31, -p2.d);
    Vector3 cross12 = Vector3CrossProduct(p1.normal, p2.normal);
    Vector3 term3 = Vector3Scale(cross12, -p3.d);

    Vector3 numerator = Vector3Add(Vector3Add(term1, term2), term3);
    out = Vector3Scale(numerator, 1.0f / denom);

    return true;
}

// Convert TrenchBroom coordinates (X right, Y forward, Z up) to engine world
// coordinates (X right, Y up, Z backward/forward axis negated).
Vector3 ConvertTBtoWorld(const Vector3& in) {
    Vector3 out = {
         in.x,
         in.z,
        -in.y
    };
    return out;
}

Vector3 ConvertTBTextureAxisToWorld(const Vector3& in) {
    Vector3 out = {
        -in.x,
        -in.z,
         in.y
    };
    return out;
}

void ConvertMapPolygonTBToWorld(MapPolygon& poly) {
    for (Vector3& vert : poly.verts) {
        vert = ConvertTBtoWorld(vert);
    }

    poly.normal = Vector3Normalize(ConvertTBtoWorld(poly.normal));
}

void ConvertMapPolygonsTBToWorld(std::vector<MapPolygon>& polys) {
    for (MapPolygon& poly : polys) {
        ConvertMapPolygonTBToWorld(poly);
    }
}

// Convert coordinates for point entities authored directly in TrenchBroom.
Vector3 ConvertTBPointEntityToWorld(const Vector3& in) {
    return ConvertTBtoWorld(in);
}

Vector3 PolygonCentroid(const std::vector<Vector3>& verts) {
    Vector3 centroid{0, 0, 0};
    if (verts.empty()) {
        return centroid;
    }
    for (const Vector3& v : verts) {
        centroid = Vector3Add(centroid, v);
    }
    return Vector3Scale(centroid, 1.0f / (float)verts.size());
}

void FaceBasis(const Vector3& n, Vector3& u, Vector3& v) {
    Vector3 ref = (fabsf(n.y) < 0.9f) ? (Vector3){0, 1, 0} : (Vector3){1, 0, 0};
    u = Vector3Normalize(Vector3CrossProduct(n, ref));
    v = Vector3CrossProduct(n, u);
}

bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py) {
    const size_t n = poly.size();
    if (n < 3) {
        return false;
    }

    constexpr float kEdgeEpsilon = 1e-4f;
    const Vector2 p{ px, py };
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        if (DistSqPointSegment2D(p, poly[i], poly[j]) <= kEdgeEpsilon * kEdgeEpsilon) {
            return true;
        }
    }

    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vector2& a = poly[i];
        const Vector2& b = poly[j];
        if ((a.y > py) == (b.y > py)) {
            continue;
        }
        const float xIntersect = (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x;
        if (px < xIntersect) {
            inside = !inside;
        }
    }
    return inside;
}

Vector2 ComputeFaceUV(const Vector3& vert,
                      const Vector3& axisU, const Vector3& axisV,
                      float offU, float offV, float rot,
                      float scaleU, float scaleV,
                      float texW, float texH)
{
    float sx = Vector3DotProduct(vert, axisU) / scaleU;
    float sy = Vector3DotProduct(vert, axisV) / scaleV;

    float rad = rot * DEG2RAD;
    float cr = cosf(rad), sr = sinf(rad);
    float sxR = sx*cr - sy*sr;
    float syR = sx*sr + sy*cr;

    // Negated for axis conversion.
    sxR += -offU;
    syR += -offV;

    if (texW > 0.f) sxR /= texW;
    if (texH > 0.f) syR /= texH;

    // Orientation flip for axis conversion.
    return (Vector2){ 1.0f - sxR, 1.0f - syR };
}
