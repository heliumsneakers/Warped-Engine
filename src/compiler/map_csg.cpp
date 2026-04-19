#include "map_csg.h"
#include "map_geometry.h"

#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <vector>

std::vector<BrushSolid> BuildBrushSolids(const std::vector<MapPolygon>& polys) {
    std::vector<BrushSolid> solids;
    solids.reserve(polys.size());
    std::unordered_map<int, size_t> solidIndexByBrushId;
    for (const MapPolygon& poly : polys) {
        if (poly.sourceBrushId < 0 || poly.verts.empty()) {
            continue;
        }
        size_t solidIndex = 0;
        auto it = solidIndexByBrushId.find(poly.sourceBrushId);
        if (it == solidIndexByBrushId.end()) {
            solidIndex = solids.size();
            solidIndexByBrushId.emplace(poly.sourceBrushId, solidIndex);
            solids.emplace_back();
            solids.back().sourceBrushId = poly.sourceBrushId;
        } else {
            solidIndex = it->second;
        }

        BrushSolid& solid = solids[solidIndex];
        if (!solid.hasBounds) {
            solid.min = poly.verts[0];
            solid.max = poly.verts[0];
            solid.hasBounds = true;
        }
        for (const Vector3& vert : poly.verts) {
            solid.min.x = std::min(solid.min.x, vert.x);
            solid.min.y = std::min(solid.min.y, vert.y);
            solid.min.z = std::min(solid.min.z, vert.z);
            solid.max.x = std::max(solid.max.x, vert.x);
            solid.max.y = std::max(solid.max.y, vert.y);
            solid.max.z = std::max(solid.max.z, vert.z);
        }
        // Use an exact plane point derived from the original face-defining data
        // rather than poly.verts[0], which carries floating-point error from the
        // 3-plane intersection.  In RL world space n_RL·v_RL == -facePlaneD for
        // all vertices on this face, so the exact point on the plane is:
        //   exactPoint = n_RL * (-facePlaneD)
        const Vector3 exactNormal = Vector3Normalize(poly.normal);
        const Vector3 exactPoint  = Vector3Scale(exactNormal, -poly.facePlaneD);
        solid.planes.push_back({ exactPoint, exactNormal });
    }
    return solids;
}

bool BrushBoundsIntersect(const BrushSolid& a, const BrushSolid& b) {
    if (!a.hasBounds || !b.hasBounds) {
        return true;
    }

    const float eps = CSG_POINT_EPS;
    if (a.min.x > b.max.x + eps || b.min.x > a.max.x + eps) {
        return false;
    }
    if (a.min.y > b.max.y + eps || b.min.y > a.max.y + eps) {
        return false;
    }
    if (a.min.z > b.max.z + eps || b.min.z > a.max.z + eps) {
        return false;
    }
    return true;
}

PolygonPlaneClass ClassifyPolygonAgainstPlane(const std::vector<Vector3>& poly,
                                                     const BrushSolidPlane& plane) {
    bool hasFront = false;
    bool hasBack = false;

    for (const Vector3& vert : poly) {
        const float dist = Vector3DotProduct(plane.normal, Vector3Subtract(vert, plane.point));
        if (dist > CSG_POLYGON_CLASSIFY_EPS) {
            hasFront = true;
        } else if (dist < -CSG_POLYGON_CLASSIFY_EPS) {
            hasBack = true;
        }

        if (hasFront && hasBack) {
            return PolygonPlaneClass::Spanning;
        }
    }

    if (hasFront) {
        return PolygonPlaneClass::Front;
    }
    if (hasBack) {
        return PolygonPlaneClass::Back;
    }
    return PolygonPlaneClass::OnPlane;
}

bool PushValidFragment(const MapPolygon& source,
                              std::vector<Vector3>&& verts,
                              std::vector<MapPolygon>& out) {
    CleanupClippedPolygon(verts, source.normal);
    if (verts.size() < 3 || PolygonArea3D(verts) <= 1e-6f) {
        return false;
    }

    MapPolygon fragment = source;
    fragment.verts = std::move(verts);
    out.push_back(std::move(fragment));
    return true;
}

bool SamePolygonVerts(const std::vector<Vector3>& a, const std::vector<Vector3>& b) {
    if (a.size() != b.size()) {
        return false;
    }

    const float epsSq = CSG_POLYGON_CLASSIFY_EPS * CSG_POLYGON_CLASSIFY_EPS;
    for (size_t i = 0; i < a.size(); ++i) {
        if (Vector3LengthSq(Vector3Subtract(a[i], b[i])) > epsSq) {
            return false;
        }
    }
    return true;
}

std::vector<MapPolygon> ClipPolygonToBrushPlanes(const MapPolygon& poly,
                                                        const BrushSolid& solid,
                                                        size_t planeIndex,
                                                        bool clipOnPlane) {
    if (poly.verts.size() < 3) {
        return {};
    }
    if (planeIndex >= solid.planes.size()) {
        return {};
    }

    const BrushSolidPlane& plane = solid.planes[planeIndex];
    switch (ClassifyPolygonAgainstPlane(poly.verts, plane)) {
        case PolygonPlaneClass::Front:
            return { poly };

        case PolygonPlaneClass::Back:
            return ClipPolygonToBrushPlanes(poly, solid, planeIndex + 1, clipOnPlane);

        case PolygonPlaneClass::OnPlane: {
            const float sameNormal = Vector3DotProduct(Vector3Normalize(poly.normal), plane.normal);
            const float angle = sameNormal - 1.0f;
            if (angle < CSG_NORMAL_EPS && angle > -CSG_NORMAL_EPS && !clipOnPlane) {
                return { poly };
            }
            return ClipPolygonToBrushPlanes(poly, solid, planeIndex + 1, clipOnPlane);
        }

        case PolygonPlaneClass::Spanning: {
            PolygonPlaneSplit split = SplitPolygonByPlane(poly.verts, plane.point, plane.normal);
            std::vector<MapPolygon> out;

            PushValidFragment(poly, std::move(split.front), out);

            std::vector<MapPolygon> backFragments;
            if (PushValidFragment(poly, std::move(split.back), backFragments)) {
                std::vector<MapPolygon> clippedBack =
                    ClipPolygonToBrushPlanes(backFragments[0], solid, planeIndex + 1, clipOnPlane);
                if (clippedBack.size() == 1 && SamePolygonVerts(clippedBack[0].verts, backFragments[0].verts)) {
                    return { poly };
                }
                out.insert(out.end(),
                           std::make_move_iterator(clippedBack.begin()),
                           std::make_move_iterator(clippedBack.end()));
            }

            return out;
        }
    }

    return { poly };
}

std::vector<MapPolygon> ClipPolygonToBrush(const MapPolygon& poly,
                                                  const BrushSolid& solid,
                                                  bool clipOnPlane) {
    return ClipPolygonToBrushPlanes(poly, solid, 0, clipOnPlane);
}

std::vector<MapPolygon> BuildExteriorPolygons(const std::vector<MapPolygon>& polys) {
    if (polys.empty()) {
        return {};
    }

    const std::vector<BrushSolid> solids = BuildBrushSolids(polys);
    std::unordered_map<int, const BrushSolid*> solidByBrushId;
    solidByBrushId.reserve(solids.size());
    for (const BrushSolid& solid : solids) {
        solidByBrushId[solid.sourceBrushId] = &solid;
    }

    std::vector<MapPolygon> exterior;
    for (const MapPolygon& poly : polys) {
        const BrushSolid* sourceSolid = nullptr;
        auto sourceSolidIt = solidByBrushId.find(poly.sourceBrushId);
        if (sourceSolidIt != solidByBrushId.end()) {
            sourceSolid = sourceSolidIt->second;
        }

        std::vector<MapPolygon> fragments;
        fragments.push_back(poly);

        for (const BrushSolid& solid : solids) {
            if (solid.sourceBrushId == poly.sourceBrushId) {
                continue;
            }
            if (sourceSolid && !BrushBoundsIntersect(*sourceSolid, solid)) {
                continue;
            }

            std::vector<MapPolygon> nextFragments;
            const bool clipOnPlane = solid.sourceBrushId > poly.sourceBrushId;
            for (const MapPolygon& fragment : fragments) {
                std::vector<MapPolygon> kept = ClipPolygonToBrush(fragment, solid, clipOnPlane);
                nextFragments.insert(nextFragments.end(),
                                     std::make_move_iterator(kept.begin()),
                                     std::make_move_iterator(kept.end()));
            }
            fragments.swap(nextFragments);
            if (fragments.empty()) {
                break;
            }
        }

        exterior.insert(exterior.end(),
                        std::make_move_iterator(fragments.begin()),
                        std::make_move_iterator(fragments.end()));
    }
    return exterior;
}
