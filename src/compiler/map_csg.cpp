#include "map_csg.h"
#include "map_geometry.h"

#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <vector>

std::vector<CsgBrush> BuildCsgBrushes(const std::vector<MapPolygon>& polys) {
    std::vector<CsgBrush> brushes;
    brushes.reserve(polys.size());
    std::unordered_map<int, size_t> brushIndexById;
    for (const MapPolygon& poly : polys) {
        if (poly.sourceBrushId < 0 || poly.verts.empty()) {
            continue;
        }
        size_t brushIndex = 0;
        auto it = brushIndexById.find(poly.sourceBrushId);
        if (it == brushIndexById.end()) {
            brushIndex = brushes.size();
            brushIndexById.emplace(poly.sourceBrushId, brushIndex);
            brushes.emplace_back();
            brushes.back().sourceBrushId = poly.sourceBrushId;
        } else {
            brushIndex = it->second;
        }

        CsgBrush& brush = brushes[brushIndex];
        if (!brush.hasBounds) {
            brush.min = poly.verts[0];
            brush.max = poly.verts[0];
            brush.hasBounds = true;
        }
        for (const Vector3& vert : poly.verts) {
            brush.min.x = std::min(brush.min.x, vert.x);
            brush.min.y = std::min(brush.min.y, vert.y);
            brush.min.z = std::min(brush.min.z, vert.z);
            brush.max.x = std::max(brush.max.x, vert.x);
            brush.max.y = std::max(brush.max.y, vert.y);
            brush.max.z = std::max(brush.max.z, vert.z);
        }
        // Use an exact plane point derived from the original face-defining data
        // rather than poly.verts[0], which carries floating-point error from the
        // 3-plane intersection.  In RL world space n_RL·v_RL == -facePlaneD for
        // all vertices on this face, so the exact point on the plane is:
        //   exactPoint = n_RL * (-facePlaneD)
        const Vector3 exactNormal = Vector3Normalize(poly.normal);
        const Vector3 exactPoint  = Vector3Scale(exactNormal, -poly.facePlaneD);
        brush.planes.push_back({ exactPoint, exactNormal });
        brush.polygons.push_back(poly);
    }
    return brushes;
}

bool BrushBoundsIntersect(const CsgBrush& a, const CsgBrush& b) {
    if (!a.hasBounds || !b.hasBounds) {
        return true;
    }

    if (a.min.x > b.max.x || b.min.x > a.max.x) {
        return false;
    }
    if (a.min.y > b.max.y || b.min.y > a.max.y) {
        return false;
    }
    if (a.min.z > b.max.z || b.min.z > a.max.z) {
        return false;
    }
    return true;
}

PolygonPlaneClass ClassifyPolygonAgainstPlane(const std::vector<Vector3>& poly,
                                                     const CsgBrushPlane& plane) {
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
                                                        const CsgBrush& brush,
                                                        size_t planeIndex,
                                                        bool clipOnPlane) {
    if (poly.verts.size() < 3) {
        return {};
    }
    if (planeIndex >= brush.planes.size()) {
        return {};
    }

    const CsgBrushPlane& plane = brush.planes[planeIndex];
    switch (ClassifyPolygonAgainstPlane(poly.verts, plane)) {
        case PolygonPlaneClass::Front:
            return { poly };

        case PolygonPlaneClass::Back:
            return ClipPolygonToBrushPlanes(poly, brush, planeIndex + 1, clipOnPlane);

        case PolygonPlaneClass::OnPlane: {
            const float sameNormal = Vector3DotProduct(Vector3Normalize(poly.normal), plane.normal);
            const float angle = sameNormal - 1.0f;
            if (angle < CSG_NORMAL_EPS && angle > -CSG_NORMAL_EPS && !clipOnPlane) {
                return { poly };
            }
            return ClipPolygonToBrushPlanes(poly, brush, planeIndex + 1, clipOnPlane);
        }

        case PolygonPlaneClass::Spanning: {
            PolygonPlaneSplit split = SplitPolygonByPlane(poly.verts, plane.point, plane.normal);
            std::vector<MapPolygon> out;

            PushValidFragment(poly, std::move(split.front), out);

            std::vector<MapPolygon> backFragments;
            if (PushValidFragment(poly, std::move(split.back), backFragments)) {
                std::vector<MapPolygon> clippedBack =
                    ClipPolygonToBrushPlanes(backFragments[0], brush, planeIndex + 1, clipOnPlane);
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
                                                  const CsgBrush& brush,
                                                  bool clipOnPlane) {
    return ClipPolygonToBrushPlanes(poly, brush, 0, clipOnPlane);
}

void ClipBrushToBrush(CsgBrush& brush, const CsgBrush& clipBrush, bool clipOnPlane) {
    std::vector<MapPolygon> clippedPolys;
    for (const MapPolygon& poly : brush.polygons) {
        std::vector<MapPolygon> fragments = ClipPolygonToBrush(poly, clipBrush, clipOnPlane);
        clippedPolys.insert(clippedPolys.end(),
                            std::make_move_iterator(fragments.begin()),
                            std::make_move_iterator(fragments.end()));
    }
    brush.polygons.swap(clippedPolys);
}

std::vector<MapPolygon> BuildExteriorPolygons(const std::vector<MapPolygon>& polys) {
    if (polys.empty()) {
        return {};
    }

    const std::vector<CsgBrush> sourceBrushes = BuildCsgBrushes(polys);
    std::vector<CsgBrush> clippedBrushes = sourceBrushes;

    for (size_t i = 0; i < clippedBrushes.size(); ++i) {
        bool clipOnPlane = false;
        for (size_t j = 0; j < sourceBrushes.size(); ++j) {
            if (i == j) {
                clipOnPlane = true;
                continue;
            }
            if (clippedBrushes[i].polygons.empty()) {
                break;
            }
            if (BrushBoundsIntersect(clippedBrushes[i], sourceBrushes[j])) {
                ClipBrushToBrush(clippedBrushes[i], sourceBrushes[j], clipOnPlane);
            }
        }
    }

    std::vector<MapPolygon> exterior;
    for (CsgBrush& brush : clippedBrushes) {
        exterior.insert(exterior.end(),
                        std::make_move_iterator(brush.polygons.begin()),
                        std::make_move_iterator(brush.polygons.end()));
    }
    return exterior;
}
