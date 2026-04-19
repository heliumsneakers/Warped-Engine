#pragma once

#include "../utils/map_types.h"

#include <cstddef>
#include <vector>

struct BrushSolidPlane {
    Vector3 point;
    Vector3 normal;
};

struct BrushSolid {
    int sourceBrushId = -1;
    Vector3 min = {0.0f, 0.0f, 0.0f};
    Vector3 max = {0.0f, 0.0f, 0.0f};
    bool hasBounds = false;
    std::vector<BrushSolidPlane> planes;
};

enum class PolygonPlaneClass {
    Front,
    Back,
    OnPlane,
    Spanning
};

std::vector<BrushSolid> BuildBrushSolids(const std::vector<MapPolygon>& polys);
bool BrushBoundsIntersect(const BrushSolid& a, const BrushSolid& b);
PolygonPlaneClass ClassifyPolygonAgainstPlane(const std::vector<Vector3>& poly, const BrushSolidPlane& plane);
bool PushValidFragment(const MapPolygon& source, std::vector<Vector3>&& verts, std::vector<MapPolygon>& out);
bool SamePolygonVerts(const std::vector<Vector3>& a, const std::vector<Vector3>& b);
std::vector<MapPolygon> ClipPolygonToBrushPlanes(const MapPolygon& poly, const BrushSolid& solid, size_t planeIndex, bool clipOnPlane);
std::vector<MapPolygon> ClipPolygonToBrush(const MapPolygon& poly, const BrushSolid& solid, bool clipOnPlane);
std::vector<MapPolygon> BuildExteriorPolygons(const std::vector<MapPolygon>& polys);
