#pragma once

#include "../utils/map_types.h"

#include <cstddef>
#include <vector>

struct CsgBrushPlane {
    Vector3 point;
    Vector3 normal;
};

struct CsgBrush {
    int sourceBrushId = -1;
    Vector3 min = {0.0f, 0.0f, 0.0f};
    Vector3 max = {0.0f, 0.0f, 0.0f};
    bool hasBounds = false;
    std::vector<CsgBrushPlane> planes;
    std::vector<MapPolygon> polygons;
};

enum class PolygonPlaneClass {
    Front,
    Back,
    OnPlane,
    Spanning
};

std::vector<CsgBrush> BuildCsgBrushes(const std::vector<MapPolygon>& polys);
bool BrushBoundsIntersect(const CsgBrush& a, const CsgBrush& b);
PolygonPlaneClass ClassifyPolygonAgainstPlane(const std::vector<Vector3>& poly, const CsgBrushPlane& plane);
bool PushValidFragment(const MapPolygon& source, std::vector<Vector3>&& verts, std::vector<MapPolygon>& out);
bool SamePolygonVerts(const std::vector<Vector3>& a, const std::vector<Vector3>& b);
std::vector<MapPolygon> ClipPolygonToBrushPlanes(const MapPolygon& poly, const CsgBrush& brush, size_t planeIndex, bool clipOnPlane);
std::vector<MapPolygon> ClipPolygonToBrush(const MapPolygon& poly, const CsgBrush& brush, bool clipOnPlane);
void ClipBrushToBrush(CsgBrush& brush, const CsgBrush& clipBrush, bool clipOnPlane);
std::vector<MapPolygon> BuildExteriorPolygons(const std::vector<MapPolygon>& polys);
