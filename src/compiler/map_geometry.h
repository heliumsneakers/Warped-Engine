#pragma once

#include "../utils/map_types.h"

#include <vector>

inline constexpr float CSG_POLYGON_CLASSIFY_EPS = 1e-2f;
inline constexpr float CSG_POINT_EPS = 1e-2f;
inline constexpr float CSG_NORMAL_EPS = 1e-5f;
inline constexpr float CSG_COLLINEAR_EPS = 1e-3f;

struct PolygonPlaneSplit {
    std::vector<Vector3> front;
    std::vector<Vector3> back;
    bool hasStrictFront = false;
};

Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3);
void RemoveDuplicatePoints(std::vector<Vector3>& points, float eps);
void SortPolygonVertices(std::vector<Vector3>& poly, const Vector3& normal);
bool GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& out);
Vector3 ConvertTBtoRaylib(const Vector3& in);
Vector3 ConvertTBPointEntityToWorld(const Vector3& in);
void CleanupClippedPolygon(std::vector<Vector3>& poly, const Vector3& expectedNormal);
float PolygonArea3D(const std::vector<Vector3>& poly);
PolygonPlaneSplit SplitPolygonByPlane(const std::vector<Vector3>& poly,
                                      const Vector3& planePoint,
                                      const Vector3& planeNormal);
Vector3 PolygonCentroid(const std::vector<Vector3>& verts);
void FaceBasis(const Vector3& n, Vector3& u, Vector3& v);
bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py);
Vector2 ComputeFaceUV(const Vector3& vert,
                      const Vector3& axisU, const Vector3& axisV,
                      float offU, float offV, float rot,
                      float scaleU, float scaleV,
                      float texW, float texH);
