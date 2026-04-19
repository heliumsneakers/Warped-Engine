#pragma once

#include "../utils/map_types.h"

#include <string>
#include <vector>

struct TextureManager;

struct MapParseResult {
    Map map;
    bool ok = false;
    std::string error;
};

MapParseResult ParseMapFile(const std::string& filePath);
std::vector<PlayerStart> GetPlayerStarts(const Map& map);
std::vector<PointLight> GetPointLights(const Map& map);
std::vector<SurfaceLightTemplate> GetSurfaceLightTemplates(const Map& map);
LightBakeSettings GetLightBakeSettings(const Map& map);
std::vector<MapPolygon> BuildMapPolygons(const Map& map, bool devMode);
std::vector<MapPolygon> BuildExteriorMapPolygons(const Map& map, bool devMode);
std::vector<MapMeshBucket> BuildMapGeometry(const Map& map, TextureManager& textureManager);
Vector2 ComputeFaceUV(const Vector3& vert,
                      const Vector3& axisU, const Vector3& axisV,
                      float offU, float offV, float rot,
                      float scaleU, float scaleV,
                      float texW, float texH);
Vector3 ConvertTBtoRaylib(const Vector3& in);
Vector3 ConvertTBPointEntityToWorld(const Vector3& in);
bool ParsePointEntityFacing(const Entity& entity, float& outYaw, float& outPitch);
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3);
void RemoveDuplicatePoints(std::vector<Vector3>& points, float eps);
bool GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& out);
void SortPolygonVertices(std::vector<Vector3>& poly, const Vector3& normal);
