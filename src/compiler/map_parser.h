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
bool ParsePointEntityFacing(const Entity& entity, float& outYaw, float& outPitch);
