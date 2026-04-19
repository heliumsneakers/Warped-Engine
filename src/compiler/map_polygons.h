#pragma once

#include "../utils/map_types.h"

#include <vector>

void AppendBrushEntityPolygons(const Entity& entity,
                               int sourceEntityId,
                               bool devMode,
                               bool exteriorOnly,
                               int* nextLightBrushGroup,
                               int* nextSourceBrushId,
                               size_t* sourceFaceCount,
                               std::vector<MapPolygon>& out);
std::vector<MapPolygon> BuildMapPolygons(const Map& map, bool devMode);
std::vector<MapPolygon> BuildExteriorMapPolygons(const Map& map, bool devMode);
