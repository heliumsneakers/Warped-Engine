#pragma once

#include "../utils/map_types.h"

struct TextureManager;

std::vector<MapMeshBucket> BuildMapGeometry(const Map& map, TextureManager& textureManager);
