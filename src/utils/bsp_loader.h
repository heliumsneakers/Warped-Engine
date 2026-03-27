// bsp_loader.h  —  runtime .bsp reader.
#pragma once
#include "map_parser.h"              // MapMeshBucket, Entity, PlayerStart
#include "../physx/collision_data.h" // MeshCollisionData
#include <vector>
#include <cstdint>

struct BSPDataLightmapPage {
    int                  width = 0;
    int                  height = 0;
    std::vector<uint8_t> pixels;
};

struct BSPData {
    std::vector<MapMeshBucket>     buckets;    // per-texture, ready for upload
    std::vector<MeshCollisionData> hulls;
    std::vector<Entity>            entities;   // point entities only
    std::vector<BSPDataLightmapPage> lightmapPages;
    std::string                    assetPackPath;
};

bool LoadBSP(const char* path, BSPData& out);

// Convenience: pull player starts from the loaded entity list.
std::vector<PlayerStart> GetPlayerStarts(const BSPData& bsp);
