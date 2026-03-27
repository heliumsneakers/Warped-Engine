// bsp_loader.h  —  runtime .bsp reader.
#pragma once
#include "map_parser.h"              // MapMeshBucket, Entity, PlayerStart
#include "../physx/collision_data.h" // MeshCollisionData
#include <vector>
#include <cstdint>

struct BSPData {
    std::vector<MapMeshBucket>     buckets;    // per-texture, ready for upload
    std::vector<MeshCollisionData> hulls;
    std::vector<Entity>            entities;   // point entities only
    std::vector<uint8_t>           lightmapPixels;   // RGBA8
    int lightmapW = 0, lightmapH = 0;
    std::string                    assetPackPath;
};

bool LoadBSP(const char* path, BSPData& out);

// Convenience: pull player starts from the loaded entity list.
std::vector<PlayerStart> GetPlayerStarts(const BSPData& bsp);
