// bsp_loader.h  —  runtime .bsp reader.
#pragma once
#include "bsp_format.h"
#include "map_types.h"               // MapMeshBucket, Entity, PlayerStart
#include "../physx/collision_data.h" // MeshCollisionData
#include <vector>
#include <cstdint>

struct BSPDataLightmapPage {
    int                  width = 0;
    int                  height = 0;
    uint32_t             format = BSP_LIGHTMAP_FORMAT_RGBA8_UNORM;
    std::vector<uint8_t> pixels;
};

struct BSPData {
    std::vector<MapMeshBucket>     buckets;    // per-texture, ready for upload
    std::vector<MeshCollisionData> hulls;
    std::vector<Entity>            entities;   // point and brush entities, properties only
    std::vector<BSPDataLightmapPage> lightmapPages;
    BSPTreeHeader                  tree{};
    std::vector<BSPPlane>          planes;
    std::vector<BSPFace>           bspFaces;
    std::vector<BSPVec3>           bspFaceVerts;
    std::vector<BSPNode>           bspNodes;
    std::vector<BSPLeaf>           bspLeaves;
    std::vector<uint32_t>          bspFaceRefs;
    std::string                    assetPackPath;
};

bool LoadBSP(const char* path, BSPData& out);

// Convenience: pull player starts from the loaded entity list.
std::vector<PlayerStart> GetPlayerStarts(const BSPData& bsp);
