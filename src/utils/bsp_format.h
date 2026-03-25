// bsp_format.h  —  on-disk layout for Warped .bsp files.
//
// Lump-table binary, little-endian, all POD.  Shared by the offline
// compile_map tool (writer) and the engine's bsp_loader (reader).
#pragma once
#include <cstdint>

#define WBSP_MAGIC    0x50534257u   // 'WBSP' little-endian
#define WBSP_VERSION  1u

enum {
    LUMP_TEXTURES = 0,   // BSPTexture[]
    LUMP_VERTICES,       // BSPVertex[]
    LUMP_INDICES,        // uint32_t[]
    LUMP_MESHES,         // BSPMesh[]
    LUMP_HULLS,          // BSPHull[]
    LUMP_HULL_PTS,       // BSPVec3[]
    LUMP_ENTITIES,       // char[]  (key/value text, \0-terminated)
    LUMP_LIGHTMAP,       // BSPLightmapHeader + rgba8 pixels
    LUMP_COUNT
};

#pragma pack(push, 1)

struct BSPLump {
    uint32_t offset;
    uint32_t length;
};

struct BSPHeader {
    uint32_t magic;
    uint32_t version;
    BSPLump  lumps[LUMP_COUNT];
};

struct BSPTexture {
    char     name[64];
    uint32_t width;
    uint32_t height;
};

struct BSPVertex {
    float x, y, z;        // world-space position (GL coords, Y-up)
    float nx, ny, nz;     // normal
    float u, v;           // diffuse UV (normalised)
    float lu, lv;         // lightmap UV (normalised into atlas)
};

struct BSPMesh {
    uint32_t textureIndex;
    uint32_t firstIndex;
    uint32_t indexCount;
    uint32_t firstVertex;
    uint32_t vertexCount;
};

struct BSPVec3 {
    float x, y, z;
};

struct BSPHull {
    uint32_t firstPoint;
    uint32_t pointCount;
    uint32_t collisionType;   // maps to enum CollisionType
};

struct BSPLightmapHeader {
    uint32_t width;
    uint32_t height;
    // followed by width*height*4 bytes of RGBA8
};

#pragma pack(pop)
