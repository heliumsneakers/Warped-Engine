// bsp_format.h  —  on-disk layout for Warped .bsp files.
//
// Lump-table binary, little-endian, all POD.  Shared by the offline
// compile_map tool (writer) and the engine's bsp_loader (reader).
#pragma once
#include <cstdint>

#define WBSP_MAGIC    0x50534257u   // 'WBSP' little-endian
#define WBSP_VERSION  3u

enum {
    LUMP_TEXTURES = 0,   // BSPTexture[]
    LUMP_VERTICES,       // BSPVertex[]
    LUMP_INDICES,        // uint32_t[]
    LUMP_MESHES,         // BSPMesh[]
    LUMP_HULLS,          // BSPHull[]
    LUMP_HULL_PTS,       // BSPVec3[]
    LUMP_ENTITIES,       // char[]  (key/value text, \0-terminated)
    LUMP_LIGHTMAP,       // BSPLightmapLumpHeader + BSPLightmapPageHeader[] + rgba8 page data
    LUMP_BSP_TREE,       // BSPTreeHeader
    LUMP_BSP_PLANES,     // BSPPlane[]
    LUMP_BSP_FACES,      // BSPFace[]
    LUMP_BSP_FACE_VERTS, // BSPVec3[]
    LUMP_BSP_NODES,      // BSPNode[]
    LUMP_BSP_LEAVES,     // BSPLeaf[]
    LUMP_BSP_FACE_REFS,  // uint32_t[]
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
    uint32_t lightmapPage;
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

struct BSPLightmapLumpHeader {
    uint32_t pageCount;
};

struct BSPLightmapPageHeader {
    uint32_t width;
    uint32_t height;
    uint32_t byteLength;
};

struct BSPTreeHeader {
    int32_t  rootChild;   // node index >= 0, leaf encoded as (-1 - leafIndex)
    int32_t  outsideLeaf; // reserved for future outside-space classification
    uint32_t reserved0;
    uint32_t reserved1;
};

struct BSPPlane {
    float nx, ny, nz;
    float d;
};

struct BSPFace {
    uint32_t planeIndex;
    uint32_t textureIndex;
    uint32_t firstVertex;
    uint32_t vertexCount;
    int32_t  sourceEntityId;
    int32_t  sourceBrushId;
    int32_t  sourceFaceIndex;
    uint32_t flags;
};

struct BSPNode {
    int32_t  planeIndex;
    int32_t  frontChild;
    int32_t  backChild;
    float    minX, minY, minZ;
    float    maxX, maxY, maxZ;
    uint32_t firstFaceRef;
    uint32_t faceRefCount;
};

struct BSPLeaf {
    int32_t  contents;
    float    minX, minY, minZ;
    float    maxX, maxY, maxZ;
    uint32_t firstFaceRef;
    uint32_t faceRefCount;
};

#pragma pack(pop)
