// renderer.h  —  sokol_gfx map renderer (GL 3.3).
#pragma once

#include "sokol_gfx.h"
#include "../math/wmath.h"
#include <vector>
#include <string>
#include <unordered_map>

struct Map;          // forward (from map_parser.h)

// ---------------------------------------------------------------------------
//  Texture manager  (stb_image → sg_image + texture view)
// ---------------------------------------------------------------------------
struct TextureEntry {
    sg_image  image;
    sg_view   view;
    int       width  = 0;
    int       height = 0;
};

struct TextureManager {
    std::unordered_map<std::string, TextureEntry> textures;
};

void                InitTextureManager(TextureManager& mgr);
const TextureEntry* LoadTextureByName(TextureManager& mgr, const std::string& name);
void                UnloadAllTextures(TextureManager& mgr);

// ---------------------------------------------------------------------------
//  GPU-resident map model (one submesh per texture)
// ---------------------------------------------------------------------------
struct SubMesh {
    sg_buffer vbuf{};
    sg_buffer ibuf{};
    sg_view   tex_view{};
    int       index_count = 0;
    AABB      bounds{};           // world-space, for frustum culling
};

struct MapModel {
    std::vector<SubMesh> meshes;
};

// ---------------------------------------------------------------------------
//  API
// ---------------------------------------------------------------------------
void      Renderer_Init(void);
void      Renderer_Shutdown(void);

MapModel  Renderer_UploadMap(const Map& map, TextureManager& texMgr);
void      Renderer_DrawMap(const MapModel& mdl,
                           const Matrix&   mvp,
                           const Matrix&   model,
                           const Frustum&  frustum);
void      Renderer_DestroyMap(MapModel& mdl);

sg_sampler Renderer_DefaultSampler(void);
