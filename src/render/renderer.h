// renderer.h  —  sokol_gfx map renderer (GL 3.3).
#pragma once

#include "sokol_gfx.h"
#include "../math/wmath.h"
#include "../utils/map_types.h"
#include <vector>
#include <string>
#include <unordered_map>

struct BSPData;      // forward (from bsp_loader.h)

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
    std::string activePackPath;
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
    uint32_t  lightmap_page = 0;
    int       index_count = 0;
    bool      fullbright = false;
    AABB      bounds{};           // world-space, for frustum culling
};

struct MapModel {
    std::vector<SubMesh> meshes;
    std::vector<sg_image> lightmapImages;
    std::vector<sg_view>  lightmapViews;
};

// ---------------------------------------------------------------------------
//  API
// ---------------------------------------------------------------------------
void      Renderer_Init(void);
void      Renderer_Shutdown(void);

MapModel  Renderer_UploadMap(const Map& map, TextureManager& texMgr);
MapModel  Renderer_UploadBSP(const BSPData& bsp, TextureManager& texMgr);
void      Renderer_DrawMap(const MapModel& mdl,
                           const Matrix&   mvp,
                           const Matrix&   model,
                           const Frustum&  frustum);
void      Renderer_DrawMapNormals(const MapModel& mdl,
                                  const Matrix&   mvp,
                                  const Matrix&   normalModel,
                                  const Frustum&  frustum);
void      Renderer_DestroyMap(MapModel& mdl);

sg_sampler Renderer_DefaultSampler(void);

bool      Renderer_BeginScenePostPass(const sg_pass_action& action,
                                      int width,
                                      int height,
                                      int sampleCount);
void      Renderer_EndScenePostPass(void);
bool      Renderer_BeginNormalPostPass(int width, int height);
void      Renderer_EndNormalPostPass(void);
void      Renderer_DrawPencilPostProcess(float timeSeconds);
bool      Renderer_PencilPostProcessReady(void);
