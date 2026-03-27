// renderer.cpp
//
// All GPU-facing code lives here.  The brush geometry itself is still produced
// by map_parser.cpp via BuildMapGeometry(); we just take the resulting CPU
// arrays, upload them to sokol_gfx buffers and render them with a single
// textured-diffuse pipeline.

#include "renderer.h"
#include "shaders.h"
#include "../utils/asset_pack.h"

#if defined(WARPED_SOKOL_BACKEND_METAL) && !defined(SOKOL_METAL)
#define SOKOL_METAL
#elif defined(WARPED_SOKOL_BACKEND_D3D11) && !defined(SOKOL_D3D11)
#define SOKOL_D3D11
#elif defined(WARPED_SOKOL_BACKEND_GLCORE) && !defined(SOKOL_GLCORE)
#define SOKOL_GLCORE
#endif

#include "shaders/generated/map.metal_dx11.h"
#include "../utils/map_parser.h"
#include "../utils/bsp_loader.h"
#include "sokol_gfx.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <cstddef>
#include <cstdio>

// ---------------------------------------------------------------------------
//  Internal state
// ---------------------------------------------------------------------------
static sg_shader   g_shader    = {};
static sg_pipeline g_pipeline  = {};
static sg_sampler  g_sampler   = {};   // repeat, for diffuse
static sg_sampler  g_lmSampler = {};   // clamp, for lightmap
static sg_image    g_whiteLm   = {};   // 1×1 white fallback lightmap
static sg_view     g_whiteLmV  = {};
static bool        g_logged_bind_diagnostics = false;

static const char* RendererBackendName(sg_backend backend) {
    switch (backend) {
        case SG_BACKEND_GLCORE:          return "GLCORE";
        case SG_BACKEND_GLES3:           return "GLES3";
        case SG_BACKEND_D3D11:           return "D3D11";
        case SG_BACKEND_METAL_IOS:       return "METAL_IOS";
        case SG_BACKEND_METAL_MACOS:     return "METAL_MACOS";
        case SG_BACKEND_METAL_SIMULATOR: return "METAL_SIMULATOR";
        case SG_BACKEND_WGPU:            return "WGPU";
        case SG_BACKEND_VULKAN:          return "VULKAN";
        case SG_BACKEND_DUMMY:           return "DUMMY";
        default:                         return "UNKNOWN";
    }
}

static const char* RendererResourceStateName(sg_resource_state state) {
    switch (state) {
        case SG_RESOURCESTATE_INITIAL: return "INITIAL";
        case SG_RESOURCESTATE_ALLOC:   return "ALLOC";
        case SG_RESOURCESTATE_VALID:   return "VALID";
        case SG_RESOURCESTATE_FAILED:  return "FAILED";
        case SG_RESOURCESTATE_INVALID: return "INVALID";
        default:                       return "UNKNOWN";
    }
}

static void Renderer_LogTextureState(const char* label, const TextureEntry& entry) {
    printf("[Renderer] %s image=%s view=%s size=%dx%d\n",
           label,
           RendererResourceStateName(sg_query_image_state(entry.image)),
           RendererResourceStateName(sg_query_view_state(entry.view)),
           entry.width, entry.height);
}

static sg_shader Renderer_MakeGeneratedMapShader(sg_backend backend) {
    const sg_shader_desc* desc = warped_map_shader_map_shader_desc(backend);
    if (!desc) {
        printf("[Renderer] No generated shader descriptor for backend %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return sg_make_shader(desc);
}

static sg_shader Renderer_MakeGLMapShader(void) {
    sg_shader_desc sd = {};
    sd.label = "map-shader";
    sd.vertex_func.source   = WARPED_VS_SRC;
    sd.fragment_func.source = WARPED_FS_SRC;

    sd.attrs[ATTR_warped_map_shader_map_a_pos].glsl_name  = "a_pos";
    sd.attrs[ATTR_warped_map_shader_map_a_nrm].glsl_name  = "a_nrm";
    sd.attrs[ATTR_warped_map_shader_map_a_uv].glsl_name   = "a_uv";
    sd.attrs[ATTR_warped_map_shader_map_a_lmuv].glsl_name = "a_lmuv";

    sd.uniform_blocks[UB_warped_map_shader_vs_params].stage  = SG_SHADERSTAGE_VERTEX;
    sd.uniform_blocks[UB_warped_map_shader_vs_params].size   = sizeof(warped_map_shader_vs_params_t);
    sd.uniform_blocks[UB_warped_map_shader_vs_params].layout = SG_UNIFORMLAYOUT_NATIVE;
    sd.uniform_blocks[UB_warped_map_shader_vs_params].glsl_uniforms[0].type      = SG_UNIFORMTYPE_MAT4;
    sd.uniform_blocks[UB_warped_map_shader_vs_params].glsl_uniforms[0].glsl_name = "u_mvp";
    sd.uniform_blocks[UB_warped_map_shader_vs_params].glsl_uniforms[1].type      = SG_UNIFORMTYPE_MAT4;
    sd.uniform_blocks[UB_warped_map_shader_vs_params].glsl_uniforms[1].glsl_name = "u_model";

    sd.views[VIEW_warped_map_shader_u_tex].texture.stage       = SG_SHADERSTAGE_FRAGMENT;
    sd.views[VIEW_warped_map_shader_u_tex].texture.image_type  = SG_IMAGETYPE_2D;
    sd.views[VIEW_warped_map_shader_u_tex].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;
    sd.views[VIEW_warped_map_shader_u_lm].texture.stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.views[VIEW_warped_map_shader_u_lm].texture.image_type   = SG_IMAGETYPE_2D;
    sd.views[VIEW_warped_map_shader_u_lm].texture.sample_type  = SG_IMAGESAMPLETYPE_FLOAT;

    sd.samplers[SMP_warped_map_shader_u_tex_smp].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[SMP_warped_map_shader_u_tex_smp].sampler_type = SG_SAMPLERTYPE_FILTERING;
    sd.samplers[SMP_warped_map_shader_u_lm_smp].stage         = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[SMP_warped_map_shader_u_lm_smp].sampler_type  = SG_SAMPLERTYPE_FILTERING;

    sd.texture_sampler_pairs[0].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[0].view_slot    = VIEW_warped_map_shader_u_tex;
    sd.texture_sampler_pairs[0].sampler_slot = SMP_warped_map_shader_u_tex_smp;
    sd.texture_sampler_pairs[0].glsl_name    = "u_tex";
    sd.texture_sampler_pairs[1].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[1].view_slot    = VIEW_warped_map_shader_u_lm;
    sd.texture_sampler_pairs[1].sampler_slot = SMP_warped_map_shader_u_lm_smp;
    sd.texture_sampler_pairs[1].glsl_name    = "u_lm";

    return sg_make_shader(&sd);
}

static sg_shader Renderer_MakePlatformMapShader(sg_backend backend) {
#if defined(WARPED_SOKOL_BACKEND_METAL)
    if (backend != SG_BACKEND_METAL_MACOS) {
        printf("[Renderer] Backend mismatch: expected METAL_MACOS, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    printf("[Renderer] Using generated Metal shader for backend %s.\n",
           RendererBackendName(backend));
    return Renderer_MakeGeneratedMapShader(backend);
#elif defined(WARPED_SOKOL_BACKEND_D3D11)
    if (backend != SG_BACKEND_D3D11) {
        printf("[Renderer] Backend mismatch: expected D3D11, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    printf("[Renderer] Using generated HLSL shader for backend %s.\n",
           RendererBackendName(backend));
    return Renderer_MakeGeneratedMapShader(backend);
#elif defined(WARPED_SOKOL_BACKEND_GLCORE)
    if (backend != SG_BACKEND_GLCORE) {
        printf("[Renderer] Backend mismatch: expected GLCORE, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    printf("[Renderer] Using GLSL shader for backend %s.\n",
           RendererBackendName(backend));
    return Renderer_MakeGLMapShader();
#else
    (void)backend;
    printf("[Renderer] No shader path configured for this platform build.\n");
    return {};
#endif
}

// ---------------------------------------------------------------------------
//  TextureManager
// ---------------------------------------------------------------------------
void InitTextureManager(TextureManager& mgr) {
    mgr.textures.clear();
    mgr.activePackPath.clear();
}

static TextureEntry MakeFallbackTexture(void) {
    // 8×8 magenta/black checker – obvious "missing texture".
    static uint32_t pix[8*8];
    for (int y=0; y<8; ++y)
        for (int x=0; x<8; ++x)
            pix[y*8+x] = ((x^y)&1) ? 0xFF000000 : 0xFFFF00FF;

    sg_image_desc id = {};
    id.width  = 8;
    id.height = 8;
    id.pixel_format = SG_PIXELFORMAT_RGBA8;
    id.data.mip_levels[0] = { pix, sizeof(pix) };
    id.label = "fallback-checker";
    sg_image img = sg_make_image(&id);

    sg_view_desc vd = {};
    vd.texture.image = img;
    sg_view view = sg_make_view(&vd);

    TextureEntry e;
    e.image  = img;
    e.view   = view;
    e.width  = 8;
    e.height = 8;
    return e;
}

static bool ParseLightBrushTextureName(const std::string& name, uint8_t& r, uint8_t& g, uint8_t& b) {
    int ir = 0, ig = 0, ib = 0;
    if (std::sscanf(name.c_str(), "__light_brush_%d_%d_%d", &ir, &ig, &ib) != 3) {
        return false;
    }
    ir = std::max(0, std::min(255, ir));
    ig = std::max(0, std::min(255, ig));
    ib = std::max(0, std::min(255, ib));
    r = (uint8_t) ir;
    g = (uint8_t) ig;
    b = (uint8_t) ib;
    return true;
}

static TextureEntry MakeSolidColorTexture(uint8_t r, uint8_t g, uint8_t b, const char* label) {
    const uint8_t pixel[4] = { r, g, b, 255 };

    sg_image_desc id = {};
    id.width = 1;
    id.height = 1;
    id.pixel_format = SG_PIXELFORMAT_RGBA8;
    id.data.mip_levels[0] = { pixel, sizeof(pixel) };
    id.label = label;
    sg_image img = sg_make_image(&id);

    sg_view_desc vd = {};
    vd.texture.image = img;
    sg_view view = sg_make_view(&vd);

    TextureEntry e;
    e.image = img;
    e.view = view;
    e.width = 1;
    e.height = 1;
    return e;
}

const TextureEntry* LoadTextureByName(TextureManager& mgr, const std::string& name) {
    auto it = mgr.textures.find(name);
    if (it != mgr.textures.end()) return &it->second;

    uint8_t lr = 0, lg = 0, lb = 0;
    if (ParseLightBrushTextureName(name, lr, lg, lb)) {
        TextureEntry entry = MakeSolidColorTexture(lr, lg, lb, name.c_str());
        Renderer_LogTextureState(name.c_str(), entry);
        auto [ins, ok] = mgr.textures.emplace(name, entry);
        (void)ok;
        return &ins->second;
    }

    std::string path = "../../assets/textures/" + name + ".png";

    int w = 0, h = 0, comp = 0;
    unsigned char* pixels = nullptr;

    if (!mgr.activePackPath.empty()) {
        std::vector<unsigned char> packedBytes;
        if (LoadRawAssetFromPack(mgr.activePackPath, "textures/" + name + ".png", packedBytes)) {
            pixels = stbi_load_from_memory(packedBytes.data(), (int)packedBytes.size(), &w, &h, &comp, 4);
            if (!pixels) {
                printf("[Renderer] Failed to decode packaged texture '%s' from '%s'.\n",
                       name.c_str(), mgr.activePackPath.c_str());
            }
        }
    }

    if (!pixels) {
        pixels = stbi_load(path.c_str(), &w, &h, &comp, 4);
    }

    TextureEntry entry;
    if (pixels) {
        sg_image_desc id = {};
        id.width  = w;
        id.height = h;
        id.pixel_format = SG_PIXELFORMAT_RGBA8;
        id.data.mip_levels[0] = { pixels, (size_t)(w * h * 4) };
        id.label = name.c_str();
        sg_image img = sg_make_image(&id);
        stbi_image_free(pixels);

        sg_view_desc vd = {};
        vd.texture.image = img;
        sg_view view = sg_make_view(&vd);

        entry.image  = img;
        entry.view   = view;
        entry.width  = w;
        entry.height = h;
        Renderer_LogTextureState(name.c_str(), entry);
    } else {
        printf("[Renderer] Failed to load '%s' – using fallback.\n", path.c_str());
        entry = MakeFallbackTexture();
        Renderer_LogTextureState("fallback-checker", entry);
    }

    auto [ins, ok] = mgr.textures.emplace(name, entry);
    (void)ok;
    return &ins->second;
}

void UnloadAllTextures(TextureManager& mgr) {
    for (auto& kv : mgr.textures) {
        sg_destroy_view (kv.second.view);
        sg_destroy_image(kv.second.image);
    }
    mgr.textures.clear();
}

// ---------------------------------------------------------------------------
//  Renderer init / shutdown
// ---------------------------------------------------------------------------
void Renderer_Init(void) {
    sg_backend backend = sg_query_backend();

    // --- sampler (repeat + bilinear) ------------------------------------
    sg_sampler_desc smp = {};
    smp.min_filter = SG_FILTER_LINEAR;
    smp.mag_filter = SG_FILTER_LINEAR;
    smp.wrap_u     = SG_WRAP_REPEAT;
    smp.wrap_v     = SG_WRAP_REPEAT;
    smp.label      = "map-sampler";
    g_sampler = sg_make_sampler(&smp);

    // --- lightmap sampler (clamp so atlas edges don't bleed) ------------
    sg_sampler_desc lsmp = {};
    lsmp.min_filter = SG_FILTER_LINEAR;
    lsmp.mag_filter = SG_FILTER_LINEAR;
    lsmp.wrap_u     = SG_WRAP_CLAMP_TO_EDGE;
    lsmp.wrap_v     = SG_WRAP_CLAMP_TO_EDGE;
    lsmp.label      = "lightmap-sampler";
    g_lmSampler = sg_make_sampler(&lsmp);

    // --- 1×1 white fallback lightmap (legacy .map path / missing lm) ----
    static uint32_t whitePix = 0xFFFFFFFFu;
    sg_image_desc wid = {};
    wid.width=1; wid.height=1; wid.pixel_format=SG_PIXELFORMAT_RGBA8;
    wid.data.mip_levels[0] = { &whitePix, sizeof(whitePix) };
    wid.label = "lightmap-white";
    g_whiteLm = sg_make_image(&wid);
    sg_view_desc wvd = {}; wvd.texture.image = g_whiteLm;
    g_whiteLmV = sg_make_view(&wvd);

    // --- shader ---------------------------------------------------------
    g_shader = Renderer_MakePlatformMapShader(backend);
    if (!g_shader.id) {
        printf("[Renderer] Failed to create map shader for backend %s.\n",
               RendererBackendName(backend));
        return;
    }
    printf("[Renderer] Shader state: %s\n",
           RendererResourceStateName(sg_query_shader_state(g_shader)));

    // --- pipeline -------------------------------------------------------
    sg_pipeline_desc pd = {};
    pd.label  = "map-pipeline";
    pd.shader = g_shader;

    pd.layout.buffers[0].stride = sizeof(MapVertex);
    pd.layout.attrs[ATTR_warped_map_shader_map_a_pos].format  = SG_VERTEXFORMAT_FLOAT3;
    pd.layout.attrs[ATTR_warped_map_shader_map_a_pos].offset  = offsetof(MapVertex, x);
    pd.layout.attrs[ATTR_warped_map_shader_map_a_nrm].format  = SG_VERTEXFORMAT_FLOAT3;
    pd.layout.attrs[ATTR_warped_map_shader_map_a_nrm].offset  = offsetof(MapVertex, nx);
    pd.layout.attrs[ATTR_warped_map_shader_map_a_uv].format   = SG_VERTEXFORMAT_FLOAT2;
    pd.layout.attrs[ATTR_warped_map_shader_map_a_uv].offset   = offsetof(MapVertex, u);
    pd.layout.attrs[ATTR_warped_map_shader_map_a_lmuv].format = SG_VERTEXFORMAT_FLOAT2;
    pd.layout.attrs[ATTR_warped_map_shader_map_a_lmuv].offset = offsetof(MapVertex, lu);

    pd.index_type           = SG_INDEXTYPE_UINT32;
    pd.cull_mode            = SG_CULLMODE_BACK;
    pd.face_winding         = SG_FACEWINDING_CCW;
    pd.depth.compare        = SG_COMPAREFUNC_LESS_EQUAL;
    pd.depth.write_enabled  = true;

    g_pipeline = sg_make_pipeline(&pd);
    printf("[Renderer] Pipeline state: %s\n",
           RendererResourceStateName(sg_query_pipeline_state(g_pipeline)));
}

void Renderer_Shutdown(void) {
    if (g_pipeline.id)  sg_destroy_pipeline(g_pipeline);
    if (g_shader.id)    sg_destroy_shader(g_shader);
    if (g_sampler.id)   sg_destroy_sampler(g_sampler);
    if (g_lmSampler.id) sg_destroy_sampler(g_lmSampler);
    if (g_whiteLmV.id)  sg_destroy_view(g_whiteLmV);
    if (g_whiteLm.id)   sg_destroy_image(g_whiteLm);
    g_pipeline = {}; g_shader = {}; g_sampler = {};
    g_lmSampler = {}; g_whiteLm = {}; g_whiteLmV = {};
}

sg_sampler Renderer_DefaultSampler(void) { return g_sampler; }

// ---------------------------------------------------------------------------
//  Bucket upload (shared by .map and .bsp paths)
// ---------------------------------------------------------------------------
static void UploadBuckets(MapModel& mdl,
                          const std::vector<MapMeshBucket>& buckets,
                          TextureManager& texMgr)
{
    mdl.meshes.reserve(buckets.size());
    for (auto& b : buckets) {
        if (b.indices.empty()) continue;

        sg_buffer_desc vbd = {};
        vbd.data  = { b.vertices.data(), b.vertices.size() * sizeof(MapVertex) };
        vbd.label = "map-vbuf";
        sg_buffer vbuf = sg_make_buffer(&vbd);

        sg_buffer_desc ibd = {};
        ibd.usage.index_buffer = true;
        ibd.data  = { b.indices.data(), b.indices.size() * sizeof(uint32_t) };
        ibd.label = "map-ibuf";
        sg_buffer ibuf = sg_make_buffer(&ibd);

        const TextureEntry* tex = LoadTextureByName(texMgr, b.texture);
        uint8_t lr = 0, lg = 0, lb = 0;

        AABB bounds = AABBInvalid();
        for (auto& v : b.vertices) AABBExtend(&bounds, (Vector3){v.x,v.y,v.z});

        SubMesh sm;
        sm.vbuf=vbuf; sm.ibuf=ibuf; sm.tex_view=tex->view;
        sm.index_count=(int)b.indices.size(); sm.bounds=bounds;
        sm.lightmap_page = b.lightmapPage;
        sm.fullbright = ParseLightBrushTextureName(b.texture, lr, lg, lb);
        mdl.meshes.push_back(sm);
    }
}

MapModel Renderer_UploadMap(const Map& map, TextureManager& texMgr) {
    std::vector<MapMeshBucket> buckets = BuildMapGeometry(map, texMgr);
    MapModel mdl;
    UploadBuckets(mdl, buckets, texMgr);
    mdl.lightmapViews.push_back(g_whiteLmV);  // no baked lm in legacy path
    printf("[Renderer] Map uploaded: %zu submeshes (no lightmap).\n", mdl.meshes.size());
    return mdl;
}

MapModel Renderer_UploadBSP(const BSPData& bsp, TextureManager& texMgr) {
    MapModel mdl;
    texMgr.activePackPath = bsp.assetPackPath;
    UploadBuckets(mdl, bsp.buckets, texMgr);

    for (const BSPDataLightmapPage& page : bsp.lightmapPages) {
        if (page.width <= 0 || page.height <= 0 || page.pixels.empty()) {
            continue;
        }
        sg_image_desc id = {};
        id.width = page.width;
        id.height = page.height;
        id.pixel_format = SG_PIXELFORMAT_RGBA8;
        id.data.mip_levels[0] = { page.pixels.data(), page.pixels.size() };
        id.label = "lightmap-page";
        sg_image image = sg_make_image(&id);
        sg_view_desc vd = {};
        vd.texture.image = image;
        mdl.lightmapImages.push_back(image);
        mdl.lightmapViews.push_back(sg_make_view(&vd));
    }

    if (mdl.lightmapViews.empty()) {
        mdl.lightmapViews.push_back(g_whiteLmV);
    }
    printf("[Renderer] BSP uploaded: %zu submeshes, %zu lightmap pages.\n",
           mdl.meshes.size(), mdl.lightmapViews.size());
    return mdl;
}

// ---------------------------------------------------------------------------
//  Draw
// ---------------------------------------------------------------------------
void Renderer_DrawMap(const MapModel& mdl,
                      const Matrix&   mvp,
                      const Matrix&   model,
                      const Frustum&  frustum)
{
    if (!g_pipeline.id) {
        return;
    }

    warped_map_shader_vs_params_t vs = {};
    float16 m = MatrixToFloat16(mvp);
    float16 n = MatrixToFloat16(model);
    for (int i=0;i<16;++i) { vs.u_mvp[i]=m.v[i]; vs.u_model[i]=n.v[i]; }

    sg_apply_pipeline(g_pipeline);

    for (auto& sm : mdl.meshes) {
        // CPU frustum cull — skip submeshes fully outside view + render-distance
        if (!FrustumAABB(&frustum, sm.bounds)) continue;
        const bool hasPage = sm.lightmap_page < mdl.lightmapViews.size();
        const sg_view lightmap_view = (sm.fullbright || !hasPage) ? g_whiteLmV : mdl.lightmapViews[sm.lightmap_page];

        if (!g_logged_bind_diagnostics) {
            printf("[Renderer] Draw bind states: diffuse_view=%s lightmap_view=%s diffuse_sampler=%s lightmap_sampler=%s\n",
                   RendererResourceStateName(sg_query_view_state(sm.tex_view)),
                   RendererResourceStateName(sg_query_view_state(lightmap_view)),
                   RendererResourceStateName(sg_query_sampler_state(g_sampler)),
                   RendererResourceStateName(sg_query_sampler_state(g_lmSampler)));
            g_logged_bind_diagnostics = true;
        }

        sg_bindings bnd = {};
        bnd.vertex_buffers[0] = sm.vbuf;
        bnd.index_buffer      = sm.ibuf;
        bnd.views[VIEW_warped_map_shader_u_tex]       = sm.tex_view;
        bnd.views[VIEW_warped_map_shader_u_lm]        = lightmap_view;
        bnd.samplers[SMP_warped_map_shader_u_tex_smp] = g_sampler;
        bnd.samplers[SMP_warped_map_shader_u_lm_smp]  = g_lmSampler;

        sg_apply_bindings(&bnd);
        sg_apply_uniforms(UB_warped_map_shader_vs_params, { &vs, sizeof(vs) });
        sg_draw(0, sm.index_count, 1);
    }
}

void Renderer_DestroyMap(MapModel& mdl) {
    for (auto& sm : mdl.meshes) {
        sg_destroy_buffer(sm.vbuf);
        sg_destroy_buffer(sm.ibuf);
    }
    mdl.meshes.clear();
    for (size_t i = 0; i < mdl.lightmapImages.size(); ++i) {
        if (i < mdl.lightmapViews.size() && mdl.lightmapViews[i].id) {
            sg_destroy_view(mdl.lightmapViews[i]);
        }
        if (mdl.lightmapImages[i].id) {
            sg_destroy_image(mdl.lightmapImages[i]);
        }
    }
    mdl.lightmapImages.clear();
    mdl.lightmapViews.clear();
}
