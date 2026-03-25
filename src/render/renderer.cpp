// renderer.cpp
//
// All GPU-facing code lives here.  The brush geometry itself is still produced
// by map_parser.cpp via BuildMapGeometry(); we just take the resulting CPU
// arrays, upload them to sokol_gfx buffers and render them with a single
// textured-diffuse pipeline.

#include "renderer.h"
#include "shaders.h"
#include "../utils/map_parser.h"
#include "../utils/bsp_loader.h"
#include "sokol_gfx.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

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

struct VsParams {
    float mvp  [16];
    float model[16];
};

// ---------------------------------------------------------------------------
//  TextureManager
// ---------------------------------------------------------------------------
void InitTextureManager(TextureManager& mgr) {
    mgr.textures.clear();
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

const TextureEntry* LoadTextureByName(TextureManager& mgr, const std::string& name) {
    auto it = mgr.textures.find(name);
    if (it != mgr.textures.end()) return &it->second;

    std::string path = "../../assets/textures/" + name + ".png";

    int w = 0, h = 0, comp = 0;
    unsigned char* pixels = stbi_load(path.c_str(), &w, &h, &comp, 4);   // force RGBA

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
    } else {
        printf("[Renderer] Failed to load '%s' – using fallback.\n", path.c_str());
        entry = MakeFallbackTexture();
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
    sg_shader_desc sd = {};
    sd.label = "map-shader";
    sd.vertex_func.source   = WARPED_VS_SRC;
    sd.fragment_func.source = WARPED_FS_SRC;

    sd.attrs[0].glsl_name = "a_pos";
    sd.attrs[1].glsl_name = "a_nrm";
    sd.attrs[2].glsl_name = "a_uv";
    sd.attrs[3].glsl_name = "a_lmuv";

    // uniform block 0 – vertex stage, two mat4
    sd.uniform_blocks[0].stage  = SG_SHADERSTAGE_VERTEX;
    sd.uniform_blocks[0].size   = sizeof(VsParams);
    sd.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_NATIVE;
    sd.uniform_blocks[0].glsl_uniforms[0].type      = SG_UNIFORMTYPE_MAT4;
    sd.uniform_blocks[0].glsl_uniforms[0].glsl_name = "u_mvp";
    sd.uniform_blocks[0].glsl_uniforms[1].type      = SG_UNIFORMTYPE_MAT4;
    sd.uniform_blocks[0].glsl_uniforms[1].glsl_name = "u_model";

    // texture views — slot 0 diffuse, slot 1 lightmap
    sd.views[0].texture.stage       = SG_SHADERSTAGE_FRAGMENT;
    sd.views[0].texture.image_type  = SG_IMAGETYPE_2D;
    sd.views[0].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;
    sd.views[1].texture.stage       = SG_SHADERSTAGE_FRAGMENT;
    sd.views[1].texture.image_type  = SG_IMAGETYPE_2D;
    sd.views[1].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;

    // samplers — slot 0 wraps (diffuse), slot 1 clamps (lightmap)
    sd.samplers[0].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[0].sampler_type = SG_SAMPLERTYPE_FILTERING;
    sd.samplers[1].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[1].sampler_type = SG_SAMPLERTYPE_FILTERING;

    // combined image-sampler pairs → GLSL names
    sd.texture_sampler_pairs[0].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[0].view_slot    = 0;
    sd.texture_sampler_pairs[0].sampler_slot = 0;
    sd.texture_sampler_pairs[0].glsl_name    = "u_tex";
    sd.texture_sampler_pairs[1].stage        = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[1].view_slot    = 1;
    sd.texture_sampler_pairs[1].sampler_slot = 1;
    sd.texture_sampler_pairs[1].glsl_name    = "u_lm";

    g_shader = sg_make_shader(&sd);

    // --- pipeline -------------------------------------------------------
    sg_pipeline_desc pd = {};
    pd.label  = "map-pipeline";
    pd.shader = g_shader;

    pd.layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3;   // pos
    pd.layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3;   // normal
    pd.layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT2;   // diffuse uv
    pd.layout.attrs[3].format = SG_VERTEXFORMAT_FLOAT2;   // lightmap uv

    pd.index_type           = SG_INDEXTYPE_UINT32;
    pd.cull_mode            = SG_CULLMODE_BACK;
    pd.face_winding         = SG_FACEWINDING_CCW;
    pd.depth.compare        = SG_COMPAREFUNC_LESS_EQUAL;
    pd.depth.write_enabled  = true;

    g_pipeline = sg_make_pipeline(&pd);
}

void Renderer_Shutdown(void) {
    sg_destroy_pipeline(g_pipeline);
    sg_destroy_shader  (g_shader);
    sg_destroy_sampler (g_sampler);
    sg_destroy_sampler (g_lmSampler);
    sg_destroy_view    (g_whiteLmV);
    sg_destroy_image   (g_whiteLm);
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

        AABB bounds = AABBInvalid();
        for (auto& v : b.vertices) AABBExtend(&bounds, (Vector3){v.x,v.y,v.z});

        SubMesh sm;
        sm.vbuf=vbuf; sm.ibuf=ibuf; sm.tex_view=tex->view;
        sm.index_count=(int)b.indices.size(); sm.bounds=bounds;
        mdl.meshes.push_back(sm);
    }
}

MapModel Renderer_UploadMap(const Map& map, TextureManager& texMgr) {
    std::vector<MapMeshBucket> buckets = BuildMapGeometry(map, texMgr);
    MapModel mdl;
    UploadBuckets(mdl, buckets, texMgr);
    mdl.lightmapView = g_whiteLmV;            // no baked lm in legacy path
    printf("[Renderer] Map uploaded: %zu submeshes (no lightmap).\n", mdl.meshes.size());
    return mdl;
}

MapModel Renderer_UploadBSP(const BSPData& bsp, TextureManager& texMgr) {
    MapModel mdl;
    UploadBuckets(mdl, bsp.buckets, texMgr);

    if (bsp.lightmapW>0 && !bsp.lightmapPixels.empty()) {
        sg_image_desc id = {};
        id.width=bsp.lightmapW; id.height=bsp.lightmapH;
        id.pixel_format=SG_PIXELFORMAT_RGBA8;
        id.data.mip_levels[0] = { bsp.lightmapPixels.data(), bsp.lightmapPixels.size() };
        id.label="lightmap-atlas";
        mdl.lightmapImage = sg_make_image(&id);
        sg_view_desc vd={}; vd.texture.image=mdl.lightmapImage;
        mdl.lightmapView = sg_make_view(&vd);
    } else {
        mdl.lightmapView = g_whiteLmV;
    }
    printf("[Renderer] BSP uploaded: %zu submeshes, lm %dx%d.\n",
           mdl.meshes.size(), bsp.lightmapW, bsp.lightmapH);
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
    VsParams vs;
    float16 m = MatrixToFloat16(mvp);
    float16 n = MatrixToFloat16(model);
    for (int i=0;i<16;++i) { vs.mvp[i]=m.v[i]; vs.model[i]=n.v[i]; }

    sg_apply_pipeline(g_pipeline);

    for (auto& sm : mdl.meshes) {
        // CPU frustum cull — skip submeshes fully outside view + render-distance
        if (!FrustumAABB(&frustum, sm.bounds)) continue;

        sg_bindings bnd = {};
        bnd.vertex_buffers[0] = sm.vbuf;
        bnd.index_buffer      = sm.ibuf;
        bnd.views[0]          = sm.tex_view;
        bnd.views[1]          = mdl.lightmapView;
        bnd.samplers[0]       = g_sampler;
        bnd.samplers[1]       = g_lmSampler;

        sg_apply_bindings(&bnd);
        sg_apply_uniforms(0, { &vs, sizeof(vs) });
        sg_draw(0, sm.index_count, 1);
    }
}

void Renderer_DestroyMap(MapModel& mdl) {
    for (auto& sm : mdl.meshes) {
        sg_destroy_buffer(sm.vbuf);
        sg_destroy_buffer(sm.ibuf);
    }
    mdl.meshes.clear();
    if (mdl.lightmapImage.id) {
        sg_destroy_view (mdl.lightmapView);
        sg_destroy_image(mdl.lightmapImage);
        mdl.lightmapImage={}; mdl.lightmapView={};
    }
}
