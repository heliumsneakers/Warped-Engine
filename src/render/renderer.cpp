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
#include "shaders/generated/normal.metal_dx11.h"
#include "shaders/generated/pencil.metal_dx11.h"
#include "../compiler/map_parser.h"
#include "../utils/bsp_loader.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
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

struct PencilPostVertex {
    float x;
    float y;
    float u;
    float v;
};

static sg_shader   g_pencilShader        = {};
static sg_pipeline g_pencilPipeline      = {};
static sg_buffer   g_pencilVertexBuffer  = {};
static sg_shader   g_normalShader        = {};
static sg_pipeline g_normalPipeline      = {};
static sg_sampler  g_postSceneSampler    = {};
static sg_sampler  g_postNormalSampler   = {};

static sg_image g_sceneColor = {};
static sg_image g_sceneResolve = {};
static sg_image g_sceneDepth = {};
static sg_view  g_sceneColorAttView = {};
static sg_view  g_sceneResolveAttView = {};
static sg_view  g_sceneDepthAttView = {};
static sg_view  g_sceneTextureView = {};
static int      g_sceneWidth = 0;
static int      g_sceneHeight = 0;
static int      g_sceneSampleCount = 0;

static sg_image g_normalColor = {};
static sg_image g_normalDepth = {};
static sg_image g_normalDepthColor = {};
static sg_view  g_normalColorAttView = {};
static sg_view  g_normalDepthAttView = {};
static sg_view  g_normalDepthColorAttView = {};
static sg_view  g_normalTextureView = {};
static sg_view  g_normalDepthTextureView = {};

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

static sg_pixel_format Renderer_LightmapPixelFormat(uint32_t format) {
    switch (format) {
        case BSP_LIGHTMAP_FORMAT_RGBA8_UNORM: return SG_PIXELFORMAT_RGBA8;
        case BSP_LIGHTMAP_FORMAT_RGBA16F:     return SG_PIXELFORMAT_RGBA16F;
        default:                              return SG_PIXELFORMAT_NONE;
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

static sg_shader Renderer_MakeGeneratedPencilShader(sg_backend backend) {
    const sg_shader_desc* desc = warped_pencil_shader_pencil_post_shader_desc(backend);
    if (!desc) {
        printf("[Renderer] No generated pencil shader descriptor for backend %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return sg_make_shader(desc);
}

static const char* WARPED_PENCIL_VS_SRC =
    "#version 330\n"
    "layout(location=0) in vec2 a_pos;\n"
    "layout(location=1) in vec2 a_uv;\n"
    "out vec2 v_uv;\n"
    "void main() {\n"
    "    v_uv = a_uv;\n"
    "    gl_Position = vec4(a_pos, 0.0, 1.0);\n"
    "}\n";

static const char* WARPED_PENCIL_FS_SRC =
    "#version 330\n"
    "uniform fs_params {\n"
    "    vec4 u_resolution_time;\n"
    "    vec4 u_effect_params;\n"
    "};\n"
    "\n"
    "uniform sampler2D u_scene;\n"
    "uniform sampler2D u_normals;\n"
    "uniform sampler2D u_depth;\n"
    "\n"
    "in vec2 v_uv;\n"
    "out vec4 frag_color;\n"
    "\n"
    "float luminance(vec3 c) {\n"
    "    return dot(c, vec3(0.299, 0.587, 0.114));\n"
    "}\n"
    "\n"
    "vec4 normal_sample(vec2 uv) {\n"
    "    vec4 n = texture(u_normals, uv);\n"
    "    return vec4(n.xyz * 2.0 - 1.0, n.a);\n"
    "}\n"
    "\n"
    "float depth_sample(vec2 uv) {\n"
    "    return texture(u_depth, uv).r;\n"
    "}\n"
    "\n"
    "float normal_delta(vec4 center, vec4 sample_n) {\n"
    "    float center_occ = step(0.0001, center.a);\n"
    "    float sample_occ = step(0.0001, sample_n.a);\n"
    "    float silhouette = abs(center_occ - sample_occ);\n"
    "    float crease = length(center.xyz - sample_n.xyz) * center_occ * sample_occ;\n"
    "    return max(silhouette, crease);\n"
    "}\n"
    "\n"
    "float geometry_edge(vec2 uv, vec2 texel, float radius, vec4 center_n, float center_d) {\n"
    "    float center_occ = step(0.0001, center_n.a);\n"
    "\n"
    "    vec4 s0 = normal_sample(uv + texel * vec2( radius, 0.0));\n"
    "    vec4 s1 = normal_sample(uv + texel * vec2(-radius, 0.0));\n"
    "    vec4 s2 = normal_sample(uv + texel * vec2(0.0,  radius));\n"
    "    vec4 s3 = normal_sample(uv + texel * vec2(0.0, -radius));\n"
    "\n"
    "    float nd0 = normal_delta(center_n, s0);\n"
    "    float nd1 = normal_delta(center_n, s1);\n"
    "    float nd2 = normal_delta(center_n, s2);\n"
    "    float nd3 = normal_delta(center_n, s3);\n"
    "\n"
    "    float support = step(0.12, nd0) + step(0.12, nd1) + step(0.12, nd2) + step(0.12, nd3);\n"
    "    float normal_e = max(max(nd0, nd1), max(nd2, nd3)) * step(1.0, support);\n"
    "\n"
    "    float d0 = depth_sample(uv + texel * vec2( radius, 0.0));\n"
    "    float d1 = depth_sample(uv + texel * vec2(-radius, 0.0));\n"
    "    float d2 = depth_sample(uv + texel * vec2(0.0,  radius));\n"
    "    float d3 = depth_sample(uv + texel * vec2(0.0, -radius));\n"
    "\n"
    "    float laplacian_h = abs(d0 + d1 - 2.0 * center_d);\n"
    "    float laplacian_v = abs(d2 + d3 - 2.0 * center_d);\n"
    "    float depth_e = smoothstep(0.008, 0.03, max(laplacian_h, laplacian_v) / max(center_d, 0.0001));\n"
    "    float max_rel_depth = max(max(abs(d0 - center_d), abs(d1 - center_d)),\n"
    "                              max(abs(d2 - center_d), abs(d3 - center_d))) / max(center_d, 0.0001);\n"
    "    float max_normal_delta = max(max(nd0, nd1), max(nd2, nd3));\n"
    "    float same_surface_noise = (1.0 - step(0.08, max_normal_delta)) *\n"
    "                               (1.0 - smoothstep(0.015, 0.08, max_rel_depth));\n"
    "    depth_e *= 1.0 - same_surface_noise;\n"
    "\n"
    "    return max(normal_e, depth_e) * center_occ;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    vec2 resolution = max(u_resolution_time.xy, vec2(1.0));\n"
    "    vec2 texel = 1.0 / resolution;\n"
    "    vec2 uv = clamp(v_uv, texel, 1.0 - texel);\n"
    "\n"
    "    vec4 scene = texture(u_scene, uv);\n"
    "    vec4 center_n = normal_sample(uv);\n"
    "    float center_d = depth_sample(uv);\n"
    "    float lum = luminance(scene.rgb);\n"
    "\n"
    "    float edge_gain = max(u_effect_params.x, 0.01);\n"
    "    float far_radius = max(u_effect_params.y, 0.25);\n"
    "    float near_radius = max(u_effect_params.z, far_radius);\n"
    "    float scene_mix = clamp(u_effect_params.w, 0.0, 1.0);\n"
    "\n"
    "    float view_dist = center_d * 4096.0;\n"
    "    float near_factor = clamp(1.0 - smoothstep(96.0, 896.0, view_dist), 0.0, 1.0);\n"
    "    float radius = mix(far_radius, near_radius, near_factor);\n"
    "    float edge_response = geometry_edge(uv, texel, radius, center_n, center_d);\n"
    "    float edge = smoothstep(0.16, 0.42, edge_response * edge_gain);\n"
    "\n"
    "    vec3 paper_color = vec3(0.965, 0.935, 0.865);\n"
    "    vec3 graphite = vec3(0.105, 0.092, 0.082);\n"
    "    vec3 washed_scene = mix(vec3(lum), scene.rgb, 0.34);\n"
    "    vec3 base = mix(paper_color, washed_scene, scene_mix);\n"
    "\n"
    "    float pencil = clamp(edge * 0.95, 0.0, 1.0) * step(0.0001, center_n.a);\n"
    "    vec3 color = mix(base, graphite, pencil);\n"
    "\n"
    "    frag_color = vec4(color, scene.a);\n"
    "}\n";


static sg_shader Renderer_MakeGLPencilShader(void) {
    sg_shader_desc sd = {};
    sd.label = "pencil-post-shader";
    sd.vertex_func.source = WARPED_PENCIL_VS_SRC;
    sd.fragment_func.source = WARPED_PENCIL_FS_SRC;

    sd.attrs[ATTR_warped_pencil_shader_pencil_post_a_pos].glsl_name = "a_pos";
    sd.attrs[ATTR_warped_pencil_shader_pencil_post_a_uv].glsl_name = "a_uv";

    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].size = sizeof(warped_pencil_shader_fs_params_t);
    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].layout = SG_UNIFORMLAYOUT_NATIVE;
    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].glsl_uniforms[0].type = SG_UNIFORMTYPE_FLOAT4;
    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].glsl_uniforms[0].glsl_name = "u_resolution_time";
    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].glsl_uniforms[1].type = SG_UNIFORMTYPE_FLOAT4;
    sd.uniform_blocks[UB_warped_pencil_shader_fs_params].glsl_uniforms[1].glsl_name = "u_effect_params";

    sd.views[VIEW_warped_pencil_shader_u_scene].texture.stage = SG_SHADERSTAGE_FRAGMENT;
    sd.views[VIEW_warped_pencil_shader_u_scene].texture.image_type = SG_IMAGETYPE_2D;
    sd.views[VIEW_warped_pencil_shader_u_scene].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;
    sd.views[VIEW_warped_pencil_shader_u_normals].texture.stage = SG_SHADERSTAGE_FRAGMENT;
    sd.views[VIEW_warped_pencil_shader_u_normals].texture.image_type = SG_IMAGETYPE_2D;
    sd.views[VIEW_warped_pencil_shader_u_normals].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;
    sd.views[VIEW_warped_pencil_shader_u_depth].texture.stage = SG_SHADERSTAGE_FRAGMENT;
    sd.views[VIEW_warped_pencil_shader_u_depth].texture.image_type = SG_IMAGETYPE_2D;
    sd.views[VIEW_warped_pencil_shader_u_depth].texture.sample_type = SG_IMAGESAMPLETYPE_FLOAT;

    sd.samplers[SMP_warped_pencil_shader_u_scene_smp].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[SMP_warped_pencil_shader_u_scene_smp].sampler_type = SG_SAMPLERTYPE_FILTERING;
    sd.samplers[SMP_warped_pencil_shader_u_normal_smp].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[SMP_warped_pencil_shader_u_normal_smp].sampler_type = SG_SAMPLERTYPE_FILTERING;
    sd.samplers[SMP_warped_pencil_shader_u_depth_smp].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.samplers[SMP_warped_pencil_shader_u_depth_smp].sampler_type = SG_SAMPLERTYPE_FILTERING;

    sd.texture_sampler_pairs[0].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[0].view_slot = VIEW_warped_pencil_shader_u_scene;
    sd.texture_sampler_pairs[0].sampler_slot = SMP_warped_pencil_shader_u_scene_smp;
    sd.texture_sampler_pairs[0].glsl_name = "u_scene";
    sd.texture_sampler_pairs[1].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[1].view_slot = VIEW_warped_pencil_shader_u_normals;
    sd.texture_sampler_pairs[1].sampler_slot = SMP_warped_pencil_shader_u_normal_smp;
    sd.texture_sampler_pairs[1].glsl_name = "u_normals";
    sd.texture_sampler_pairs[2].stage = SG_SHADERSTAGE_FRAGMENT;
    sd.texture_sampler_pairs[2].view_slot = VIEW_warped_pencil_shader_u_depth;
    sd.texture_sampler_pairs[2].sampler_slot = SMP_warped_pencil_shader_u_depth_smp;
    sd.texture_sampler_pairs[2].glsl_name = "u_depth";
    return sg_make_shader(&sd);
}

static sg_shader Renderer_MakePlatformPencilShader(sg_backend backend) {
#if defined(WARPED_SOKOL_BACKEND_METAL)
    if (backend != SG_BACKEND_METAL_MACOS) {
        printf("[Renderer] Backend mismatch: expected METAL_MACOS, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return Renderer_MakeGeneratedPencilShader(backend);
#elif defined(WARPED_SOKOL_BACKEND_D3D11)
    if (backend != SG_BACKEND_D3D11) {
        printf("[Renderer] Backend mismatch: expected D3D11, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return Renderer_MakeGeneratedPencilShader(backend);
#elif defined(WARPED_SOKOL_BACKEND_GLCORE)
    if (backend != SG_BACKEND_GLCORE) {
        printf("[Renderer] Backend mismatch: expected GLCORE, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return Renderer_MakeGLPencilShader();
#else
    (void)backend;
    return {};
#endif
}

static sg_shader Renderer_MakeGeneratedNormalShader(sg_backend backend) {
    const sg_shader_desc* desc = warped_normal_shader_normal_pass_shader_desc(backend);
    if (!desc) {
        printf("[Renderer] No generated normal shader descriptor for backend %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return sg_make_shader(desc);
}

static const char* WARPED_NORMAL_VS_SRC =
    "#version 330\n"
    "uniform mat4 u_mvp;\n"
    "uniform mat4 u_normal_model;\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec3 a_nrm;\n"
    "out vec3 v_nrm;\n"
    "out float v_view_dist;\n"
    "void main() {\n"
    "    v_nrm = normalize(mat3(u_normal_model) * a_nrm);\n"
    "    vec4 view_pos = u_normal_model * vec4(a_pos, 1.0);\n"
    "    v_view_dist = max(-view_pos.z, 0.0);\n"
    "    gl_Position = u_mvp * vec4(a_pos, 1.0);\n"
    "}\n";

static const char* WARPED_NORMAL_FS_SRC =
    "#version 330\n"
    "in vec3 v_nrm;\n"
    "in float v_view_dist;\n"
    "layout(location=0) out vec4 frag_color;\n"
    "layout(location=1) out vec4 frag_depth_out;\n"
    "void main() {\n"
    "    vec3 n = normalize(v_nrm) * 0.5 + 0.5;\n"
    "    frag_color = vec4(n, 1.0);\n"
    "    frag_depth_out = vec4(v_view_dist / 4096.0, 0.0, 0.0, 0.0);\n"
    "}\n";

static sg_shader Renderer_MakeGLNormalShader(void) {
    sg_shader_desc sd = {};
    sd.label = "normal-post-shader";
    sd.vertex_func.source = WARPED_NORMAL_VS_SRC;
    sd.fragment_func.source = WARPED_NORMAL_FS_SRC;

    sd.attrs[ATTR_warped_normal_shader_normal_pass_a_pos].glsl_name = "a_pos";
    sd.attrs[ATTR_warped_normal_shader_normal_pass_a_nrm].glsl_name = "a_nrm";

    sd.uniform_blocks[UB_warped_normal_shader_vs_params].stage = SG_SHADERSTAGE_VERTEX;
    sd.uniform_blocks[UB_warped_normal_shader_vs_params].size = sizeof(warped_normal_shader_vs_params_t);
    sd.uniform_blocks[UB_warped_normal_shader_vs_params].layout = SG_UNIFORMLAYOUT_NATIVE;
    sd.uniform_blocks[UB_warped_normal_shader_vs_params].glsl_uniforms[0].type = SG_UNIFORMTYPE_MAT4;
    sd.uniform_blocks[UB_warped_normal_shader_vs_params].glsl_uniforms[0].glsl_name = "u_mvp";
    sd.uniform_blocks[UB_warped_normal_shader_vs_params].glsl_uniforms[1].type = SG_UNIFORMTYPE_MAT4;
    sd.uniform_blocks[UB_warped_normal_shader_vs_params].glsl_uniforms[1].glsl_name = "u_normal_model";

    return sg_make_shader(&sd);
}

static sg_shader Renderer_MakePlatformNormalShader(sg_backend backend) {
#if defined(WARPED_SOKOL_BACKEND_METAL)
    if (backend != SG_BACKEND_METAL_MACOS) {
        printf("[Renderer] Backend mismatch: expected METAL_MACOS, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return Renderer_MakeGeneratedNormalShader(backend);
#elif defined(WARPED_SOKOL_BACKEND_D3D11)
    if (backend != SG_BACKEND_D3D11) {
        printf("[Renderer] Backend mismatch: expected D3D11, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return Renderer_MakeGeneratedNormalShader(backend);
#elif defined(WARPED_SOKOL_BACKEND_GLCORE)
    if (backend != SG_BACKEND_GLCORE) {
        printf("[Renderer] Backend mismatch: expected GLCORE, got %s.\n",
               RendererBackendName(backend));
        return {};
    }
    return Renderer_MakeGLNormalShader();
#else
    (void)backend;
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

    TextureEntry entry;
    bool loaded = false;

    // Try loading mipmapped asset from pack
    if (!mgr.activePackPath.empty()) {
        MipmapChain mipChain;
        if (LoadMipmappedAssetFromPack(mgr.activePackPath, "textures/" + name + ".png", mipChain) &&
            !mipChain.levels.empty()) {
            const int numMips = std::min((int)mipChain.levels.size(), (int)SG_MAX_MIPMAPS);
            printf("[Renderer] Loaded mipmapped texture '%s': %dx%d, %d mip levels\n",
                   name.c_str(), mipChain.levels[0].width, mipChain.levels[0].height, numMips);
            sg_image_desc id = {};
            id.width = mipChain.levels[0].width;
            id.height = mipChain.levels[0].height;
            id.num_mipmaps = numMips;
            id.pixel_format = SG_PIXELFORMAT_RGBA8;
            for (int m = 0; m < numMips; ++m) {
                id.data.mip_levels[m] = {
                    mipChain.levels[m].pixels.data(),
                    (size_t)(mipChain.levels[m].width * mipChain.levels[m].height * 4)
                };
            }
            id.label = name.c_str();
            sg_image img = sg_make_image(&id);

            sg_view_desc vd = {};
            vd.texture.image = img;
            sg_view view = sg_make_view(&vd);

            entry.image = img;
            entry.view = view;
            entry.width = id.width;
            entry.height = id.height;
            Renderer_LogTextureState(name.c_str(), entry);
            loaded = true;
        }
    }

    // Fallback: load single-level texture from filesystem (no mipmaps)
    if (!loaded) {
        int w = 0, h = 0, comp = 0;
        unsigned char* pixels = stbi_load(path.c_str(), &w, &h, &comp, 4);
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
            loaded = true;
        }
    }

    if (!loaded) {
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

static void Renderer_DestroyScenePostTargets(void) {
    if (g_sceneTextureView.id) {
        sg_destroy_view(g_sceneTextureView);
    }
    if (g_sceneDepthAttView.id) {
        sg_destroy_view(g_sceneDepthAttView);
    }
    if (g_sceneResolveAttView.id) {
        sg_destroy_view(g_sceneResolveAttView);
    }
    if (g_sceneColorAttView.id) {
        sg_destroy_view(g_sceneColorAttView);
    }
    if (g_normalTextureView.id) {
        sg_destroy_view(g_normalTextureView);
    }
    if (g_normalDepthTextureView.id) {
        sg_destroy_view(g_normalDepthTextureView);
    }
    if (g_normalDepthColorAttView.id) {
        sg_destroy_view(g_normalDepthColorAttView);
    }
    if (g_normalDepthAttView.id) {
        sg_destroy_view(g_normalDepthAttView);
    }
    if (g_normalColorAttView.id) {
        sg_destroy_view(g_normalColorAttView);
    }
    if (g_sceneDepth.id) {
        sg_destroy_image(g_sceneDepth);
    }
    if (g_sceneResolve.id) {
        sg_destroy_image(g_sceneResolve);
    }
    if (g_sceneColor.id) {
        sg_destroy_image(g_sceneColor);
    }
    if (g_normalDepthColor.id) {
        sg_destroy_image(g_normalDepthColor);
    }
    if (g_normalDepth.id) {
        sg_destroy_image(g_normalDepth);
    }
    if (g_normalColor.id) {
        sg_destroy_image(g_normalColor);
    }

    g_sceneTextureView = {};
    g_sceneDepthAttView = {};
    g_sceneResolveAttView = {};
    g_sceneColorAttView = {};
    g_sceneDepth = {};
    g_sceneResolve = {};
    g_sceneColor = {};
    g_normalTextureView = {};
    g_normalDepthTextureView = {};
    g_normalDepthColorAttView = {};
    g_normalDepthAttView = {};
    g_normalColorAttView = {};
    g_normalDepthColor = {};
    g_normalDepth = {};
    g_normalColor = {};
    g_sceneWidth = 0;
    g_sceneHeight = 0;
    g_sceneSampleCount = 0;
}

static bool Renderer_EnsureScenePostTargets(int width, int height, int sampleCount) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    sampleCount = std::max(1, sampleCount);

    if (g_sceneWidth == width &&
        g_sceneHeight == height &&
        g_sceneSampleCount == sampleCount &&
        sg_query_view_state(g_sceneTextureView) == SG_RESOURCESTATE_VALID &&
        sg_query_view_state(g_normalTextureView) == SG_RESOURCESTATE_VALID &&
        sg_query_view_state(g_normalDepthTextureView) == SG_RESOURCESTATE_VALID) {
        return true;
    }

    Renderer_DestroyScenePostTargets();

    const sg_environment env = sglue_environment();

    sg_image_desc colorDesc = {};
    colorDesc.usage.color_attachment = true;
    colorDesc.width = width;
    colorDesc.height = height;
    colorDesc.pixel_format = env.defaults.color_format;
    colorDesc.sample_count = sampleCount;
    colorDesc.label = "scene-post-color";
    g_sceneColor = sg_make_image(&colorDesc);

    if (sampleCount > 1) {
        sg_image_desc resolveDesc = {};
        resolveDesc.usage.resolve_attachment = true;
        resolveDesc.width = width;
        resolveDesc.height = height;
        resolveDesc.pixel_format = env.defaults.color_format;
        resolveDesc.sample_count = 1;
        resolveDesc.label = "scene-post-resolve";
        g_sceneResolve = sg_make_image(&resolveDesc);
    }

    sg_image_desc depthDesc = {};
    depthDesc.usage.depth_stencil_attachment = true;
    depthDesc.width = width;
    depthDesc.height = height;
    depthDesc.pixel_format = env.defaults.depth_format;
    depthDesc.sample_count = sampleCount;
    depthDesc.label = "scene-post-depth";
    g_sceneDepth = sg_make_image(&depthDesc);

    sg_view_desc colorViewDesc = {};
    colorViewDesc.color_attachment.image = g_sceneColor;
    colorViewDesc.label = "scene-post-color-view";
    g_sceneColorAttView = sg_make_view(&colorViewDesc);

    if (sampleCount > 1) {
        sg_view_desc resolveViewDesc = {};
        resolveViewDesc.resolve_attachment.image = g_sceneResolve;
        resolveViewDesc.label = "scene-post-resolve-view";
        g_sceneResolveAttView = sg_make_view(&resolveViewDesc);
    }

    sg_view_desc depthViewDesc = {};
    depthViewDesc.depth_stencil_attachment.image = g_sceneDepth;
    depthViewDesc.label = "scene-post-depth-view";
    g_sceneDepthAttView = sg_make_view(&depthViewDesc);

    sg_view_desc textureViewDesc = {};
    textureViewDesc.texture.image = (sampleCount > 1) ? g_sceneResolve : g_sceneColor;
    textureViewDesc.label = "scene-post-texture-view";
    g_sceneTextureView = sg_make_view(&textureViewDesc);

    sg_image_desc normalColorDesc = {};
    normalColorDesc.usage.color_attachment = true;
    normalColorDesc.width = width;
    normalColorDesc.height = height;
    normalColorDesc.pixel_format = env.defaults.color_format;
    normalColorDesc.sample_count = 1;
    normalColorDesc.label = "normal-post-color";
    g_normalColor = sg_make_image(&normalColorDesc);

    sg_image_desc normalDepthColorDesc = {};
    normalDepthColorDesc.usage.color_attachment = true;
    normalDepthColorDesc.width = width;
    normalDepthColorDesc.height = height;
    normalDepthColorDesc.pixel_format = SG_PIXELFORMAT_R16F;
    normalDepthColorDesc.sample_count = 1;
    normalDepthColorDesc.label = "normal-depth-color";
    g_normalDepthColor = sg_make_image(&normalDepthColorDesc);

    sg_image_desc normalDepthDesc = {};
    normalDepthDesc.usage.depth_stencil_attachment = true;
    normalDepthDesc.width = width;
    normalDepthDesc.height = height;
    normalDepthDesc.pixel_format = env.defaults.depth_format;
    normalDepthDesc.sample_count = 1;
    normalDepthDesc.label = "normal-post-depth";
    g_normalDepth = sg_make_image(&normalDepthDesc);

    sg_view_desc normalColorViewDesc = {};
    normalColorViewDesc.color_attachment.image = g_normalColor;
    normalColorViewDesc.label = "normal-post-color-view";
    g_normalColorAttView = sg_make_view(&normalColorViewDesc);

    sg_view_desc normalDepthColorViewDesc = {};
    normalDepthColorViewDesc.color_attachment.image = g_normalDepthColor;
    normalDepthColorViewDesc.label = "normal-depth-color-view";
    g_normalDepthColorAttView = sg_make_view(&normalDepthColorViewDesc);

    sg_view_desc normalDepthViewDesc = {};
    normalDepthViewDesc.depth_stencil_attachment.image = g_normalDepth;
    normalDepthViewDesc.label = "normal-post-depth-view";
    g_normalDepthAttView = sg_make_view(&normalDepthViewDesc);

    sg_view_desc normalTextureViewDesc = {};
    normalTextureViewDesc.texture.image = g_normalColor;
    normalTextureViewDesc.label = "normal-post-texture-view";
    g_normalTextureView = sg_make_view(&normalTextureViewDesc);

    sg_view_desc normalDepthTextureViewDesc = {};
    normalDepthTextureViewDesc.texture.image = g_normalDepthColor;
    normalDepthTextureViewDesc.label = "normal-depth-texture-view";
    g_normalDepthTextureView = sg_make_view(&normalDepthTextureViewDesc);

    const bool ok =
        (sg_query_image_state(g_sceneColor) == SG_RESOURCESTATE_VALID) &&
        (sg_query_image_state(g_sceneDepth) == SG_RESOURCESTATE_VALID) &&
        (sg_query_image_state(g_normalColor) == SG_RESOURCESTATE_VALID) &&
        (sg_query_image_state(g_normalDepthColor) == SG_RESOURCESTATE_VALID) &&
        (sg_query_image_state(g_normalDepth) == SG_RESOURCESTATE_VALID) &&
        (sampleCount <= 1 || sg_query_image_state(g_sceneResolve) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_sceneColorAttView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_sceneDepthAttView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_normalColorAttView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_normalDepthColorAttView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_normalDepthAttView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_normalTextureView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_normalDepthTextureView) == SG_RESOURCESTATE_VALID) &&
        (sampleCount <= 1 || sg_query_view_state(g_sceneResolveAttView) == SG_RESOURCESTATE_VALID) &&
        (sg_query_view_state(g_sceneTextureView) == SG_RESOURCESTATE_VALID);

    if (!ok) {
        printf("[Renderer] Failed to create post targets: color=%s resolve=%s depth=%s textureView=%s normal=%s normalView=%s depthColor=%s.\n",
               RendererResourceStateName(sg_query_image_state(g_sceneColor)),
               (sampleCount > 1) ? RendererResourceStateName(sg_query_image_state(g_sceneResolve)) : "unused",
               RendererResourceStateName(sg_query_image_state(g_sceneDepth)),
               RendererResourceStateName(sg_query_view_state(g_sceneTextureView)),
               RendererResourceStateName(sg_query_image_state(g_normalColor)),
               RendererResourceStateName(sg_query_view_state(g_normalTextureView)),
               RendererResourceStateName(sg_query_image_state(g_normalDepthColor)));
        Renderer_DestroyScenePostTargets();
        return false;
    }

    g_sceneWidth = width;
    g_sceneHeight = height;
    g_sceneSampleCount = sampleCount;
    return true;
}

static void Renderer_InitPencilPostProcess(sg_backend backend) {
    sg_sampler_desc sceneSamplerDesc = {};
    sceneSamplerDesc.min_filter = SG_FILTER_LINEAR;
    sceneSamplerDesc.mag_filter = SG_FILTER_LINEAR;
    sceneSamplerDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
    sceneSamplerDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
    sceneSamplerDesc.label = "post-scene-sampler";
    g_postSceneSampler = sg_make_sampler(&sceneSamplerDesc);

    sg_sampler_desc normalSamplerDesc = {};
    normalSamplerDesc.min_filter = SG_FILTER_NEAREST;
    normalSamplerDesc.mag_filter = SG_FILTER_NEAREST;
    normalSamplerDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
    normalSamplerDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
    normalSamplerDesc.label = "post-normal-sampler";
    g_postNormalSampler = sg_make_sampler(&normalSamplerDesc);

    g_pencilShader = Renderer_MakePlatformPencilShader(backend);
    if (!g_pencilShader.id) {
        printf("[Renderer] Failed to create pencil post shader for backend %s.\n",
               RendererBackendName(backend));
        return;
    }

    static const PencilPostVertex kFullscreenVerts[] = {
        { -1.0f, -1.0f, 0.0f, 1.0f },
        {  1.0f, -1.0f, 1.0f, 1.0f },
        { -1.0f,  1.0f, 0.0f, 0.0f },
        { -1.0f,  1.0f, 0.0f, 0.0f },
        {  1.0f, -1.0f, 1.0f, 1.0f },
        {  1.0f,  1.0f, 1.0f, 0.0f },
    };

    sg_buffer_desc vbd = {};
    vbd.data = { kFullscreenVerts, sizeof(kFullscreenVerts) };
    vbd.label = "pencil-post-vbuf";
    g_pencilVertexBuffer = sg_make_buffer(&vbd);

    const sg_environment env = sglue_environment();
    sg_pipeline_desc pd = {};
    pd.label = "pencil-post-pipeline";
    pd.shader = g_pencilShader;
    pd.layout.buffers[0].stride = sizeof(PencilPostVertex);
    pd.layout.attrs[ATTR_warped_pencil_shader_pencil_post_a_pos].format = SG_VERTEXFORMAT_FLOAT2;
    pd.layout.attrs[ATTR_warped_pencil_shader_pencil_post_a_pos].offset = offsetof(PencilPostVertex, x);
    pd.layout.attrs[ATTR_warped_pencil_shader_pencil_post_a_uv].format = SG_VERTEXFORMAT_FLOAT2;
    pd.layout.attrs[ATTR_warped_pencil_shader_pencil_post_a_uv].offset = offsetof(PencilPostVertex, u);
    pd.color_count = 1;
    pd.colors[0].pixel_format = env.defaults.color_format;
    pd.depth.pixel_format = env.defaults.depth_format;
    pd.depth.compare = SG_COMPAREFUNC_ALWAYS;
    pd.depth.write_enabled = false;
    pd.sample_count = env.defaults.sample_count;
    pd.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    pd.cull_mode = SG_CULLMODE_NONE;
    g_pencilPipeline = sg_make_pipeline(&pd);

    printf("[Renderer] Pencil post pipeline state: %s\n",
           RendererResourceStateName(sg_query_pipeline_state(g_pencilPipeline)));

    g_normalShader = Renderer_MakePlatformNormalShader(backend);
    if (!g_normalShader.id) {
        printf("[Renderer] Failed to create normal post shader for backend %s.\n",
               RendererBackendName(backend));
        return;
    }

    sg_pipeline_desc npd = {};
    npd.label = "normal-post-pipeline";
    npd.shader = g_normalShader;
    npd.layout.buffers[0].stride = sizeof(MapVertex);
    npd.layout.attrs[ATTR_warped_normal_shader_normal_pass_a_pos].format = SG_VERTEXFORMAT_FLOAT3;
    npd.layout.attrs[ATTR_warped_normal_shader_normal_pass_a_pos].offset = offsetof(MapVertex, x);
    npd.layout.attrs[ATTR_warped_normal_shader_normal_pass_a_nrm].format = SG_VERTEXFORMAT_FLOAT3;
    npd.layout.attrs[ATTR_warped_normal_shader_normal_pass_a_nrm].offset = offsetof(MapVertex, nx);
    npd.index_type = SG_INDEXTYPE_UINT32;
    npd.cull_mode = SG_CULLMODE_BACK;
    npd.face_winding = SG_FACEWINDING_CCW;
    npd.color_count = 2;
    npd.colors[0].pixel_format = env.defaults.color_format;
    npd.colors[1].pixel_format = SG_PIXELFORMAT_R16F;
    npd.depth.pixel_format = env.defaults.depth_format;
    npd.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
    npd.depth.write_enabled = true;
    npd.sample_count = 1;
    npd.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    g_normalPipeline = sg_make_pipeline(&npd);

    printf("[Renderer] Normal post pipeline state: %s\n",
           RendererResourceStateName(sg_query_pipeline_state(g_normalPipeline)));
}

static void Renderer_DestroyPencilPostProcess(void) {
    Renderer_DestroyScenePostTargets();

    if (g_pencilPipeline.id) {
        sg_destroy_pipeline(g_pencilPipeline);
    }
    if (g_pencilShader.id) {
        sg_destroy_shader(g_pencilShader);
    }
    if (g_pencilVertexBuffer.id) {
        sg_destroy_buffer(g_pencilVertexBuffer);
    }
    if (g_normalPipeline.id) {
        sg_destroy_pipeline(g_normalPipeline);
    }
    if (g_normalShader.id) {
        sg_destroy_shader(g_normalShader);
    }
    if (g_postSceneSampler.id) {
        sg_destroy_sampler(g_postSceneSampler);
    }
    if (g_postNormalSampler.id) {
        sg_destroy_sampler(g_postNormalSampler);
    }

    g_pencilPipeline = {};
    g_pencilShader = {};
    g_pencilVertexBuffer = {};
    g_normalPipeline = {};
    g_normalShader = {};
    g_postSceneSampler = {};
    g_postNormalSampler = {};
}

// ---------------------------------------------------------------------------
//  Renderer init / shutdown
// ---------------------------------------------------------------------------
void Renderer_Init(void) {
    sg_backend backend = sg_query_backend();

    /*
    sg_pixel_format

    sokol_gfx.h basically uses the same pixel formats as WebGPU, since these
    are supported on most newer GPUs.

    A pixelformat name consist of three parts:

        - components (R, RG, RGB or RGBA)
        - bit width per component (8, 16 or 32)
        - component data type:
            - unsigned normalized (no postfix)
            - signed normalized (SN postfix)
            - unsigned integer (UI postfix)
            - signed integer (SI postfix)
            - float (F postfix)

    Not all pixel formats can be used for everything, call sg_query_pixelformat()
    to inspect the capabilities of a given pixelformat. The function returns
    an sg_pixelformat_info struct with the following members:

        - sample: the pixelformat can be sampled as texture at least with
                  nearest filtering
        - filter: the pixelformat can be sampled as texture with linear
                  filtering
        - render: the pixelformat can be used as render-pass attachment
        - blend:  blending is supported when used as render-pass attachment
        - msaa:   multisample-antialiasing is supported when used
                  as render-pass attachment
        - depth:  the pixelformat can be used for depth-stencil attachments
        - compressed: this is a block-compressed format
        - bytes_per_pixel: the numbers of bytes in a pixel (0 for compressed formats)

    The default pixel format for texture images is SG_PIXELFORMAT_RGBA8.

    The default pixel format for render target images is platform-dependent
    and taken from the sg_environment struct passed into sg_setup(). Typically
    the default formats are:

        - for the Metal, D3D11 and WebGPU backends: SG_PIXELFORMAT_BGRA8
        - for GL backends: SG_PIXELFORMAT_RGBA8
*/

    // --- sampler (repeat + trilinear) ------------------------------------
    sg_sampler_desc smp = {};
    smp.min_filter = SG_FILTER_LINEAR;
    smp.mag_filter = SG_FILTER_LINEAR;
    smp.mipmap_filter = SG_FILTER_LINEAR;
    smp.max_anisotropy = 16;
    smp.wrap_u     = SG_WRAP_REPEAT;
    smp.wrap_v     = SG_WRAP_REPEAT;
    smp.label      = "map-sampler";
    g_sampler = sg_make_sampler(&smp);

    // --- lightmap sampler (clamp so atlas edges don't bleed) ------------
    sg_sampler_desc lsmp = {};
    lsmp.min_filter = SG_FILTER_LINEAR;
    lsmp.mag_filter = SG_FILTER_LINEAR;
    lsmp.mipmap_filter = SG_FILTER_LINEAR;
    lsmp.max_anisotropy = 16;
    lsmp.wrap_u     = SG_WRAP_REPEAT;
    lsmp.wrap_v     = SG_WRAP_REPEAT;
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

    Renderer_InitPencilPostProcess(backend);
}

void Renderer_Shutdown(void) {
    Renderer_DestroyPencilPostProcess();
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

bool Renderer_PencilPostProcessReady(void) {
    return (sg_query_pipeline_state(g_pencilPipeline) == SG_RESOURCESTATE_VALID) &&
           (sg_query_pipeline_state(g_normalPipeline) == SG_RESOURCESTATE_VALID) &&
           (sg_query_buffer_state(g_pencilVertexBuffer) == SG_RESOURCESTATE_VALID) &&
           (sg_query_sampler_state(g_postSceneSampler) == SG_RESOURCESTATE_VALID) &&
           (sg_query_sampler_state(g_postNormalSampler) == SG_RESOURCESTATE_VALID);
}

bool Renderer_BeginScenePostPass(const sg_pass_action& action,
                                 int width,
                                 int height,
                                 int sampleCount)
{
    if (!Renderer_PencilPostProcessReady()) {
        return false;
    }
    if (!Renderer_EnsureScenePostTargets(width, height, sampleCount)) {
        return false;
    }

    sg_pass pass = {};
    pass.action = action;
    pass.attachments.colors[0] = g_sceneColorAttView;
    if (g_sceneResolveAttView.id) {
        pass.attachments.resolves[0] = g_sceneResolveAttView;
    }
    pass.attachments.depth_stencil = g_sceneDepthAttView;
    pass.label = "scene-post-pass";
    sg_begin_pass(&pass);
    return true;
}

void Renderer_EndScenePostPass(void) {
    sg_end_pass();
}

bool Renderer_BeginNormalPostPass(int width, int height) {
    if (!Renderer_PencilPostProcessReady() ||
        width != g_sceneWidth ||
        height != g_sceneHeight ||
        sg_query_view_state(g_normalColorAttView) != SG_RESOURCESTATE_VALID ||
        sg_query_view_state(g_normalDepthColorAttView) != SG_RESOURCESTATE_VALID ||
        sg_query_view_state(g_normalDepthAttView) != SG_RESOURCESTATE_VALID ||
        sg_query_view_state(g_normalTextureView) != SG_RESOURCESTATE_VALID ||
        sg_query_view_state(g_normalDepthTextureView) != SG_RESOURCESTATE_VALID) {
        return false;
    }

    sg_pass_action action = {};
    action.colors[0].load_action = SG_LOADACTION_CLEAR;
    action.colors[0].clear_value = { 0.5f, 0.5f, 1.0f, 0.0f };
    action.colors[1].load_action = SG_LOADACTION_CLEAR;
    action.colors[1].clear_value = { 0.0f, 0.0f, 0.0f, 0.0f };
    action.depth.load_action = SG_LOADACTION_CLEAR;
    action.depth.clear_value = 1.0f;

    sg_pass pass = {};
    pass.action = action;
    pass.attachments.colors[0] = g_normalColorAttView;
    pass.attachments.colors[1] = g_normalDepthColorAttView;
    pass.attachments.depth_stencil = g_normalDepthAttView;
    pass.label = "normal-post-pass";
    sg_begin_pass(&pass);
    return true;
}

void Renderer_EndNormalPostPass(void) {
    sg_end_pass();
}

void Renderer_DrawPencilPostProcess(float timeSeconds) {
    if (!Renderer_PencilPostProcessReady() ||
        sg_query_view_state(g_sceneTextureView) != SG_RESOURCESTATE_VALID ||
        sg_query_view_state(g_normalTextureView) != SG_RESOURCESTATE_VALID ||
        sg_query_view_state(g_normalDepthTextureView) != SG_RESOURCESTATE_VALID) {
        return;
    }

    warped_pencil_shader_fs_params_t fs = {};
    fs.u_resolution_time[0] = (float)g_sceneWidth;
    fs.u_resolution_time[1] = (float)g_sceneHeight;
    fs.u_resolution_time[2] = timeSeconds;
    fs.u_resolution_time[3] = 0.0f;
    fs.u_effect_params[0] = 2.85f;
    fs.u_effect_params[1] = 0.70f;
    fs.u_effect_params[2] = 3.35f;
    fs.u_effect_params[3] = 0.58f;

    sg_bindings bnd = {};
    bnd.vertex_buffers[0] = g_pencilVertexBuffer;
    bnd.views[VIEW_warped_pencil_shader_u_scene] = g_sceneTextureView;
    bnd.views[VIEW_warped_pencil_shader_u_normals] = g_normalTextureView;
    bnd.views[VIEW_warped_pencil_shader_u_depth] = g_normalDepthTextureView;
    bnd.samplers[SMP_warped_pencil_shader_u_scene_smp] = g_postSceneSampler;
    bnd.samplers[SMP_warped_pencil_shader_u_normal_smp] = g_postNormalSampler;
    bnd.samplers[SMP_warped_pencil_shader_u_depth_smp] = g_postNormalSampler;

    sg_apply_pipeline(g_pencilPipeline);
    sg_apply_bindings(&bnd);
    sg_apply_uniforms(UB_warped_pencil_shader_fs_params, { &fs, sizeof(fs) });
    sg_draw(0, 6, 1);
}

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
        const sg_pixel_format pixelFormat = Renderer_LightmapPixelFormat(page.format);
        if (pixelFormat == SG_PIXELFORMAT_NONE) {
            printf("[Renderer] Skipping lightmap page with unsupported format %u.\n", page.format);
            continue;
        }
        const sg_pixelformat_info pixelInfo = sg_query_pixelformat(pixelFormat);
        if (!pixelInfo.sample) {
            printf("[Renderer] Skipping lightmap page format %u on backend %s because it is not sampleable.\n",
                   page.format, RendererBackendName(sg_query_backend()));
            continue;
        }
        const size_t expectedBytes = (size_t)page.width * (size_t)page.height * pixelInfo.bytes_per_pixel;
        if (page.pixels.size() != expectedBytes) {
            printf("[Renderer] Skipping malformed lightmap page: format=%u size=%zu expected=%zu.\n",
                   page.format, page.pixels.size(), expectedBytes);
            continue;
        }
        sg_image_desc id = {};
        id.width = page.width;
        id.height = page.height;
        id.pixel_format = pixelFormat;
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

void Renderer_DrawMapNormals(const MapModel& mdl,
                             const Matrix&   mvp,
                             const Matrix&   normalModel,
                             const Frustum&  frustum)
{
    if (!g_normalPipeline.id) {
        return;
    }

    warped_normal_shader_vs_params_t vs = {};
    float16 m = MatrixToFloat16(mvp);
    float16 n = MatrixToFloat16(normalModel);
    for (int i = 0; i < 16; ++i) {
        vs.u_mvp[i] = m.v[i];
        vs.u_normal_model[i] = n.v[i];
    }

    sg_apply_pipeline(g_normalPipeline);

    for (auto& sm : mdl.meshes) {
        if (!FrustumAABB(&frustum, sm.bounds)) {
            continue;
        }

        sg_bindings bnd = {};
        bnd.vertex_buffers[0] = sm.vbuf;
        bnd.index_buffer = sm.ibuf;

        sg_apply_bindings(&bnd);
        sg_apply_uniforms(UB_warped_normal_shader_vs_params, { &vs, sizeof(vs) });
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
