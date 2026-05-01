// main.cpp  —  sokol_app + sokol_gfx entry point.

#if defined(WARPED_SOKOL_BACKEND_METAL) && !defined(SOKOL_METAL)
#define SOKOL_METAL
#elif defined(WARPED_SOKOL_BACKEND_D3D11) && !defined(SOKOL_D3D11)
#define SOKOL_D3D11
#elif defined(WARPED_SOKOL_BACKEND_GLCORE) && !defined(SOKOL_GLCORE)
#define SOKOL_GLCORE
#endif

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_gl.h"
#include "sokol_glue.h"
#include "sokol_log.h"
#include "sokol_debugtext.h"

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "math/wmath.h"
#include "platform/platform.h"
#include "input/input.h"
#include "render/renderer.h"
#include "render/debug_draw.h"
#include "utils/bsp_loader.h"
#include "utils/parameters.h"
#include "physx/collision_data.h"
#include "physx/physics.h"
#include "player/player.h"
#include "ui/ui_menu.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyInterface.h"

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------
static struct {
    TextureManager      texMgr;
    MapModel            mapModel;
    Player              player;
    JPH::BodyInterface* bodyInterface = nullptr;

    sg_pass_action      menuPassAction;
    sg_pass_action      gamePassAction;

    AppMode             mode = AppMode::MainMenu;
    bool                gameplayLoaded = false;
    bool                physicsReady = false;

    std::vector<MapEntry> availableMaps;
    std::string           menuStatus;
    std::string           currentMapName;
} G;

static const char* GfxBackendName(sg_backend backend) {
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

static bool LoadSelectedMap(const MapEntry& map) {
    if (G.gameplayLoaded) {
        G.menuStatus = "Map switching from the menu is not implemented yet.";
        return false;
    }

    BSPData bsp;
    if (!LoadBSP(map.bspPath.c_str(), bsp)) {
        G.menuStatus = "Failed to load " + map.name + ".";
        return false;
    }

    if (!G.physicsReady) {
        InitPhysicsSystem();
        G.bodyInterface = &GetBodyInterface();
        G.physicsReady = true;
    }

    std::vector<PlayerStart> starts = GetPlayerStarts(bsp);
    const PlayerStart start = starts.empty() ? PlayerStart{ (Vector3){0, 0, 0}, 0.0f, 0.0f } : starts[0];
    Vector3 spawn = start.position;

    InitPlayer(&G.player, spawn, (Vector3){0, 0, 0}, (Vector3){0, 1, 0}, 90.0f);
    G.mapModel = Renderer_UploadBSP(bsp, G.texMgr);

    BuildMapPhysics(bsp.hulls, bsp.entities, G.bodyInterface);
    SpawnDebugPhysObj(G.bodyInterface);
    InitJoltCharacter(&G.player, s_physics_system);
    RespawnPlayer(&G.player, s_physics_system, start.position, start.yaw, start.pitch);

    G.currentMapName = map.name;
    G.menuStatus.clear();
    G.gameplayLoaded = true;
    G.mode = AppMode::Playing;
    Input_LockMouse(true);
    Input_EndFrame();

    printf("[menu] loaded map '%s'\n", map.bspPath.c_str());
    return true;
}

// ---------------------------------------------------------------------------
//  Frame-rate limiter
// ---------------------------------------------------------------------------
static void FrameRateLimiter(void)
{
    using clock = std::chrono::steady_clock;
    static clock::time_point next = clock::now();

    if (FPS_MAX <= 0) {
        next = clock::now();
        return;
    }

    const auto budget = std::chrono::nanoseconds((long long)(1.0e9 / (double)FPS_MAX));
    const auto spin   = std::chrono::microseconds(1000);

    auto now = clock::now();
    if (next < now) next = now;

    if (next - now > spin) {
        std::this_thread::sleep_for((next - now) - spin);
    }
    while (clock::now() < next) { }

    next += budget;
}

// ---------------------------------------------------------------------------
//  INIT
// ---------------------------------------------------------------------------
static void init(void) {

    sg_desc gfx = {};
    gfx.buffer_pool_size = 1024;
    gfx.image_pool_size = 512;
    gfx.sampler_pool_size = 128;
    gfx.shader_pool_size = 64;
    gfx.pipeline_pool_size = 128;
    gfx.view_pool_size = 1024;
    gfx.environment = sglue_environment();
    gfx.logger.func = slog_func;
    sg_setup(&gfx);

#if defined(__APPLE__)
    WarpedPlatform_SetSwapInterval(0);
#endif

    printf("[init] sokol backend: %s\n", GfxBackendName(sg_query_backend()));

    sdtx_desc_t dtx = {};
    dtx.logger.func = slog_func;
    dtx.fonts[0] = sdtx_font_oric();
    sdtx_setup(&dtx);

    Debug_Init();
    UI_Init(sapp_width(), sapp_height());

    Renderer_Init();
    InitTextureManager(G.texMgr);
    Input_Init();
    Input_LockMouse(false);

    UI_RefreshMapList(G.availableMaps, G.menuStatus);

    G.menuPassAction = {};
    G.menuPassAction.colors[0].load_action = SG_LOADACTION_CLEAR;
    G.menuPassAction.colors[0].clear_value = { 0.06f, 0.07f, 0.09f, 1.0f };
    G.menuPassAction.depth.load_action = SG_LOADACTION_CLEAR;
    G.menuPassAction.depth.clear_value = 1.0f;

    G.gamePassAction = {};
    G.gamePassAction.colors[0].load_action = SG_LOADACTION_CLEAR;
    G.gamePassAction.colors[0].clear_value = { 0.52f, 0.68f, 0.86f, 1.0f };
    G.gamePassAction.depth.load_action = SG_LOADACTION_CLEAR;
    G.gamePassAction.depth.clear_value = 1.0f;

    printf("[init] main menu ready\n");
}

// ---------------------------------------------------------------------------
//  FRAME
// ---------------------------------------------------------------------------
static void frame(void) {
    FrameRateLimiter();

    float frameDt = (float)sapp_frame_duration();
    static float postTime = 0.0f;
    postTime += frameDt;
    static double accumulator = 0.0;

    if (G.mode == AppMode::MainMenu) {
        if (Input_KeyPressed(WKEY_TAB)) {
            sapp_toggle_fullscreen();
        }
        if (Input_KeyPressed(WKEY_ESCAPE)) {
            sapp_request_quit();
        }

        UI_NewFrame();
        int requestedMapIndex = UI_UpdateMenu(G.availableMaps, G.menuStatus);

        if (requestedMapIndex >= 0 && requestedMapIndex < (int)G.availableMaps.size()) {
            LoadSelectedMap(G.availableMaps[requestedMapIndex]);
            accumulator = 0.0;
            if (G.mode == AppMode::Playing) {
                return;
            }
        }

        sg_pass pass = {};
        pass.action = G.menuPassAction;
        pass.swapchain = sglue_swapchain();
        sg_begin_pass(&pass);
            sgl_defaults();
            sgl_viewport(0, 0, sapp_width(), sapp_height(), true);
            sgl_matrix_mode_projection();
            sgl_load_identity();
            sgl_matrix_mode_modelview();
            sgl_load_identity();
            UI_RenderMenu();
            sgl_draw();
        sg_end_pass();
        sg_commit();

        Input_EndFrame();
        return;
    }

    const double simTickInterval = 1.0 / (double)SIM_TICK_RATE;
    accumulator += (double)frameDt;
    if (accumulator > 0.25) accumulator = 0.25;

    while (accumulator >= simTickInterval) {
        if (Input_KeyPressed(WKEY_TAB))    sapp_toggle_fullscreen();
        if (Input_KeyPressed(WKEY_ESCAPE)) sapp_request_quit();

        UpdatePhysicsSystem(deltaTime, G.bodyInterface);

        if (DEVMODE) {
            UpdatePlayer(&G.player, s_physics_system, deltaTime);
        } else {
            UpdatePlayerMove(&G.player, s_physics_system, deltaTime);
        }

        Input_EndFrame();
        accumulator -= simTickInterval;
    }

    UpdateCameraTarget(&G.player);

    float aspect = (float)sapp_width() / (float)sapp_height();
    Matrix proj  = MatrixPerspective(G.player.camera.fovy * DEG2RAD, aspect, 0.1f, RENDER_DISTANCE);
    Matrix view  = MatrixLookAt(G.player.camera.position,
                                G.player.camera.target,
                                G.player.camera.up);
    Matrix vp    = MatrixMultiply(proj, view);
    Matrix model = MatrixIdentity();
    Matrix mvp   = MatrixMultiply(vp, model);
    Matrix normalModel = MatrixMultiply(view, model);
    Frustum frustum = FrustumFromVP(vp);

    Debug_NewFrame();
    Debug_SetCamera(proj, view);

    if (G.bodyInterface && G.bodyInterface->IsActive(debugSphereID)) {
        JPH::RVec3 pos = G.bodyInterface->GetCenterOfMassPosition(debugSphereID);
        Debug_Sphere((Vector3){(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()},
                     10.0f, WCOLOR(230, 41, 55, 255));
    }
    DebugDrawPlayerAABB(&G.player);
    DebugDrawGroundProbe();
    DebugDir(&G.player);

    sdtx_canvas(sapp_widthf()*0.5f, sapp_heightf()*0.5f);
    sdtx_origin(1.0f, 1.0f);
    sdtx_font(0);

    sdtx_color3b(0, 200, 0);
    sdtx_pos(0, 0);
    sdtx_printf("FPS %5.1f", (frameDt > 0.0f) ? 1.0f / frameDt : 0.0f);
    sdtx_pos(0, 1);
    sdtx_printf("MAP %s", G.currentMapName.c_str());

    DebugDrawPlayerPos(&G.player, 0, 3);
    DebugDrawPlayerVel(0, 5);

    bool usePost = Renderer_BeginScenePostPass(G.gamePassAction,
                                               sapp_width(),
                                               sapp_height(),
                                               sapp_sample_count());
    if (usePost) {
        Renderer_DrawMap(G.mapModel, mvp, model, frustum);
        Renderer_EndScenePostPass();
        if (usePost) {
            usePost = Renderer_BeginNormalPostPass(sapp_width(), sapp_height());
            if (usePost) {
                Renderer_DrawMapNormals(G.mapModel, mvp, normalModel, frustum);
                Renderer_EndNormalPostPass();
            }
        }
    }

    if (usePost) {
        sg_pass_action presentAction = {};
        presentAction.colors[0].load_action = SG_LOADACTION_DONTCARE;
        presentAction.depth.load_action = SG_LOADACTION_CLEAR;
        presentAction.depth.clear_value = 1.0f;

        sg_pass pass = {};
        pass.action = presentAction;
        pass.swapchain = sglue_swapchain();
        sg_begin_pass(&pass);
            Renderer_DrawPencilPostProcess(postTime);
            Debug_Flush();
            sdtx_draw();
        sg_end_pass();
    } else {
        sg_pass pass = {};
        pass.action = G.gamePassAction;
        pass.swapchain = sglue_swapchain();
        sg_begin_pass(&pass);
            Renderer_DrawMap(G.mapModel, mvp, model, frustum);
            Debug_Flush();
            sdtx_draw();
        sg_end_pass();
    }
    sg_commit();
}

// ---------------------------------------------------------------------------
//  EVENTS
// ---------------------------------------------------------------------------
static void event(const sapp_event* ev) {
    Input_HandleEvent(ev);
    UI_HandleEvent(ev);
}

// ---------------------------------------------------------------------------
//  CLEANUP
// ---------------------------------------------------------------------------
static void cleanup(void) {
    if (G.gameplayLoaded) {
        Renderer_DestroyMap(G.mapModel);
    }
    if (G.physicsReady) {
        ShutdownPhysicsSystem();
    }
    UnloadAllTextures(G.texMgr);
    Renderer_Shutdown();
    UI_Shutdown();
    Debug_Shutdown();
    sdtx_shutdown();
    sg_shutdown();
}

// ---------------------------------------------------------------------------
//  sokol_app entry
// ---------------------------------------------------------------------------
sapp_desc sokol_main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    sapp_desc d = {};
    d.init_cb      = init;
    d.frame_cb     = frame;
    d.event_cb     = event;
    d.cleanup_cb   = cleanup;
    d.width        = SCREEN_WIDTH;
    d.height       = SCREEN_HEIGHT;
    d.window_title = "Warped Engine";
    d.sample_count = 4;
    d.swap_interval = 0;
    d.high_dpi     = true;
    d.logger.func  = slog_func;
    return d;
}
