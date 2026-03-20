// main.cpp  —  sokol_app + sokol_gfx entry point.
//
// Same boot sequence as the raylib version:
//   parse .map → build render/collision mesh → start Jolt → game loop.
//
// sokol_app drives the loop via callbacks; we buffer input between
// event_cb and frame_cb.

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_log.h"
#include "sokol_glue.h"
#include "sokol_debugtext.h"

#include "math/wmath.h"
#include "platform/platform.h"
#include "input/input.h"
#include "render/renderer.h"
#include "render/debug_draw.h"
#include "utils/map_parser.h"
#include "utils/parameters.h"
#include "physx/collision_data.h"
#include "physx/physics.h"
#include "player/player.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyInterface.h"

#include <cstdio>
#include <vector>
#include <chrono>
#include <thread>

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------
static struct {
    TextureManager      texMgr;
    MapModel            mapModel;
    Player              player;
    JPH::BodyInterface* bodyInterface = nullptr;

    sg_pass_action      passAction;
} G;

// ---------------------------------------------------------------------------
//  Frame-rate limiter
// ---------------------------------------------------------------------------
// Caps render rate at FPS_MAX (parameters.cpp).  0 = uncapped.
// Hybrid sleep + spin: sleep coarse, spin the last ~1 ms for tight pacing
// without full busy-wait CPU burn.  Called once at the top of frame() so the
// whole render path runs exactly once per budget window.
static void FrameRateLimiter(void)
{
    using clock = std::chrono::steady_clock;
    static clock::time_point next = clock::now();

    if (FPS_MAX <= 0) {                 // uncapped: just reseed so re-enabling later is clean
        next = clock::now();
        return;
    }

    const auto budget = std::chrono::nanoseconds((long long)(1.0e9 / (double)FPS_MAX));
    const auto spin   = std::chrono::microseconds(1000);      // spin the final ms

    auto now = clock::now();
    if (next < now) next = now;                               // fell behind — reseed

    // coarse sleep (leave headroom for scheduler granularity)
    if (next - now > spin) {
        std::this_thread::sleep_for((next - now) - spin);
    }
    // fine spin
    while (clock::now() < next) { /* busy-wait */ }

    next += budget;
}

// ---------------------------------------------------------------------------
//  INIT
// ---------------------------------------------------------------------------
static void init(void) {

    // --- sokol_gfx ------------------------------------------------------
    sg_desc gfx = {};
    gfx.environment = sglue_environment();
    gfx.logger.func = slog_func;
    sg_setup(&gfx);

    // sokol_app's macOS/GL path hard-locks vsync; punch through and disable it
    // so we can run past the display's adaptive-refresh ceiling.
    WarpedPlatform_SetSwapInterval(0);

    // --- debug text -----------------------------------------------------
    sdtx_desc_t dtx = {};
    dtx.logger.func = slog_func;
    dtx.fonts[0] = sdtx_font_oric();
    sdtx_setup(&dtx);

    // --- debug 3-D lines (sokol_gl) ------------------------------------
    Debug_Init();

    // --- map renderer pipeline -----------------------------------------
    Renderer_Init();

    // --- Texture manager -----------------------------------------------
    InitTextureManager(G.texMgr);

    printf("[init] sokol gfx+app OK\n");

    // --- Parse map ------------------------------------------------------
    Map map = ParseMapFile("../../assets/maps/test.map");

    std::vector<PlayerStart> starts = GetPlayerStarts(map);
    Vector3 spawnRL = starts.empty() ? (Vector3){0,0,0} : starts[0].position;

    // --- Player ---------------------------------------------------------
    Input_Init();
    InitPlayer(&G.player, spawnRL, (Vector3){0,0,0}, (Vector3){0,1,0}, 90.0f);

    printf("\n Spawn point:: x:%f y:%f z:%f\n\n",
           G.player.center.x, G.player.center.y, G.player.center.z);

    // --- Upload render geometry ----------------------------------------
    G.mapModel = Renderer_UploadMap(map, G.texMgr);

    // --- Physics --------------------------------------------------------
    InitPhysicsSystem();
    G.bodyInterface = &GetBodyInterface();

    printf("\n\n EXTRACTING COLLISION DATA");
    std::vector<MeshCollisionData> collisionData = ExtractCollisionData(map);
    printf("\n\n COLLISION DATA EXTRACTED SUCCESSFULLY");

    BuildMapPhysics(collisionData, G.bodyInterface);
    SpawnDebugPhysObj(G.bodyInterface);

    InitJoltCharacter(&G.player, s_physics_system);

    // --- Pass action ----------------------------------------------------
    G.passAction = {};
    G.passAction.colors[0].load_action = SG_LOADACTION_CLEAR;
    G.passAction.colors[0].clear_value = { 0.97f, 0.97f, 0.97f, 1.0f };
    G.passAction.depth.load_action = SG_LOADACTION_CLEAR;
    G.passAction.depth.clear_value = 1.0f;
}

// ---------------------------------------------------------------------------
//  FRAME
// ---------------------------------------------------------------------------
static void frame(void) {

    // ----- FPS cap ------------------------------------------------------
    FrameRateLimiter();

    // Real wall-clock delta — used only for on-screen FPS readout and to feed
    // the fixed-timestep accumulator.
    float frameDt = (float)sapp_frame_duration();

    // ----- simulate (fixed-timestep accumulator) ------------------------
    // The raylib build ran SetTargetFPS(144) and ticked the sim once per
    // frame with the fixed `deltaTime` step.  All movement / physics constants
    // were tuned against that effective rate, so to preserve feel we replicate
    // it exactly: tick at SIM_TICK_RATE Hz wall-clock, each tick integrates
    // by the fixed `deltaTime`.  Render rate (FPS_MAX) is fully decoupled.
    //
    // Input accumulates across render frames; Input_EndFrame() after each tick
    // consumes mouse Δ + key-pressed edges so only the first tick in a batch
    // sees the accumulated mouse motion (yaw += Δ is not dt-integrated).
    const double simTickInterval = 1.0 / (double)SIM_TICK_RATE;

    static double accumulator = 0.0;
    accumulator += (double)frameDt;
    if (accumulator > 0.25) accumulator = 0.25;          // clamp: avoid spiral-of-death

    while (accumulator >= simTickInterval) {

        // window helpers — at sim rate so KeyPressed edges fire exactly once
        if (Input_KeyPressed(WKEY_TAB))    sapp_toggle_fullscreen();
        if (Input_KeyPressed(WKEY_ESCAPE)) sapp_request_quit();

        UpdatePhysicsSystem(deltaTime, G.bodyInterface);

        if (DEVMODE) {
            UpdatePlayer(&G.player, s_physics_system, deltaTime);
        } else {
            UpdatePlayerMove(&G.player, s_physics_system, deltaTime);
        }

        Input_EndFrame();                                // consume mouse Δ + pressed edges
        accumulator -= simTickInterval;
    }

    UpdateCameraTarget(&G.player);

    // ----- matrices -----------------------------------------------------
    float aspect = (float)sapp_width() / (float)sapp_height();
    Matrix proj  = MatrixPerspective(G.player.camera.fovy * DEG2RAD, aspect, 0.1f, RENDER_DISTANCE);
    Matrix view  = MatrixLookAt(G.player.camera.position,
                                G.player.camera.target,
                                G.player.camera.up);
    Matrix vp    = MatrixMultiply(proj, view);          // column-major: P * V
    Matrix model = MatrixIdentity();
    Matrix mvp   = MatrixMultiply(vp, model);
    Frustum frustum = FrustumFromVP(vp);                // for per-submesh cull

    // ----- 3-D debug batch (recorded, flushed inside pass) -------------
    Debug_NewFrame();
    Debug_SetCamera(proj, view);

    if (G.bodyInterface->IsActive(debugSphereID)) {
        JPH::RVec3 pos = G.bodyInterface->GetCenterOfMassPosition(debugSphereID);
        Debug_Sphere((Vector3){(float)pos.GetX(),(float)pos.GetY(),(float)pos.GetZ()},
                     10.0f, WCOLOR(230,41,55,255));
    }
    DebugDrawPlayerAABB(&G.player);
    DebugDir(&G.player);

    // ----- debug text (recorded) ---------------------------------------
    sdtx_canvas(sapp_widthf()*0.5f, sapp_heightf()*0.5f);   // 2× scale
    sdtx_origin(1.0f, 1.0f);
    sdtx_font(0);

    sdtx_color3b(0,200,0);
    sdtx_pos(0,0);
    sdtx_printf("FPS %5.1f", (frameDt>0.0f)?1.0f/frameDt:0.0f);

    DebugDrawPlayerPos(&G.player, 0, 2);
    DebugDrawPlayerVel(0, 4);

    // ----- render pass --------------------------------------------------
    sg_pass pass = {};
    pass.action    = G.passAction;
    pass.swapchain = sglue_swapchain();
    sg_begin_pass(&pass);

        Renderer_DrawMap(G.mapModel, mvp, model, frustum);
        Debug_Flush();
        sdtx_draw();

    sg_end_pass();
    sg_commit();
}

// ---------------------------------------------------------------------------
//  EVENTS
// ---------------------------------------------------------------------------
static void event(const sapp_event* ev) {
    Input_HandleEvent(ev);
}

// ---------------------------------------------------------------------------
//  CLEANUP
// ---------------------------------------------------------------------------
static void cleanup(void) {
    ShutdownPhysicsSystem();
    Renderer_DestroyMap(G.mapModel);
    UnloadAllTextures(G.texMgr);
    Renderer_Shutdown();
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
    d.init_cb     = init;
    d.frame_cb    = frame;
    d.event_cb    = event;
    d.cleanup_cb  = cleanup;
    d.width       = SCREEN_WIDTH;
    d.height      = SCREEN_HEIGHT;
    d.window_title = "Warped Engine";
    d.sample_count = 4;
    d.swap_interval = 0;          // uncapped — raylib build ran at FPS_MAX (144)
    d.logger.func = slog_func;
    return d;
}
