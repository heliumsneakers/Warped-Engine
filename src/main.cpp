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
#include <filesystem>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstdint>

#define FONTSTASH_IMPLEMENTATION
#include "fontstash.h"
#define SOKOL_FONTSTASH_IMPL
#include "sokol_fontstash.h"

#define CLAY_IMPLEMENTATION
#include "clay.h"
#define SOKOL_CLAY_IMPL
#include "renderers/sokol/sokol_clay.h"

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

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyInterface.h"

namespace fs = std::filesystem;

static constexpr const char* kMapDirectory = "../../assets/maps";
static constexpr const char* kMenuFontPath = "../../lib/clay/examples/sokol-video-demo/resources/Roboto-Regular.ttf";

enum ClayFontId {
    FONT_ID_UI_16,
    FONT_ID_UI_24,
    FONT_ID_UI_40,
    FONT_ID_COUNT
};

enum class AppMode {
    MainMenu,
    Playing,
};

struct MapEntry {
    std::string name;
    std::string bspPath;
};

static sclay_font_t g_clayFonts[FONT_ID_COUNT] = {};
static void* g_clayMemory = nullptr;

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
    int                   requestedMapIndex = -1;
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

static Clay_String ClayString(const std::string& str) {
    return Clay_String{ false, (int32_t)str.size(), str.c_str() };
}

static void HandleClayError(Clay_ErrorData errorData) {
    printf("[Clay] %.*s\n", errorData.errorText.length, errorData.errorText.chars);
}

static void RefreshMapList() {
    G.availableMaps.clear();
    G.menuStatus.clear();

    std::error_code ec;
    if (!fs::exists(kMapDirectory, ec)) {
        G.menuStatus = "Map directory not found: assets/maps";
        return;
    }

    for (const fs::directory_entry& entry : fs::directory_iterator(kMapDirectory, ec)) {
        if (ec || !entry.is_regular_file()) {
            continue;
        }
        if (entry.path().extension() != ".bsp") {
            continue;
        }

        G.availableMaps.push_back({
            entry.path().stem().string(),
            entry.path().string()
        });
    }

    std::sort(G.availableMaps.begin(), G.availableMaps.end(), [](const MapEntry& a, const MapEntry& b) {
        return a.name < b.name;
    });

    if (G.availableMaps.empty()) {
        G.menuStatus = "No compiled .bsp maps found in assets/maps.";
    }
}

static void HandleMapButtonInteraction(Clay_ElementId, Clay_PointerData pointerData, void* userData) {
    if (pointerData.state != CLAY_POINTER_DATA_PRESSED_THIS_FRAME) {
        return;
    }
    G.requestedMapIndex = (int)(intptr_t)userData;
}

static Clay_RenderCommandArray BuildMainMenuLayout() {
    const Clay_Color bg = { 18, 20, 24, 255 };
    const Clay_Color panel = { 32, 36, 44, 255 };
    const Clay_Color panelBorder = { 76, 84, 98, 255 };
    const Clay_Color button = { 56, 65, 82, 255 };
    const Clay_Color buttonHover = { 84, 98, 126, 255 };
    const Clay_Color text = { 235, 238, 244, 255 };
    const Clay_Color muted = { 164, 171, 184, 255 };
    const Clay_Color accent = { 123, 196, 255, 255 };

    Clay_BeginLayout();

    CLAY(CLAY_ID("Root"), {
        .backgroundColor = bg,
        .layout = {
            .sizing = { CLAY_SIZING_GROW(0), CLAY_SIZING_GROW(0) },
            .padding = CLAY_PADDING_ALL(32),
            .childAlignment = { CLAY_ALIGN_X_CENTER, CLAY_ALIGN_Y_CENTER },
        }
    }) {
        CLAY(CLAY_ID("Panel"), {
            .backgroundColor = panel,
            .cornerRadius = CLAY_CORNER_RADIUS(18),
            .border = {
                .width = CLAY_BORDER_ALL(2),
                .color = panelBorder,
            },
            .layout = {
                .layoutDirection = CLAY_TOP_TO_BOTTOM,
                .childGap = 18,
                .padding = CLAY_PADDING_ALL(24),
                .sizing = {
                    CLAY_SIZING_FIXED(520),
                    CLAY_SIZING_FIT(0, 720)
                },
            }
        }) {
            CLAY(CLAY_ID("Header"), {
                .layout = {
                    .layoutDirection = CLAY_TOP_TO_BOTTOM,
                    .childGap = 8,
                }
            }) {
                CLAY_TEXT(CLAY_STRING("Warped"), CLAY_TEXT_CONFIG({
                    .fontId = FONT_ID_UI_40,
                    .fontSize = 40,
                    .textColor = accent,
                }));
                CLAY_TEXT(CLAY_STRING("Select a map to start playing."), CLAY_TEXT_CONFIG({
                    .fontId = FONT_ID_UI_16,
                    .fontSize = 18,
                    .textColor = muted,
                }));
            }

            if (!G.menuStatus.empty()) {
                CLAY(CLAY_ID("Status"), {
                    .backgroundColor = { 44, 49, 58, 255 },
                    .cornerRadius = CLAY_CORNER_RADIUS(12),
                    .layout = {
                        .padding = CLAY_PADDING_ALL(14),
                        .sizing = { CLAY_SIZING_GROW(0), CLAY_SIZING_FIT(0, 0) },
                    }
                }) {
                    CLAY_TEXT(ClayString(G.menuStatus), CLAY_TEXT_CONFIG({
                        .fontId = FONT_ID_UI_16,
                        .fontSize = 16,
                        .textColor = text,
                    }));
                }
            }

            CLAY(CLAY_ID("MapList"), {
                .layout = {
                    .layoutDirection = CLAY_TOP_TO_BOTTOM,
                    .childGap = 12,
                    .sizing = { CLAY_SIZING_GROW(0), CLAY_SIZING_FIT(0, 560) },
                }
            }) {
                for (int i = 0; i < (int)G.availableMaps.size(); ++i) {
                    const MapEntry& map = G.availableMaps[i];

                    CLAY(CLAY_IDI("MapButton", i), {
                        .backgroundColor = Clay_Hovered() ? buttonHover : button,
                        .cornerRadius = CLAY_CORNER_RADIUS(12),
                        .border = {
                            .width = CLAY_BORDER_ALL(1),
                            .color = Clay_Hovered() ? accent : panelBorder,
                        },
                        .layout = {
                            .padding = CLAY_PADDING_ALL(16),
                            .sizing = { CLAY_SIZING_GROW(0), CLAY_SIZING_FIXED(58) },
                            .childAlignment = { CLAY_ALIGN_X_LEFT, CLAY_ALIGN_Y_CENTER },
                        }
                    }) {
                        Clay_OnHover(HandleMapButtonInteraction, (void*)(intptr_t)i);
                        CLAY_TEXT(ClayString(map.name), CLAY_TEXT_CONFIG({
                            .fontId = FONT_ID_UI_24,
                            .fontSize = 24,
                            .textColor = text,
                        }));
                    }
                }
            }

            CLAY_TEXT(CLAY_STRING("Maps are loaded from assets/maps/*.bsp"), CLAY_TEXT_CONFIG({
                .fontId = FONT_ID_UI_16,
                .fontSize = 14,
                .textColor = muted,
            }));
        }
    }

    return Clay_EndLayout();
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
    Vector3 spawn = starts.empty() ? (Vector3){0, 0, 0} : starts[0].position;

    InitPlayer(&G.player, spawn, (Vector3){0, 0, 0}, (Vector3){0, 1, 0}, 90.0f);
    G.mapModel = Renderer_UploadBSP(bsp, G.texMgr);

    BuildMapPhysics(bsp.hulls, G.bodyInterface);
    SpawnDebugPhysObj(G.bodyInterface);
    InitJoltCharacter(&G.player, s_physics_system);
    UpdateCameraTarget(&G.player);

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
    sclay_setup();

    uint64_t clayMemorySize = Clay_MinMemorySize();
    g_clayMemory = malloc(clayMemorySize);
    Clay_Arena clayArena = Clay_CreateArenaWithCapacityAndMemory(clayMemorySize, g_clayMemory);
    Clay_Initialize(clayArena, (Clay_Dimensions){ (float)sapp_width(), (float)sapp_height() }, (Clay_ErrorHandler){ HandleClayError, nullptr });

    g_clayFonts[FONT_ID_UI_16] = sclay_add_font(kMenuFontPath);
    g_clayFonts[FONT_ID_UI_24] = g_clayFonts[FONT_ID_UI_16];
    g_clayFonts[FONT_ID_UI_40] = g_clayFonts[FONT_ID_UI_16];
    Clay_SetMeasureTextFunction(sclay_measure_text, &g_clayFonts);

    Renderer_Init();
    InitTextureManager(G.texMgr);
    Input_Init();
    Input_LockMouse(false);

    RefreshMapList();

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
    static double accumulator = 0.0;

    if (G.mode == AppMode::MainMenu) {
        if (Input_KeyPressed(WKEY_TAB)) {
            sapp_toggle_fullscreen();
        }
        if (Input_KeyPressed(WKEY_ESCAPE)) {
            sapp_request_quit();
        }

        G.requestedMapIndex = -1;
        sclay_new_frame();
        Clay_RenderCommandArray renderCommands = BuildMainMenuLayout();

        if (G.requestedMapIndex >= 0 && G.requestedMapIndex < (int)G.availableMaps.size()) {
            LoadSelectedMap(G.availableMaps[G.requestedMapIndex]);
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
            sclay_render(renderCommands, g_clayFonts);
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

    sg_pass pass = {};
    pass.action = G.gamePassAction;
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
    sclay_handle_event(ev);
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
    sclay_shutdown();
    Debug_Shutdown();
    sdtx_shutdown();
    sg_shutdown();

    free(g_clayMemory);
    g_clayMemory = nullptr;
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
