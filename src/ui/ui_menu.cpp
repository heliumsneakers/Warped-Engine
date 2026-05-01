// ui_menu.cpp  —  Clay UI layout and menu rendering.

#if defined(WARPED_SOKOL_BACKEND_METAL) && !defined(SOKOL_METAL)
#define SOKOL_METAL
#elif defined(WARPED_SOKOL_BACKEND_D3D11) && !defined(SOKOL_D3D11)
#define SOKOL_D3D11
#elif defined(WARPED_SOKOL_BACKEND_GLCORE) && !defined(SOKOL_GLCORE)
#define SOKOL_GLCORE
#endif

// sokol_clay.h requires: sokol_gl.h → sokol_fontstash.h → sokol_app.h → clay.h → sokol_clay.h
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_gl.h"

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#define FONTSTASH_IMPLEMENTATION
#include "fontstash.h"
#define SOKOL_FONTSTASH_IMPL
#include "sokol_fontstash.h"

#define CLAY_IMPLEMENTATION
#include "clay.h"
#define SOKOL_CLAY_IMPL
#include "renderers/sokol/sokol_clay.h"

#include "ui_menu.h"

namespace fs = std::filesystem;

struct MapDirectory {
    const char* filesystemPath;
    const char* displayPath;
    const char* sourceLabel;
};

static constexpr MapDirectory kMapDirectories[] = {
    { "../../maps", "../../maps", "maps" },
    { "../../assets/maps", "../../assets/maps", "assets" },
};
static constexpr const char* kMenuFontPath = "../../lib/clay/examples/sokol-video-demo/resources/Roboto-Regular.ttf";

static sclay_font_t         g_clayFonts[FONT_ID_COUNT] = {};
static void*                g_clayMemory               = nullptr;
static Clay_RenderCommandArray g_lastCommands          = {};

static int s_requestedMapIndex = -1;

static Clay_String ClayString(const std::string& str) {
    return Clay_String{ false, (int32_t)str.size(), str.c_str() };
}

static void HandleClayError(Clay_ErrorData errorData) {
    printf("[Clay] %.*s\n", errorData.errorText.length, errorData.errorText.chars);
}

static void HandleMapButtonInteraction(Clay_ElementId, Clay_PointerData pointerData, void* userData) {
    if (pointerData.state != CLAY_POINTER_DATA_PRESSED_THIS_FRAME) return;
    s_requestedMapIndex = (int)(intptr_t)userData;
}

static Clay_RenderCommandArray BuildMainMenuLayout(
    const std::vector<MapEntry>& maps,
    const std::string& status)
{
    const Clay_Color bg          = {  18,  20,  24, 255 };
    const Clay_Color panel       = {  32,  36,  44, 255 };
    const Clay_Color panelBorder = {  76,  84,  98, 255 };
    const Clay_Color button      = {  56,  65,  82, 255 };
    const Clay_Color buttonHover = {  84,  98, 126, 255 };
    const Clay_Color text        = { 235, 238, 244, 255 };
    const Clay_Color muted       = { 164, 171, 184, 255 };
    const Clay_Color accent      = { 123, 196, 255, 255 };

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

            if (!status.empty()) {
                CLAY(CLAY_ID("Status"), {
                    .backgroundColor = { 44, 49, 58, 255 },
                    .cornerRadius = CLAY_CORNER_RADIUS(12),
                    .layout = {
                        .padding = CLAY_PADDING_ALL(14),
                        .sizing = { CLAY_SIZING_GROW(0), CLAY_SIZING_FIT(0, 0) },
                    }
                }) {
                    CLAY_TEXT(ClayString(status), CLAY_TEXT_CONFIG({
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
                for (int i = 0; i < (int)maps.size(); ++i) {
                    const MapEntry& map = maps[i];
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
                        CLAY_TEXT(ClayString(map.displayName), CLAY_TEXT_CONFIG({
                            .fontId = FONT_ID_UI_24,
                            .fontSize = 24,
                            .textColor = text,
                        }));
                    }
                }
            }

            CLAY_TEXT(CLAY_STRING("Maps are loaded from maps/*.bsp and assets/maps/*.bsp"), CLAY_TEXT_CONFIG({
                .fontId = FONT_ID_UI_16,
                .fontSize = 14,
                .textColor = muted,
            }));
        }
    }

    return Clay_EndLayout();
}

// ---------------------------------------------------------------------------

void UI_Init(int width, int height) {
    sclay_setup();

    uint64_t clayMemorySize = Clay_MinMemorySize();
    g_clayMemory = malloc(clayMemorySize);
    Clay_Arena clayArena = Clay_CreateArenaWithCapacityAndMemory(clayMemorySize, g_clayMemory);
    Clay_Initialize(clayArena,
        (Clay_Dimensions){ (float)width, (float)height },
        (Clay_ErrorHandler){ HandleClayError, nullptr });

    g_clayFonts[FONT_ID_UI_16] = sclay_add_font(kMenuFontPath);
    g_clayFonts[FONT_ID_UI_24] = g_clayFonts[FONT_ID_UI_16];
    g_clayFonts[FONT_ID_UI_40] = g_clayFonts[FONT_ID_UI_16];
    Clay_SetMeasureTextFunction(sclay_measure_text, &g_clayFonts);
}

void UI_Shutdown() {
    sclay_shutdown();
    free(g_clayMemory);
    g_clayMemory = nullptr;
}

void UI_NewFrame() {
    s_requestedMapIndex = -1;
    sclay_new_frame();
}

void UI_HandleEvent(const sapp_event* ev) {
    sclay_handle_event(ev);
}

void UI_RefreshMapList(std::vector<MapEntry>& maps, std::string& status) {
    struct DiscoveredMap {
        std::string name;
        std::string bspPath;
        std::string sourceLabel;
    };

    maps.clear();
    status.clear();

    std::vector<DiscoveredMap> discoveredMaps;
    discoveredMaps.reserve(16);

    bool foundAnyDirectory = false;
    for (const MapDirectory& dir : kMapDirectories) {
        std::error_code ec;
        if (!fs::exists(dir.filesystemPath, ec) || ec) {
            continue;
        }

        foundAnyDirectory = true;

        fs::directory_iterator iter(dir.filesystemPath, ec);
        if (ec) {
            continue;
        }

        for (const fs::directory_entry& entry : iter) {
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ".bsp") continue;

            discoveredMaps.push_back({
                entry.path().stem().string(),
                entry.path().string(),
                dir.sourceLabel,
            });
        }
    }

    if (!foundAnyDirectory) {
        status = "Map directories not found: maps, assets/maps";
        return;
    }

    std::unordered_map<std::string, int> nameCounts;
    for (const DiscoveredMap& map : discoveredMaps) {
        ++nameCounts[map.name];
    }

    for (const DiscoveredMap& map : discoveredMaps) {
        const bool duplicateName = nameCounts[map.name] > 1;
        maps.push_back({
            map.name,
            duplicateName ? map.name + " (" + map.sourceLabel + ")" : map.name,
            map.bspPath,
        });
    }

    std::sort(maps.begin(), maps.end(), [](const MapEntry& a, const MapEntry& b) {
        if (a.name != b.name) return a.name < b.name;
        return a.bspPath < b.bspPath;
    });

    if (maps.empty()) {
        status = "No compiled .bsp maps found in maps/ or assets/maps.";
    }
}

int UI_UpdateMenu(const std::vector<MapEntry>& maps, const std::string& status) {
    g_lastCommands = BuildMainMenuLayout(maps, status);
    return s_requestedMapIndex;
}

void UI_RenderMenu() {
    sclay_render(g_lastCommands, g_clayFonts);
}
