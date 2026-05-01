// ui_menu.h  —  Clay UI menu interface.
#pragma once
#include <string>
#include <vector>

struct sapp_event;

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
    std::string displayName;
    std::string bspPath;
};

void UI_Init(int width, int height);
void UI_Shutdown();
void UI_NewFrame();
void UI_HandleEvent(const sapp_event* ev);
void UI_RefreshMapList(std::vector<MapEntry>& maps, std::string& status);
int  UI_UpdateMenu(const std::vector<MapEntry>& maps, const std::string& status);
void UI_RenderMenu();
