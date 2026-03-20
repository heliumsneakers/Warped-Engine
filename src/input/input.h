// input.h  —  thin event accumulator over sokol_app.
// Replaces IsKeyDown / IsKeyPressed / GetMouseDelta from raylib.
#pragma once

#include "sokol_app.h"

// Raylib-style aliases so call-sites stay readable
enum WKey {
    WKEY_W          = SAPP_KEYCODE_W,
    WKEY_A          = SAPP_KEYCODE_A,
    WKEY_S          = SAPP_KEYCODE_S,
    WKEY_D          = SAPP_KEYCODE_D,
    WKEY_SPACE      = SAPP_KEYCODE_SPACE,
    WKEY_LEFT_SHIFT = SAPP_KEYCODE_LEFT_SHIFT,
    WKEY_TAB        = SAPP_KEYCODE_TAB,
    WKEY_M          = SAPP_KEYCODE_M,
    WKEY_ESCAPE     = SAPP_KEYCODE_ESCAPE,
};

void  Input_Init(void);
void  Input_HandleEvent(const sapp_event* ev);   // call from sokol event_cb
void  Input_EndFrame(void);                      // clear "pressed" edges + mouse delta

bool  Input_KeyDown(int key);
bool  Input_KeyPressed(int key);                 // edge-triggered this frame

float Input_MouseDeltaX(void);
float Input_MouseDeltaY(void);

void  Input_LockMouse(bool lock);
bool  Input_MouseLocked(void);
