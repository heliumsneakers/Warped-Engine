#include "input.h"
#include <string.h>

static bool  s_key_down   [SAPP_MAX_KEYCODES];
static bool  s_key_pressed[SAPP_MAX_KEYCODES];
static float s_mouse_dx = 0.0f;
static float s_mouse_dy = 0.0f;

void Input_Init(void) {
    memset(s_key_down,    0, sizeof(s_key_down));
    memset(s_key_pressed, 0, sizeof(s_key_pressed));
    s_mouse_dx = s_mouse_dy = 0.0f;
}

void Input_HandleEvent(const sapp_event* ev) {
    switch (ev->type) {
        case SAPP_EVENTTYPE_KEY_DOWN:
            if ((int)ev->key_code >= 0 && (int)ev->key_code < SAPP_MAX_KEYCODES) {
                if (!s_key_down[ev->key_code]) {
                    s_key_pressed[ev->key_code] = true;
                }
                s_key_down[ev->key_code] = true;
            }
            break;
        case SAPP_EVENTTYPE_KEY_UP:
            if ((int)ev->key_code >= 0 && (int)ev->key_code < SAPP_MAX_KEYCODES) {
                s_key_down[ev->key_code] = false;
            }
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE:
            s_mouse_dx += ev->mouse_dx;
            s_mouse_dy += ev->mouse_dy;
            break;
        case SAPP_EVENTTYPE_UNFOCUSED:
            // drop all held keys so we don't get stuck
            memset(s_key_down, 0, sizeof(s_key_down));
            break;
        default: break;
    }
}

void Input_EndFrame(void) {
    memset(s_key_pressed, 0, sizeof(s_key_pressed));
    s_mouse_dx = 0.0f;
    s_mouse_dy = 0.0f;
}

bool Input_KeyDown(int key)    { return (key >= 0 && key < SAPP_MAX_KEYCODES) && s_key_down[key]; }
bool Input_KeyPressed(int key) { return (key >= 0 && key < SAPP_MAX_KEYCODES) && s_key_pressed[key]; }

float Input_MouseDeltaX(void)  { return s_mouse_dx; }
float Input_MouseDeltaY(void)  { return s_mouse_dy; }

void Input_LockMouse(bool lock) { sapp_lock_mouse(lock); }
bool Input_MouseLocked(void)    { return sapp_mouse_locked(); }
