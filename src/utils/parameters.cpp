#include "parameters.h"

int SCREEN_WIDTH = 1280;
int SCREEN_HEIGHT = 720;

// Render-rate cap.  0 = uncapped (run as fast as the loop can drive).
int FPS_MAX = 300;

// Gameplay simulation tick rate.
int SIM_TICK_RATE = 144;

bool DEVMODE = false;
float deltaTime = 1.0f / 144.0f; // denominator must match tick rate so we dont simulate faster than we update.
float RENDER_DISTANCE = 32768.0f;
