#include "parameters.h"

int SCREEN_WIDTH = 1280;
int SCREEN_HEIGHT = 720;

// Render-rate cap.  0 = uncapped (run as fast as the loop can drive).
int FPS_MAX = 300;

// Gameplay simulation tick rate.  Movement constants were tuned with the
// raylib build running SetTargetFPS(144) + fixed deltaTime step per frame,
// so 144 here reproduces that feel exactly.  Do not set to 0.
int SIM_TICK_RATE = 144;

bool DEVMODE = false;
float deltaTime = 1.0f / 60.0f;
float RENDER_DISTANCE = 4096.0f;
