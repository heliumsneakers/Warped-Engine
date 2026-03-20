#pragma once

extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;
extern int FPS_MAX;             // render-rate cap; 0 = uncapped
extern int SIM_TICK_RATE;       // fixed gameplay tick rate (Hz) — never 0
extern bool DEVMODE;
extern float deltaTime;
extern float RENDER_DISTANCE;   // far-clip plane, world units
