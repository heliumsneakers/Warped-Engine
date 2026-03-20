// debug_draw.h  —  immediate-mode helpers on top of sokol_gl.
#pragma once

#include "../math/wmath.h"

void Debug_Init(void);
void Debug_Shutdown(void);

void Debug_NewFrame(void);
void Debug_SetCamera(const Matrix& proj, const Matrix& view);
void Debug_Flush(void);                            // call inside sg_begin_pass

void Debug_Line(Vector3 a, Vector3 b, Color c);
void Debug_WireBox(Vector3 center, Vector3 halfExt, Color c);
void Debug_Sphere(Vector3 center, float radius, Color c, int rings = 8, int slices = 12);
void Debug_Point(Vector3 p, float size, Color c);
