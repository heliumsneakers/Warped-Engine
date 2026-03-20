# Raylib → Sokol Migration Plan

## Goal

Replace every raylib subsystem (windowing, input, GPU resource management,
mesh upload, texturing, debug text, debug lines) with the equivalent
sokol header and keep the existing Jolt‑based physics + TrenchBroom `.map`
pipeline working unchanged.

The coordinate system (Y‑up, right‑handed, same as raylib / GL 3.3) and the
brush‑parser math are preserved 1‑to‑1, so collision hulls and the render mesh
continue to line up with no additional swizzling.

---

## 1. Layering / file‑responsibility after migration

| Concern                                   | Old (raylib)                        | New (sokol)                                                      | File(s)                                   |
|-------------------------------------------|-------------------------------------|------------------------------------------------------------------|-------------------------------------------|
| Windowing / swap‑chain / main loop         | `InitWindow`, `WindowShouldClose`   | `sokol_app.h` – `init_cb / frame_cb / cleanup_cb / event_cb`     | `src/main.cpp`                            |
| Keyboard / mouse input                    | `IsKeyDown`, `GetMouseDelta`        | `sokol_app.h` events → buffered in `src/input/input.cpp`         | `src/input/input.{h,cpp}`                 |
| 3‑D textured mesh rendering               | `Model`, `DrawModel`                | `sokol_gfx.h` pipeline + vertex/index buffers + GLSL 330 shader  | `src/render/renderer.{h,cpp}`             |
| Shaders                                   | raylib default                      | Hand‑rolled GLSL 330 (GL 3.3 backend)                            | `src/render/shaders.h`                    |
| Texture loading                           | `LoadImage` / `LoadTextureFromImage`| `stb_image.h` → `sg_make_image`                                  | `src/render/renderer.cpp`                 |
| Debug text (pos / vel / FPS)              | `DrawText`, `DrawFPS`               | `sokol_debugtext.h`                                              | `src/main.cpp`, `src/player/player.cpp`   |
| Debug 3‑D primitives (AABB, arrows, sphere)| `DrawCubeWires`, `DrawLine3D`, …   | `sokol_gl.h`                                                     | `src/render/debug_draw.{h,cpp}`           |
| Vector / matrix math                      | `raymath.h`                         | Small stand‑alone `src/math/wmath.h` (same struct layout)        | Everywhere that used `Vector3`/`Matrix`   |

---

## 2. Math abstraction – `src/math/wmath.h`

Raylib types are used all over the geometry code (`Vector3`, `Vector2`,
`Matrix`, `Vector3DotProduct`, …).  We provide a drop‑in header that:

* Re‑declares `Vector2`, `Vector3`, `Matrix` with the **exact same memory
  layout** so `collision_data.cpp` / `map_parser.cpp` keep compiling.
* Re‑implements only the helpers actually called:
  `Vector3Add/Subtract/Scale/DotProduct/CrossProduct/Normalize/Length/Zero`,
  `MatrixIdentity/Multiply/Perspective/LookAt/Translate` and the
  `DEG2RAD` constant.
* Provides row‑major → column‑major conversion for the GL uniform
  matrix (`MatrixToFloat16`).

Because the struct layouts are identical, **no change** is required in
`BuildBrushGeometry`, plane intersection, or Jolt hull building.

---

## 3. Rendering pipeline – `src/render/renderer.{h,cpp}`

### 3.1  Vertex format

```
struct MapVertex {
    float x, y, z;   // position  (Y‑up, already converted from TB)
    float nx,ny,nz;  // normal
    float u, v;      // texcoord (0‑1, wrap/repeat)
};
```

### 3.2  GL 3.3 shader (`src/render/shaders.h`)

Single textured + N·L diffuse program compiled through
`sg_make_shader` with GLSL 330 sources.  Uniform block:

```
layout(std140) uniform vs_params {
    mat4 mvp;
    mat4 model;
};
```

### 3.3  `MapModel` container

Replaces `Model`.  One `SubMesh` per texture (same bucketing the old
`MapToMesh` already does):

```
struct SubMesh {
    sg_buffer   vbuf;
    sg_buffer   ibuf;
    sg_image    tex;
    sg_sampler  smp;
    int         index_count;
};
struct MapModel { std::vector<SubMesh> meshes; };
```

### 3.4  Public API

```
void        Renderer_Init();                    // pipeline + default sampler
MapModel    Renderer_UploadMap(const Map&, TextureManager&);
void        Renderer_DrawMap(const MapModel&, const Matrix& viewProj);
void        Renderer_Shutdown(MapModel&);
```

---

## 4. `map_parser.{h,cpp}` changes

* Replace `#include "raylib.h" / "raymath.h"` with `math/wmath.h`.
* `TextureManager` now stores `sg_image` + `{width,height}` (needed for the
  Quake UV divide).
* Remove `Model MapToMesh(...)` – replaced by `BuildMapGeometry(...)` which
  returns CPU‑side vertex / index buffers per texture (same algorithm, same
  trigger‑brush `DEVMODE` skip).  GPU upload is delegated to
  `Renderer_UploadMap`.
* Image loading goes through `stb_image` (RGBA, non‑POT allowed – GL 3.3
  supports `GL_REPEAT` on NPOT).

---

## 5. `collision_data.{h,cpp}` & `physics.{h,cpp}`

Only the include line changes (`raylib.h` → `math/wmath.h`).  The vertex
point‑cloud feed into `JPH::ConvexHullShapeSettings` is untouched.

---

## 6. `player.{h,cpp}`

* `Camera3D` replaced by our own POD `Camera` (`position/target/up/fovy`).
* `IsKeyDown(KEY_*)` / `GetMouseDelta` → `Input_KeyDown(...)` /
  `Input_MouseDelta()` backed by the sokol event callback.
* `DisableCursor / EnableCursor` → `sapp_lock_mouse(true/false)`.
* Debug draw functions pipe through `debug_draw.h` (sokol_gl) and
  `sokol_debugtext.h`.

---

## 7. `main.cpp` (sokol_app entry)

Classic sokol entry pattern:

```
sokol_main()  →  sapp_desc{ init_cb, frame_cb, cleanup_cb, event_cb }
```

* `init_cb`  – sets up `sg_*`, debug‑text, sokol‑gl, parses map, uploads
  geometry, boots Jolt, spawns character.
* `frame_cb` – steps physics, updates player, builds view/proj, issues
  `sg_begin_pass → draw map → sokol_gl debug → debug‑text → sg_end_pass`.
* `event_cb` – forwards to `Input_HandleEvent`.
* `cleanup_cb` – mirrors the old shutdown order.

Fullscreen toggle (`TAB`) → `sapp_toggle_fullscreen()`.

---

## 8. `CMakeLists.txt`

* Drop raylib `find_package / FetchContent`.
* Add `lib/sokol` to `include_directories`.
* Add `lib/stb` for `stb_image.h` (single‑header vendored copy).
* **Cross‑platform sokol implementation TU** `src/platform/sokol_impl.{c,m}`:
  * macOS → compile as `sokol_impl.m` (Obj‑C, `-x objective-c`,
    `-DSOKOL_GLCORE`, link `Cocoa/QuartzCore/OpenGL`).
  * Linux → `sokol_impl.c` (`-DSOKOL_GLCORE`, link `X11 Xi Xcursor GL dl pthread m`).
  * Windows → `sokol_impl.c` (`-DSOKOL_GLCORE`, link `opengl32 gdi32`).
* Keep the existing Clang / libc++ / ASan switches intact.

---

## 9. Coordinate‑system sanity check

* TrenchBroom export = Z‑up.
* `ConvertTBtoRaylib` (now renamed internally to `ConvertTBToWorld`) already
  swizzles into Y‑up.  Unchanged.
* sokol_gfx + GL 3.3 uses right‑handed Y‑up when we supply our own MVP, so the
  existing conversion is already correct – **no additional axis flip is
  introduced**.

---

## 10. Implementation order

1. `src/math/wmath.h`                         – compiles on its own.
2. `src/input/input.{h,cpp}`                  – depends on sokol_app.
3. `src/render/shaders.h`, `renderer.{h,cpp}` – depends on sokol_gfx + wmath.
4. `src/render/debug_draw.{h,cpp}`            – depends on sokol_gl.
5. Patch `map_parser`, `collision_data`, `physics`, `player`.
6. Rewrite `main.cpp`.
7. `src/platform/sokol_impl.c` / `.m`.
8. `CMakeLists.txt`.

---
