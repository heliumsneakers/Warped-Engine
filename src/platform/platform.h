// platform.h  —  small hooks we need that sokol_app doesn't expose.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Override the swapchain vsync setting after sokol_app has done its own setup.
// On macOS/GL sokol hard-codes NSOpenGLContextParameterSwapInterval = 1 and
// also coerces desc.swap_interval 0→1, so without this we are locked to
// display refresh (ProMotion then opportunistically throttles to 60–80 Hz).
// 0 = immediate (no vsync), 1 = sync to display.
void WarpedPlatform_SetSwapInterval(int interval);

#ifdef __cplusplus
}
#endif
