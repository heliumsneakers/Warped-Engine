/*
 * Single translation unit that pulls in the sokol implementations.
 *
 * On macOS CMake compiles this file as Objective-C so that sokol_app can
 * talk to Cocoa.  On Linux/Windows it is plain C.
 */

#define SOKOL_IMPL
#define SOKOL_GLCORE        /* desktop GL 3.3 backend on every platform */

#if defined(_WIN32)
    #define SOKOL_WIN32_FORCE_MAIN   /* use int main() instead of WinMain */
#endif

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_log.h"
#include "sokol_glue.h"
#include "sokol_gl.h"
#include "sokol_debugtext.h"

/* ------------------------------------------------------------------------- */
/*  Platform overrides                                                       */
/* ------------------------------------------------------------------------- */
#include "platform.h"

#if defined(__APPLE__) && defined(SOKOL_GLCORE)
    /*
     * sokol_app's macOS/GL backend has two hard limits we need to break:
     *
     *   1. prepareOpenGL hard-codes NSOpenGLContextParameterSwapInterval = 1
     *      (and _sapp_def coerces desc.swap_interval 0→1) → vsync always on.
     *
     *   2. The frame timer calls [view setNeedsDisplay:YES] (async, coalesced
     *      by AppKit's display cycle) instead of driving the frame
     *      synchronously, and runs in NSDefaultRunLoopMode (stalls during
     *      tracking loops).
     *
     *   Together these cap us at display refresh AND let ProMotion
     *   opportunistically throttle to 48-80 Hz when content looks "static".
     *
     *   We fix both from outside the submodule by (a) overriding the GL swap
     *   interval after init, and (b) swapping the timerFired: IMP for one that
     *   calls _sapp_macos_frame() directly.  _sapp_macos_frame and
     *   _sapp_macos_view are visible here because sokol_app.h is included
     *   with SOKOL_IMPL above.
     */
    #import <AppKit/AppKit.h>
    #import <objc/runtime.h>

    static void _warped_timerFired(id self, SEL _cmd, id sender) {
        (void)self; (void)_cmd; (void)sender;
        /* Context is already current after prepareOpenGL; drive the frame
         * synchronously so AppKit display-coalescing can't cap us. */
        NSOpenGLContext* ctx = [_sapp.macos.view openGLContext];
        [ctx makeCurrentContext];
        _sapp_macos_frame();
    }

    void WarpedPlatform_SetSwapInterval(int interval) {
        /* (a) disable GL vsync */
        NSOpenGLContext* ctx = [NSOpenGLContext currentContext];
        if (ctx) {
            GLint v = (GLint)interval;
            [ctx setValues:&v forParameter:NSOpenGLContextParameterSwapInterval];
        }
        /* (b) when running uncapped, replace the async display request with a
         *     synchronous frame call so the timer-driven loop actually
         *     outruns display refresh. */
        if (interval == 0) {
            Class cls = [_sapp_macos_view class];
            SEL   sel = @selector(timerFired:);
            Method m  = class_getInstanceMethod(cls, sel);
            if (m) {
                method_setImplementation(m, (IMP)_warped_timerFired);
            }
        }
    }
#else
    /* Other platforms: sokol honours sapp_desc.swap_interval, so no override
     * is needed.  Keep the symbol so linker is happy. */
    void WarpedPlatform_SetSwapInterval(int interval) { (void)interval; }
#endif
