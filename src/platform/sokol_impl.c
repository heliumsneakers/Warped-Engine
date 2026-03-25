/*
 * Single translation unit that pulls in the sokol implementations.
 *
 * On macOS CMake compiles this file as Objective-C so that sokol_app can
 * talk to Cocoa.  On Linux/Windows it is plain C.
 */

#if (defined(WARPED_SOKOL_BACKEND_METAL) + defined(WARPED_SOKOL_BACKEND_D3D11) + defined(WARPED_SOKOL_BACKEND_GLCORE)) != 1
    #error "Exactly one WARPED_SOKOL_BACKEND_* define must be set by CMake."
#endif

#define SOKOL_IMPL

#if defined(WARPED_SOKOL_BACKEND_METAL)
    #define SOKOL_METAL
#elif defined(WARPED_SOKOL_BACKEND_D3D11)
    #define SOKOL_D3D11
#elif defined(WARPED_SOKOL_BACKEND_GLCORE)
    #define SOKOL_GLCORE
#endif

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
#elif defined(__APPLE__) && defined(SOKOL_METAL)
    #import <QuartzCore/QuartzCore.h>
    #import <objc/message.h>
    #import <objc/runtime.h>

    static NSTimer* g_warped_metal_timer = nil;

    static void _warped_macos_mtl_frame(void) {
        const CFTimeInterval cur_timestamp = CACurrentMediaTime();
        if (_sapp.macos.mtl.timing.timestamp > 0.0) {
            _sapp.macos.mtl.timing.frame_duration_sec = cur_timestamp - _sapp.macos.mtl.timing.timestamp;
            if (_sapp.macos.mtl.timing.frame_duration_sec <= 0.00001) {
                _sapp.macos.mtl.timing.frame_duration_sec = 1.0 / _sapp_macos_max_fps();
            } else if (_sapp.macos.mtl.timing.frame_duration_sec > _SAPP_MACOS_MTL_MAX_FRAME_DURATION_IN_SECONDS) {
                _sapp.macos.mtl.timing.frame_duration_sec = _SAPP_MACOS_MTL_MAX_FRAME_DURATION_IN_SECONDS;
            }
        }
        _sapp.macos.mtl.timing.timestamp = cur_timestamp;

        @autoreleasepool {
            _sapp_frame();
        }
        if (_sapp.quit_requested || _sapp.quit_ordered) {
            [_sapp.macos.window performClose:nil];
        }
    }

    static void _warped_metalTimerFired(id self, SEL _cmd, NSTimer* timer) {
        (void)self; (void)_cmd; (void)timer;
        _warped_macos_mtl_frame();
    }

    static void _warped_displayLinkNoop(id self, SEL _cmd, id sender) {
        (void)self; (void)_cmd; (void)sender;
    }

    void WarpedPlatform_SetSwapInterval(int interval) {
        if ([_sapp.macos.mtl.layer respondsToSelector:@selector(setDisplaySyncEnabled:)]) {
            typedef void (*warped_objc_msgsend_bool)(id, SEL, BOOL);
            ((warped_objc_msgsend_bool)objc_msgSend)(_sapp.macos.mtl.layer, @selector(setDisplaySyncEnabled:), interval != 0);
        }

        if (interval != 0) {
            return;
        }

        Class cls = [_sapp_macos_view class];
        Method dl = class_getInstanceMethod(cls, @selector(displayLinkFired:));
        Method ft = class_getInstanceMethod(cls, @selector(fallbackTimerFired:));
        if (dl) {
            method_setImplementation(dl, (IMP)_warped_displayLinkNoop);
        }
        if (ft) {
            method_setImplementation(ft, (IMP)_warped_metalTimerFired);
        }

        if (_sapp.macos.mtl.display_link) {
            _sapp.macos.mtl.display_link.paused = YES;
        }

        if (nil == g_warped_metal_timer) {
            g_warped_metal_timer = [NSTimer
                timerWithTimeInterval:0.001
                target:_sapp.macos.view
                selector:@selector(fallbackTimerFired:)
                userInfo:nil
                repeats:YES];
            g_warped_metal_timer.tolerance = 0.0;
            [[NSRunLoop currentRunLoop] addTimer:g_warped_metal_timer forMode:NSRunLoopCommonModes];
        }
    }
#else
    /* Other platforms: sokol honours sapp_desc.swap_interval, so no override
     * is needed.  Keep the symbol so linker is happy. */
    void WarpedPlatform_SetSwapInterval(int interval) { (void)interval; }
#endif
