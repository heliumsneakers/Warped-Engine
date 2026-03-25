/*
 * Compiler-side sokol implementation TU.
 *
 * This is separate from the runtime app TU because the offline map compiler
 * only needs sokol_gfx + sokol_log and supplies its own backend bootstrap.
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

#include "sokol_gfx.h"
#include "sokol_log.h"
