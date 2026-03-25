#include "lightmap_compute.h"

#include "sokol_gfx.h"

#if defined(WARPED_SOKOL_BACKEND_METAL)

#import <Metal/Metal.h>

#include <cstring>
#include <string>

namespace {

void SetMetalError(std::string* error, const char* message) {
    if (error) {
        *error = message;
    }
}

id<MTLDevice> g_metalDevice = nil;

} // namespace

bool LightmapComputePlatform_Init_Impl(sg_environment* env, std::string* error) {
    g_metalDevice = MTLCreateSystemDefaultDevice();
    if (!g_metalDevice) {
        SetMetalError(error, "Failed to create a Metal device for lightmap compute baking.");
        return false;
    }
    env->metal.device = (__bridge const void*) g_metalDevice;
    return true;
}

void LightmapComputePlatform_Shutdown_Impl(void) {
    g_metalDevice = nil;
}

bool LightmapComputePlatform_ReadbackBuffer_Impl(sg_buffer buffer, size_t numBytes, void* dst, std::string* error) {
    sg_mtl_buffer_info info = sg_mtl_query_buffer_info(buffer);
    id<MTLBuffer> mtlBuffer = (__bridge id<MTLBuffer>) info.buf[info.active_slot];
    id<MTLCommandQueue> queue = (__bridge id<MTLCommandQueue>) sg_mtl_command_queue();
    if (!mtlBuffer || !queue) {
        SetMetalError(error, "Metal readback failed: missing output buffer or command queue.");
        return false;
    }

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    if (!cmd) {
        SetMetalError(error, "Metal readback failed: could not allocate a synchronization command buffer.");
        return false;
    }

    if ([mtlBuffer storageMode] == MTLStorageModeManaged) {
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit synchronizeResource:mtlBuffer];
        [blit endEncoding];
    }

    [cmd commit];
    [cmd waitUntilCompleted];
    memcpy(dst, [mtlBuffer contents], numBytes);
    return true;
}

#endif
