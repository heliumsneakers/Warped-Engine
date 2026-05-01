#include "lightmap_compute.h"
#include "lightmap_constants.h"

#include "sokol_gfx.h"
#include "sokol_log.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>

#ifdef WARPED_LIGHTMAP_USE_EMBREE
    #include <embree4/rtcore.h>
    #include <embree4/rtcore_builder.h>
#endif

#if defined(WARPED_SOKOL_BACKEND_METAL)
    #define SOKOL_METAL
#elif defined(WARPED_SOKOL_BACKEND_D3D11)
    #define SOKOL_D3D11
#elif defined(WARPED_SOKOL_BACKEND_GLCORE)
    #define SOKOL_GLCORE
#endif

#include "shaders/generated/lightmap_bake.compute.h"

#if defined(_WIN32)
    #include <d3d11.h>
#elif defined(__linux__)
    #ifndef GL_GLEXT_PROTOTYPES
        #define GL_GLEXT_PROTOTYPES
    #endif
    #include <X11/Xlib.h>
    #include <GL/gl.h>
    #include <GL/glx.h>
#endif

#if defined(__APPLE__)
bool LightmapComputePlatform_Init_Impl(sg_environment* env, std::string* error);
void LightmapComputePlatform_Shutdown_Impl(void);
bool LightmapComputePlatform_ReadbackBuffer_Impl(sg_buffer buffer, size_t numBytes, void* dst, std::string* error);
#endif

namespace {

constexpr int kWorkgroupSize = 8;
constexpr int kDispatchBatchSize = 32;
constexpr size_t kUniformUploadAlign = 256;
constexpr float kShadowBias = 0.03125f;
constexpr float kRayEps = 1e-4f;

using GpuPointLight = warped_lightmap_bake_point_light_t;
using GpuSurfaceEmitter = warped_lightmap_bake_surface_emitter_t;
using GpuSurfaceEmitterSample = warped_lightmap_bake_surface_emitter_sample_t;
using GpuSurfaceEmitterIndex = warped_lightmap_bake_surface_emitter_index_t;
using GpuOccluderTri = warped_lightmap_bake_occluder_tri_t;
using GpuBrushSolid = warped_lightmap_bake_brush_solid_t;
using GpuSolidPlane = warped_lightmap_bake_solid_plane_t;
using GpuRepairSourcePoly = warped_lightmap_bake_repair_source_poly_t;
using GpuRepairSourceNeighbor = warped_lightmap_bake_repair_source_neighbor_t;
using GpuPhongSourcePoly = warped_lightmap_bake_phong_source_poly_t;
using GpuPhongNeighbor = warped_lightmap_bake_phong_neighbor_t;
using GpuFaceParams = warped_lightmap_bake_cs_face_params_t;
using GpuBvhNode = warped_lightmap_bake_bvh_node_t;
using GpuBvhTriIndex = warped_lightmap_bake_bvh_tri_index_t;
using GpuBakedPixel = warped_lightmap_bake_baked_pixel_t;

#ifdef WARPED_LIGHTMAP_USE_EMBREE
struct EmbreeGpuBvhNode {
    bool leaf = false;
    unsigned int childCount = 0;
    EmbreeGpuBvhNode* children[2] = {};
    AABB childBounds[2] = {};
    AABB bounds = AABBInvalid();
    size_t leafFirst = 0;
    size_t leafCount = 0;
};

struct EmbreeGpuBvhBuildContext {
    const std::vector<LightmapComputeOccluderTri>* occluders = nullptr;
    std::vector<uint32_t> triIndices;
    std::mutex triIndexMutex;
};

Vector3 RtcBoundsMin(const RTCBounds& bounds) {
    return Vector3{bounds.lower_x, bounds.lower_y, bounds.lower_z};
}

Vector3 RtcBoundsMax(const RTCBounds& bounds) {
    return Vector3{bounds.upper_x, bounds.upper_y, bounds.upper_z};
}

AABB AABBFromRtcBounds(const RTCBounds& bounds) {
    AABB result = AABBInvalid();
    AABBExtend(&result, RtcBoundsMin(bounds));
    AABBExtend(&result, RtcBoundsMax(bounds));
    return result;
}

AABB AABBFromBuildPrimitive(const RTCBuildPrimitive& prim) {
    AABB result = AABBInvalid();
    AABBExtend(&result, Vector3{prim.lower_x, prim.lower_y, prim.lower_z});
    AABBExtend(&result, Vector3{prim.upper_x, prim.upper_y, prim.upper_z});
    return result;
}

AABB MergeAABB(AABB a, const AABB& b) {
    AABBExtend(&a, b.min);
    AABBExtend(&a, b.max);
    return a;
}

void* EmbreeGpuBvhCreateNode(RTCThreadLocalAllocator allocator, unsigned int childCount, void* userPtr) {
    (void) userPtr;
    void* ptr = rtcThreadLocalAlloc(allocator, sizeof(EmbreeGpuBvhNode), 16);
    EmbreeGpuBvhNode* node = new (ptr) EmbreeGpuBvhNode();
    node->leaf = false;
    node->childCount = std::min(childCount, 2u);
    return node;
}

void EmbreeGpuBvhSetNodeChildren(void* nodePtr, void** children, unsigned int childCount, void* userPtr) {
    (void) userPtr;
    EmbreeGpuBvhNode* node = (EmbreeGpuBvhNode*) nodePtr;
    node->childCount = std::min(childCount, 2u);
    for (unsigned int i = 0; i < node->childCount; ++i) {
        node->children[i] = (EmbreeGpuBvhNode*) children[i];
    }
}

void EmbreeGpuBvhSetNodeBounds(void* nodePtr, const RTCBounds** bounds, unsigned int childCount, void* userPtr) {
    (void) userPtr;
    EmbreeGpuBvhNode* node = (EmbreeGpuBvhNode*) nodePtr;
    const unsigned int storedChildCount = std::min(childCount, 2u);
    for (unsigned int i = 0; i < storedChildCount; ++i) {
        node->childBounds[i] = AABBFromRtcBounds(*bounds[i]);
    }
}

void* EmbreeGpuBvhCreateLeaf(RTCThreadLocalAllocator allocator,
                             const RTCBuildPrimitive* primitives,
                             size_t primitiveCount,
                             void* userPtr) {
    EmbreeGpuBvhBuildContext* context = (EmbreeGpuBvhBuildContext*) userPtr;
    void* ptr = rtcThreadLocalAlloc(allocator, sizeof(EmbreeGpuBvhNode), 16);
    EmbreeGpuBvhNode* node = new (ptr) EmbreeGpuBvhNode();
    node->leaf = true;
    node->bounds = AABBInvalid();

    std::lock_guard<std::mutex> lock(context->triIndexMutex);
    node->leafFirst = context->triIndices.size();
    node->leafCount = primitiveCount;
    for (size_t i = 0; i < primitiveCount; ++i) {
        context->triIndices.push_back(primitives[i].primID);
        node->bounds = MergeAABB(node->bounds, AABBFromBuildPrimitive(primitives[i]));
    }
    return node;
}

int FlattenEmbreeGpuBvhNode(const EmbreeGpuBvhNode* node, LightmapComputeBvh* outBvh) {
    if (!node) {
        return -1;
    }

    if (node->leaf || node->childCount == 0) {
        const int nodeIndex = (int) outBvh->nodes.size();
        LightmapComputeBvhNode flat{};
        flat.boundsMin = node->bounds.min;
        flat.boundsMax = node->bounds.max;
        flat.leftFirst = (int) node->leafFirst;
        flat.rightCount = (int) node->leafCount;
        outBvh->nodes.push_back(flat);
        return nodeIndex;
    }

    if (node->childCount == 1) {
        return FlattenEmbreeGpuBvhNode(node->children[0], outBvh);
    }

    const int nodeIndex = (int) outBvh->nodes.size();
    outBvh->nodes.push_back(LightmapComputeBvhNode{});

    const int leftIndex = FlattenEmbreeGpuBvhNode(node->children[0], outBvh);
    const int rightIndex = FlattenEmbreeGpuBvhNode(node->children[1], outBvh);
    if ((leftIndex < 0) || (rightIndex < 0)) {
        return -1;
    }

    AABB bounds = AABBInvalid();
    bounds = MergeAABB(bounds, AABB{outBvh->nodes[(size_t)leftIndex].boundsMin, outBvh->nodes[(size_t)leftIndex].boundsMax});
    bounds = MergeAABB(bounds, AABB{outBvh->nodes[(size_t)rightIndex].boundsMin, outBvh->nodes[(size_t)rightIndex].boundsMax});

    LightmapComputeBvhNode& flat = outBvh->nodes[(size_t)nodeIndex];
    flat.boundsMin = bounds.min;
    flat.boundsMax = bounds.max;
    flat.leftFirst = leftIndex;
    flat.rightCount = -rightIndex;
    return nodeIndex;
}
#endif

const char* BackendName(sg_backend backend) {
    switch (backend) {
        case SG_BACKEND_GLCORE: return "GLCORE";
        case SG_BACKEND_D3D11: return "D3D11";
        case SG_BACKEND_METAL_MACOS: return "METAL_MACOS";
        default: return "UNKNOWN";
    }
}

const char* ResourceStateName(sg_resource_state state) {
    switch (state) {
        case SG_RESOURCESTATE_INITIAL: return "INITIAL";
        case SG_RESOURCESTATE_ALLOC: return "ALLOC";
        case SG_RESOURCESTATE_VALID: return "VALID";
        case SG_RESOURCESTATE_FAILED: return "FAILED";
        case SG_RESOURCESTATE_INVALID: return "INVALID";
        default: return "UNKNOWN";
    }
}

bool BackendMatchesBuild(sg_backend backend) {
#if defined(WARPED_SOKOL_BACKEND_METAL)
    return backend == SG_BACKEND_METAL_MACOS;
#elif defined(WARPED_SOKOL_BACKEND_D3D11)
    return backend == SG_BACKEND_D3D11;
#elif defined(WARPED_SOKOL_BACKEND_GLCORE)
    return backend == SG_BACKEND_GLCORE;
#else
    (void) backend;
    return false;
#endif
}

void SetError(std::string* error, const char* fmt, ...) {
    if (!error) {
        return;
    }
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    *error = buffer;
}

template<typename T>
sg_range ByteRange(const std::vector<T>& items) {
    sg_range range{};
    range.ptr = items.data();
    range.size = items.size() * sizeof(T);
    return range;
}

sg_range ByteRange(const void* ptr, size_t size) {
    sg_range range{};
    range.ptr = ptr;
    range.size = size;
    return range;
}

void CopyVec3(float (&dst)[3], const Vector3& src) {
    dst[0] = src.x;
    dst[1] = src.y;
    dst[2] = src.z;
}

size_t AlignUp(size_t value, size_t align) {
    return (value + (align - 1)) & ~(align - 1);
}

bool LightmapComputePlatform_Init(sg_environment* env, std::string* error);
void LightmapComputePlatform_Shutdown(void);
bool LightmapComputePlatform_ReadbackBuffer(sg_buffer buffer, size_t numBytes, void* dst, std::string* error);

#if defined(__APPLE__)

bool LightmapComputePlatform_Init(sg_environment* env, std::string* error) {
    return LightmapComputePlatform_Init_Impl(env, error);
}

void LightmapComputePlatform_Shutdown(void) {
    LightmapComputePlatform_Shutdown_Impl();
}

bool LightmapComputePlatform_ReadbackBuffer(sg_buffer buffer, size_t numBytes, void* dst, std::string* error) {
    return LightmapComputePlatform_ReadbackBuffer_Impl(buffer, numBytes, dst, error);
}

#else

#if defined(_WIN32)
static ID3D11Device* g_d3d11Device = nullptr;
static ID3D11DeviceContext* g_d3d11Context = nullptr;

bool LightmapComputePlatform_Init(sg_environment* env, std::string* error) {
    D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;
    const D3D_FEATURE_LEVEL requested[] = {
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0,
    };
    UINT flags = 0;
    HRESULT hr = D3D11CreateDevice(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        flags,
        requested,
        ARRAYSIZE(requested),
        D3D11_SDK_VERSION,
        &g_d3d11Device,
        &featureLevel,
        &g_d3d11Context);
    if (FAILED(hr) || !g_d3d11Device || !g_d3d11Context) {
        SetError(error, "Failed to create D3D11 device for lightmap compute (hr=0x%08x).", (unsigned) hr);
        return false;
    }
    env->d3d11.device = g_d3d11Device;
    env->d3d11.device_context = g_d3d11Context;
    return true;
}

void LightmapComputePlatform_Shutdown(void) {
    if (g_d3d11Context) {
        g_d3d11Context->Release();
        g_d3d11Context = nullptr;
    }
    if (g_d3d11Device) {
        g_d3d11Device->Release();
        g_d3d11Device = nullptr;
    }
}

bool LightmapComputePlatform_ReadbackBuffer(sg_buffer buffer, size_t numBytes, void* dst, std::string* error) {
    sg_d3d11_buffer_info info = sg_d3d11_query_buffer_info(buffer);
    ID3D11Buffer* src = (ID3D11Buffer*) info.buf;
    if (!src || !g_d3d11Device || !g_d3d11Context) {
        SetError(error, "D3D11 readback failed: missing source buffer or device context.");
        return false;
    }

    D3D11_BUFFER_DESC desc{};
    src->GetDesc(&desc);
    desc.BindFlags = 0;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    desc.Usage = D3D11_USAGE_STAGING;
    desc.MiscFlags = 0;
    desc.StructureByteStride = 0;

    ID3D11Buffer* staging = nullptr;
    HRESULT hr = g_d3d11Device->CreateBuffer(&desc, nullptr, &staging);
    if (FAILED(hr) || !staging) {
        SetError(error, "Failed to create D3D11 staging buffer for readback (hr=0x%08x).", (unsigned) hr);
        return false;
    }

    g_d3d11Context->CopyResource(staging, src);
    g_d3d11Context->Flush();

    D3D11_MAPPED_SUBRESOURCE mapped{};
    hr = g_d3d11Context->Map(staging, 0, D3D11_MAP_READ, 0, &mapped);
    if (FAILED(hr)) {
        staging->Release();
        SetError(error, "Failed to map D3D11 staging buffer for readback (hr=0x%08x).", (unsigned) hr);
        return false;
    }

    memcpy(dst, mapped.pData, numBytes);
    g_d3d11Context->Unmap(staging, 0);
    staging->Release();
    return true;
}

#elif defined(__linux__)
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

static Display* g_x11Display = nullptr;
static GLXContext g_glContext = nullptr;
static GLXPbuffer g_glPbuffer = 0;

bool LightmapComputePlatform_Init(sg_environment* env, std::string* error) {
    (void) env;

    g_x11Display = XOpenDisplay(nullptr);
    if (!g_x11Display) {
        SetError(error, "Failed to open X11 display for GL compute lightmap baking.");
        return false;
    }

    const int screen = DefaultScreen(g_x11Display);
    const int fbAttribs[] = {
        GLX_X_RENDERABLE, True,
        GLX_DRAWABLE_TYPE, GLX_PBUFFER_BIT,
        GLX_RENDER_TYPE, GLX_RGBA_BIT,
        GLX_X_VISUAL_TYPE, GLX_TRUE_COLOR,
        GLX_RED_SIZE, 8,
        GLX_GREEN_SIZE, 8,
        GLX_BLUE_SIZE, 8,
        GLX_ALPHA_SIZE, 8,
        None
    };

    int fbCount = 0;
    GLXFBConfig* fbConfigs = glXChooseFBConfig(g_x11Display, screen, fbAttribs, &fbCount);
    if (!fbConfigs || fbCount == 0) {
        if (fbConfigs) {
            XFree(fbConfigs);
        }
        SetError(error, "Failed to find a GLX framebuffer config for GL compute lightmap baking.");
        return false;
    }

    glXCreateContextAttribsARBProc createContext =
        (glXCreateContextAttribsARBProc) glXGetProcAddressARB((const GLubyte*) "glXCreateContextAttribsARB");
    if (!createContext) {
        XFree(fbConfigs);
        SetError(error, "GLX_ARB_create_context is unavailable; cannot request an OpenGL 4.3 compute context.");
        return false;
    }

    const int contextAttribs[] = {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 4,
        GLX_CONTEXT_MINOR_VERSION_ARB, 3,
        GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
        None
    };
    g_glContext = createContext(g_x11Display, fbConfigs[0], 0, True, contextAttribs);
    if (!g_glContext) {
        XFree(fbConfigs);
        SetError(error, "Failed to create an OpenGL 4.3 context for compute lightmap baking.");
        return false;
    }

    const int pbufferAttribs[] = {
        GLX_PBUFFER_WIDTH, 1,
        GLX_PBUFFER_HEIGHT, 1,
        None
    };
    g_glPbuffer = glXCreatePbuffer(g_x11Display, fbConfigs[0], pbufferAttribs);
    XFree(fbConfigs);
    if (!g_glPbuffer) {
        glXDestroyContext(g_x11Display, g_glContext);
        g_glContext = nullptr;
        SetError(error, "Failed to create a GLX pbuffer for compute lightmap baking.");
        return false;
    }

    if (!glXMakeContextCurrent(g_x11Display, g_glPbuffer, g_glPbuffer, g_glContext)) {
        glXDestroyPbuffer(g_x11Display, g_glPbuffer);
        glXDestroyContext(g_x11Display, g_glContext);
        g_glPbuffer = 0;
        g_glContext = nullptr;
        SetError(error, "Failed to activate the OpenGL compute context for lightmap baking.");
        return false;
    }

    return true;
}

void LightmapComputePlatform_Shutdown(void) {
    if (g_x11Display && g_glContext) {
        glXMakeContextCurrent(g_x11Display, None, None, nullptr);
        glXDestroyContext(g_x11Display, g_glContext);
        g_glContext = nullptr;
    }
    if (g_x11Display && g_glPbuffer) {
        glXDestroyPbuffer(g_x11Display, g_glPbuffer);
        g_glPbuffer = 0;
    }
    if (g_x11Display) {
        XCloseDisplay(g_x11Display);
        g_x11Display = nullptr;
    }
}

bool LightmapComputePlatform_ReadbackBuffer(sg_buffer buffer, size_t numBytes, void* dst, std::string* error) {
    sg_gl_buffer_info info = sg_gl_query_buffer_info(buffer);
    GLuint glBuffer = info.buf[info.active_slot];
    if (!glBuffer) {
        SetError(error, "GL readback failed: storage buffer handle is invalid.");
        return false;
    }

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, glBuffer);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr) numBytes, dst);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    glFinish();
    return true;
}

#else
bool LightmapComputePlatform_Init(sg_environment* env, std::string* error) {
    (void) env;
    SetError(error, "No lightmap compute backend is implemented for this platform.");
    return false;
}

void LightmapComputePlatform_Shutdown(void) {
}

bool LightmapComputePlatform_ReadbackBuffer(sg_buffer buffer, size_t numBytes, void* dst, std::string* error) {
    (void) buffer;
    (void) numBytes;
    (void) dst;
    SetError(error, "No lightmap compute readback path is implemented for this platform.");
    return false;
}
#endif

#endif

} // namespace

bool BuildLightmapComputeBvh(const std::vector<LightmapComputeOccluderTri>& occluders,
                             LightmapComputeBvh* outBvh,
                             std::string* error) {
    if (!outBvh) {
        SetError(error, "GPU BVH output pointer was null.");
        return false;
    }

    outBvh->nodes.clear();
    outBvh->triIndices.clear();
    if (occluders.empty()) {
        return true;
    }

#ifndef WARPED_LIGHTMAP_USE_EMBREE
    SetError(error, "Embree builder support is disabled in this compile_map build.");
    return false;
#else
    if (occluders.size() > (size_t)std::numeric_limits<unsigned int>::max()) {
        SetError(error, "Too many occluder triangles for Embree GPU BVH build.");
        return false;
    }

    std::vector<RTCBuildPrimitive> primitives(occluders.size());
    for (size_t i = 0; i < occluders.size(); ++i) {
        const LightmapComputeOccluderTri& tri = occluders[i];
        RTCBuildPrimitive& prim = primitives[i];
        prim.lower_x = tri.bounds.min.x;
        prim.lower_y = tri.bounds.min.y;
        prim.lower_z = tri.bounds.min.z;
        prim.geomID = 0;
        prim.upper_x = tri.bounds.max.x;
        prim.upper_y = tri.bounds.max.y;
        prim.upper_z = tri.bounds.max.z;
        prim.primID = (unsigned int)i;
    }

    RTCDevice device = rtcNewDevice(nullptr);
    if (!device) {
        SetError(error, "Failed to create Embree device for GPU BVH build.");
        return false;
    }

    RTCBVH embreeBvh = rtcNewBVH(device);
    if (!embreeBvh) {
        const RTCError rtcError = rtcGetDeviceError(device);
        const char* rtcMessage = rtcGetDeviceLastErrorMessage(device);
        SetError(error,
                 "Failed to create Embree BVH for GPU bake (%s%s%s).",
                 rtcGetErrorString(rtcError),
                 (rtcMessage && rtcMessage[0]) ? ": " : "",
                 (rtcMessage && rtcMessage[0]) ? rtcMessage : "");
        rtcReleaseDevice(device);
        return false;
    }

    EmbreeGpuBvhBuildContext context;
    context.occluders = &occluders;
    context.triIndices.reserve(occluders.size());

    RTCBuildArguments args = rtcDefaultBuildArguments();
    args.buildQuality = RTC_BUILD_QUALITY_MEDIUM;
    args.buildFlags = RTC_BUILD_FLAG_NONE;
    args.maxBranchingFactor = 2;
    args.maxDepth = 64;
    args.sahBlockSize = 1;
    args.minLeafSize = 1;
    args.maxLeafSize = 4;
    args.traversalCost = 1.0f;
    args.intersectionCost = 1.0f;
    args.bvh = embreeBvh;
    args.primitives = primitives.data();
    args.primitiveCount = primitives.size();
    args.primitiveArrayCapacity = primitives.size();
    args.createNode = EmbreeGpuBvhCreateNode;
    args.setNodeChildren = EmbreeGpuBvhSetNodeChildren;
    args.setNodeBounds = EmbreeGpuBvhSetNodeBounds;
    args.createLeaf = EmbreeGpuBvhCreateLeaf;
    args.userPtr = &context;

    EmbreeGpuBvhNode* root = (EmbreeGpuBvhNode*)rtcBuildBVH(&args);
    if (!root) {
        const RTCError rtcError = rtcGetDeviceError(device);
        const char* rtcMessage = rtcGetDeviceLastErrorMessage(device);
        SetError(error,
                 "Embree GPU BVH build failed (%s%s%s).",
                 rtcGetErrorString(rtcError),
                 (rtcMessage && rtcMessage[0]) ? ": " : "",
                 (rtcMessage && rtcMessage[0]) ? rtcMessage : "");
        rtcReleaseBVH(embreeBvh);
        rtcReleaseDevice(device);
        return false;
    }

    if (context.triIndices.size() > (size_t)std::numeric_limits<int>::max()) {
        SetError(error, "Embree GPU BVH produced too many triangle references.");
        rtcReleaseBVH(embreeBvh);
        rtcReleaseDevice(device);
        return false;
    }

    outBvh->triIndices = std::move(context.triIndices);
    outBvh->nodes.reserve(outBvh->triIndices.size() * 2);
    const int rootIndex = FlattenEmbreeGpuBvhNode(root, outBvh);
    rtcReleaseBVH(embreeBvh);
    rtcReleaseDevice(device);

    if (rootIndex != 0 || outBvh->nodes.empty()) {
        outBvh->nodes.clear();
        outBvh->triIndices.clear();
        SetError(error, "Embree GPU BVH flattening produced an invalid root.");
        return false;
    }
    if (outBvh->nodes.size() > (size_t)std::numeric_limits<int>::max()) {
        outBvh->nodes.clear();
        outBvh->triIndices.clear();
        SetError(error, "Embree GPU BVH produced too many nodes.");
        return false;
    }

    return true;
#endif
}

bool BakeLightmapCompute(const std::vector<LightmapComputeFaceRect>& rects,
                         const std::vector<LightmapComputeOccluderTri>& occluders,
                         const LightmapComputeBvh& bvh,
                         const std::vector<LightmapComputeBrushSolid>& brushSolids,
                         const std::vector<LightmapComputeSolidPlane>& solidPlanes,
                         const std::vector<LightmapComputeRepairSourcePoly>& repairSourcePolys,
                         const std::vector<LightmapComputeRepairSourceNeighbor>& repairSourceNeighbors,
                         const std::vector<LightmapComputePhongSourcePoly>& phongSourcePolys,
                         const std::vector<LightmapComputePhongNeighbor>& phongNeighbors,
                         const std::vector<PointLight>& lights,
                         const std::vector<LightmapComputeSurfaceEmitter>& surfaceEmitters,
                         const std::vector<LightmapComputeSurfaceEmitterSample>& surfaceEmitterSamples,
                         const std::vector<LightmapComputeRectSurfaceEmitterRange>& rectSurfaceEmitterRanges,
                         const std::vector<uint32_t>& rectSurfaceEmitterIndices,
                         const LightBakeSettings& settings,
                         float skyTraceDistance,
                         int atlasWidth,
                         int atlasHeight,
                         bool oversampledOutput,
                         std::vector<float>& outPixels,
                         std::string* error)
{
    const size_t pixelCount = (size_t) atlasWidth * (size_t) atlasHeight;
    std::vector<GpuBakedPixel> bakedPixels(pixelCount);
    outPixels.assign(pixelCount * 4, 0.0f);

    std::vector<GpuPointLight> gpuLights(std::max<size_t>(1, lights.size()));
    for (size_t i = 0; i < lights.size(); ++i) {
        CopyVec3(gpuLights[i].position, lights[i].position);
        gpuLights[i].intensity = lights[i].intensity;
        CopyVec3(gpuLights[i].color, lights[i].color);
        gpuLights[i].angle_scale = lights[i].angleScale;
        CopyVec3(gpuLights[i].emission_normal, lights[i].emissionNormal);
        gpuLights[i].directional = lights[i].directional;
        CopyVec3(gpuLights[i].parallel_direction, lights[i].parallelDirection);
        gpuLights[i].parallel = lights[i].parallel;
        CopyVec3(gpuLights[i].spot_direction, lights[i].spotDirection);
        gpuLights[i].spot_outer_cos = lights[i].spotOuterCos;
        gpuLights[i].ignore_occluder_group = lights[i].ignoreOccluderGroup;
        gpuLights[i].attenuation_mode = (int32_t) lights[i].attenuationMode;
        gpuLights[i].spot_inner_cos = lights[i].spotInnerCos;
        gpuLights[i].dirt = (int32_t) lights[i].dirt;
        gpuLights[i].dirt_scale = lights[i].dirtScale;
        gpuLights[i].dirt_gain = lights[i].dirtGain;
        gpuLights[i]._pad0 = 0;
        gpuLights[i]._pad1 = 0;
    }

    std::vector<GpuSurfaceEmitter> gpuSurfaceEmitters(std::max<size_t>(1, surfaceEmitters.size()));
    for (size_t i = 0; i < surfaceEmitters.size(); ++i) {
        CopyVec3(gpuSurfaceEmitters[i].color, surfaceEmitters[i].baseLight.color);
        gpuSurfaceEmitters[i].intensity = surfaceEmitters[i].baseLight.intensity;
        CopyVec3(gpuSurfaceEmitters[i].surface_normal, surfaceEmitters[i].surfaceNormal);
        gpuSurfaceEmitters[i].sample_intensity_scale = surfaceEmitters[i].sampleIntensityScale;
        CopyVec3(gpuSurfaceEmitters[i].spot_direction, surfaceEmitters[i].baseLight.spotDirection);
        gpuSurfaceEmitters[i].spot_outer_cos = surfaceEmitters[i].baseLight.spotOuterCos;
        gpuSurfaceEmitters[i].spot_inner_cos = surfaceEmitters[i].baseLight.spotInnerCos;
        gpuSurfaceEmitters[i].attenuation_scale = surfaceEmitters[i].attenuationScale;
        gpuSurfaceEmitters[i].transport_scale = surfaceEmitters[i].transportScale;
        gpuSurfaceEmitters[i].hotspot_clamp = surfaceEmitters[i].hotspotClamp;
        gpuSurfaceEmitters[i].dirt_scale = surfaceEmitters[i].baseLight.dirtScale;
        gpuSurfaceEmitters[i].dirt_gain = surfaceEmitters[i].baseLight.dirtGain;
        gpuSurfaceEmitters[i].ignore_occluder_group = surfaceEmitters[i].baseLight.ignoreOccluderGroup;
        gpuSurfaceEmitters[i].dirt = (int32_t)surfaceEmitters[i].baseLight.dirt;
        gpuSurfaceEmitters[i].first_sample_point = surfaceEmitters[i].firstSamplePoint;
        gpuSurfaceEmitters[i].sample_point_count = surfaceEmitters[i].samplePointCount;
        gpuSurfaceEmitters[i].omnidirectional = (int32_t)surfaceEmitters[i].omnidirectional;
        gpuSurfaceEmitters[i].rescale = (int32_t)surfaceEmitters[i].rescale;
    }

    std::vector<GpuSurfaceEmitterSample> gpuSurfaceEmitterSamples(std::max<size_t>(1, surfaceEmitterSamples.size()));
    for (size_t i = 0; i < surfaceEmitterSamples.size(); ++i) {
        CopyVec3(gpuSurfaceEmitterSamples[i].point, surfaceEmitterSamples[i].point);
        gpuSurfaceEmitterSamples[i]._pad0 = 0.0f;
    }

    std::vector<GpuSurfaceEmitterIndex> gpuRectSurfaceEmitterIndices(std::max<size_t>(1, rectSurfaceEmitterIndices.size()));
    for (size_t i = 0; i < rectSurfaceEmitterIndices.size(); ++i) {
        gpuRectSurfaceEmitterIndices[i].emitter_index = rectSurfaceEmitterIndices[i];
    }

    std::vector<GpuOccluderTri> gpuOccluders(std::max<size_t>(1, occluders.size()));
    for (size_t i = 0; i < occluders.size(); ++i) {
        CopyVec3(gpuOccluders[i].a, occluders[i].a);
        gpuOccluders[i]._pad0 = 0.0f;
        CopyVec3(gpuOccluders[i].b, occluders[i].b);
        gpuOccluders[i]._pad1 = 0.0f;
        CopyVec3(gpuOccluders[i].c, occluders[i].c);
        gpuOccluders[i]._pad2 = 0.0f;
        CopyVec3(gpuOccluders[i].bounds_min, occluders[i].bounds.min);
        gpuOccluders[i].occluder_group = occluders[i].occluderGroup;
        CopyVec3(gpuOccluders[i].bounds_max, occluders[i].bounds.max);
        gpuOccluders[i].source_poly_index = occluders[i].sourcePolyIndex;
    }

    if ((bvh.nodes.size() > (size_t)std::numeric_limits<int>::max()) ||
        (bvh.triIndices.size() > (size_t)std::numeric_limits<int>::max())) {
        SetError(error, "GPU BVH is too large for shader dispatch parameters.");
        return false;
    }

    std::vector<GpuBvhNode> gpuBvhNodes(std::max<size_t>(1, bvh.nodes.size()));
    for (size_t i = 0; i < bvh.nodes.size(); ++i) {
        CopyVec3(gpuBvhNodes[i].bounds_min, bvh.nodes[i].boundsMin);
        gpuBvhNodes[i].left_first = bvh.nodes[i].leftFirst;
        CopyVec3(gpuBvhNodes[i].bounds_max, bvh.nodes[i].boundsMax);
        gpuBvhNodes[i].right_count = bvh.nodes[i].rightCount;
    }

    std::vector<GpuBvhTriIndex> gpuBvhTriIndices(std::max<size_t>(1, bvh.triIndices.size()));
    for (size_t i = 0; i < bvh.triIndices.size(); ++i) {
        gpuBvhTriIndices[i].tri_index = bvh.triIndices[i];
        gpuBvhTriIndices[i]._pad0[0] = 0.0f;
        gpuBvhTriIndices[i]._pad0[1] = 0.0f;
        gpuBvhTriIndices[i]._pad0[2] = 0.0f;
    }

    std::vector<GpuBrushSolid> gpuBrushSolids(std::max<size_t>(1, brushSolids.size()));
    for (size_t i = 0; i < brushSolids.size(); ++i) {
        gpuBrushSolids[i].first_plane = brushSolids[i].firstPlane;
        gpuBrushSolids[i].plane_count = brushSolids[i].planeCount;
        gpuBrushSolids[i]._pad[0] = 0.0f;
        gpuBrushSolids[i]._pad[1] = 0.0f;
    }

    std::vector<GpuSolidPlane> gpuSolidPlanes(std::max<size_t>(1, solidPlanes.size()));
    for (size_t i = 0; i < solidPlanes.size(); ++i) {
        CopyVec3(gpuSolidPlanes[i].point, solidPlanes[i].point);
        gpuSolidPlanes[i]._pad0 = 0.0f;
        CopyVec3(gpuSolidPlanes[i].normal, solidPlanes[i].normal);
        gpuSolidPlanes[i]._pad1 = 0.0f;
    }

    std::vector<GpuRepairSourcePoly> gpuRepairSourcePolys(std::max<size_t>(1, repairSourcePolys.size()));
    for (size_t i = 0; i < repairSourcePolys.size(); ++i) {
        CopyVec3(gpuRepairSourcePolys[i].plane_point, repairSourcePolys[i].planePoint);
        gpuRepairSourcePolys[i]._pad0 = 0.0f;
        CopyVec3(gpuRepairSourcePolys[i].axis_u, repairSourcePolys[i].axisU);
        gpuRepairSourcePolys[i]._pad1 = 0.0f;
        CopyVec3(gpuRepairSourcePolys[i].axis_v, repairSourcePolys[i].axisV);
        gpuRepairSourcePolys[i]._pad2 = 0.0f;
        CopyVec3(gpuRepairSourcePolys[i].normal, repairSourcePolys[i].normal);
        gpuRepairSourcePolys[i].poly_count = repairSourcePolys[i].polyCount;
        gpuRepairSourcePolys[i].first_neighbor = repairSourcePolys[i].firstNeighbor;
        gpuRepairSourcePolys[i].neighbor_count = repairSourcePolys[i].neighborCount;
        gpuRepairSourcePolys[i]._pad3[0] = 0.0f;
        gpuRepairSourcePolys[i]._pad3[1] = 0.0f;
        for (int vi = 0; vi < LIGHTMAP_COMPUTE_MAX_POLY_VERTS; ++vi) {
            gpuRepairSourcePolys[i].poly_verts[vi][0] = repairSourcePolys[i].polyVerts[vi][0];
            gpuRepairSourcePolys[i].poly_verts[vi][1] = repairSourcePolys[i].polyVerts[vi][1];
            gpuRepairSourcePolys[i].poly_verts[vi][2] = repairSourcePolys[i].polyVerts[vi][2];
            gpuRepairSourcePolys[i].poly_verts[vi][3] = repairSourcePolys[i].polyVerts[vi][3];
        }
    }

    std::vector<GpuRepairSourceNeighbor> gpuRepairSourceNeighbors(std::max<size_t>(1, repairSourceNeighbors.size()));
    for (size_t i = 0; i < repairSourceNeighbors.size(); ++i) {
        gpuRepairSourceNeighbors[i].source_poly_index = repairSourceNeighbors[i].sourcePolyIndex;
        gpuRepairSourceNeighbors[i].edge_index = repairSourceNeighbors[i].edgeIndex;
        gpuRepairSourceNeighbors[i]._pad[0] = 0.0f;
        gpuRepairSourceNeighbors[i]._pad[1] = 0.0f;
    }

    std::vector<GpuPhongSourcePoly> gpuPhongSourcePolys(std::max<size_t>(1, phongSourcePolys.size()));
    for (size_t i = 0; i < phongSourcePolys.size(); ++i) {
        CopyVec3(gpuPhongSourcePolys[i].normal, phongSourcePolys[i].normal);
        gpuPhongSourcePolys[i].area_weight = phongSourcePolys[i].areaWeight;
        gpuPhongSourcePolys[i].enabled = phongSourcePolys[i].enabled;
        gpuPhongSourcePolys[i].first_neighbor = phongSourcePolys[i].firstNeighbor;
        gpuPhongSourcePolys[i].neighbor_count = phongSourcePolys[i].neighborCount;
        gpuPhongSourcePolys[i]._pad0 = 0.0f;
    }

    std::vector<GpuPhongNeighbor> gpuPhongNeighbors(std::max<size_t>(1, phongNeighbors.size()));
    for (size_t i = 0; i < phongNeighbors.size(); ++i) {
        gpuPhongNeighbors[i].source_poly_index = phongNeighbors[i].sourcePolyIndex;
        gpuPhongNeighbors[i]._pad0[0] = 0.0f;
        gpuPhongNeighbors[i]._pad0[1] = 0.0f;
        gpuPhongNeighbors[i]._pad0[2] = 0.0f;
        CopyVec3(gpuPhongNeighbors[i].edge_a, phongNeighbors[i].edgeA);
        gpuPhongNeighbors[i]._pad1 = 0.0f;
        CopyVec3(gpuPhongNeighbors[i].edge_b, phongNeighbors[i].edgeB);
        gpuPhongNeighbors[i]._pad2 = 0.0f;
        CopyVec3(gpuPhongNeighbors[i].normal, phongNeighbors[i].normal);
        gpuPhongNeighbors[i].area_weight = phongNeighbors[i].areaWeight;
    }

    sg_environment environment{};
    if (!LightmapComputePlatform_Init(&environment, error)) {
        return false;
    }

    bool success = false;
    sg_buffer lightsBuffer{};
    sg_buffer surfaceEmitterBuffer{};
    sg_buffer surfaceEmitterSampleBuffer{};
    sg_buffer rectSurfaceEmitterIndexBuffer{};
    sg_buffer occluderBuffer{};
    sg_buffer bvhNodeBuffer{};
    sg_buffer bvhTriIndexBuffer{};
    sg_buffer solidBuffer{};
    sg_buffer solidPlaneBuffer{};
    sg_buffer repairSourcePolyBuffer{};
    sg_buffer repairSourceNeighborBuffer{};
    sg_buffer phongSourcePolyBuffer{};
    sg_buffer phongNeighborBuffer{};
    sg_buffer outputBuffer{};
    sg_view lightsView{};
    sg_view surfaceEmitterView{};
    sg_view surfaceEmitterSampleView{};
    sg_view rectSurfaceEmitterIndexView{};
    sg_view occluderView{};
    sg_view bvhNodeView{};
    sg_view bvhTriIndexView{};
    sg_view solidView{};
    sg_view solidPlaneView{};
    sg_view repairSourcePolyView{};
    sg_view repairSourceNeighborView{};
    sg_view phongSourcePolyView{};
    sg_view phongNeighborView{};
    sg_view outputView{};
    sg_shader shader{};
    sg_pipeline pipeline{};
    sg_desc setupDesc{};
    sg_pipeline_desc pipelineDesc{};
    sg_buffer_desc lightsDesc{};
    sg_buffer_desc surfaceEmitterDesc{};
    sg_buffer_desc surfaceEmitterSampleDesc{};
    sg_buffer_desc rectSurfaceEmitterIndexDesc{};
    sg_buffer_desc occluderDesc{};
    sg_buffer_desc bvhNodeDesc{};
    sg_buffer_desc bvhTriIndexDesc{};
    sg_buffer_desc solidDesc{};
    sg_buffer_desc solidPlaneDesc{};
    sg_buffer_desc repairSourcePolyDesc{};
    sg_buffer_desc repairSourceNeighborDesc{};
    sg_buffer_desc phongSourcePolyDesc{};
    sg_buffer_desc phongNeighborDesc{};
    sg_buffer_desc outputDesc{};
    sg_view_desc lightsViewDesc{};
    sg_view_desc surfaceEmitterViewDesc{};
    sg_view_desc surfaceEmitterSampleViewDesc{};
    sg_view_desc rectSurfaceEmitterIndexViewDesc{};
    sg_view_desc occluderViewDesc{};
    sg_view_desc bvhNodeViewDesc{};
    sg_view_desc bvhTriIndexViewDesc{};
    sg_view_desc solidViewDesc{};
    sg_view_desc solidPlaneViewDesc{};
    sg_view_desc repairSourcePolyViewDesc{};
    sg_view_desc repairSourceNeighborViewDesc{};
    sg_view_desc phongSourcePolyViewDesc{};
    sg_view_desc phongNeighborViewDesc{};
    sg_view_desc outputViewDesc{};
    sg_pass pass{};
    sg_bindings bindings{};

    setupDesc.buffer_pool_size = 14;
    setupDesc.shader_pool_size = 2;
    setupDesc.pipeline_pool_size = 2;
    setupDesc.view_pool_size = 14;
    // One face-params upload is issued per rect dispatch. On Metal these uploads
    // are placed into a ring buffer with 256-byte alignment, so a 32-rect batch
    // needs far more than the old fixed 4 KB allocation.
    setupDesc.uniform_buffer_size = (int)(kDispatchBatchSize * AlignUp(sizeof(GpuFaceParams), kUniformUploadAlign));
    setupDesc.logger.func = slog_func;
    setupDesc.environment = environment;
    sg_setup(&setupDesc);

    if (!sg_isvalid()) {
        SetError(error, "sg_setup failed for the lightmap compute baker.");
        goto cleanup;
    }

    if (!BackendMatchesBuild(sg_query_backend())) {
        SetError(error,
                 "Lightmap compute backend mismatch: build expects this platform backend, runtime resolved %s.",
                 BackendName(sg_query_backend()));
        goto cleanup;
    }

    if (!sg_query_features().compute) {
        SetError(error, "Sokol compute is unavailable on backend %s.", BackendName(sg_query_backend()));
        goto cleanup;
    }

    shader = sg_make_shader(warped_lightmap_bake_bake_shader_desc(sg_query_backend()));
    if (sg_query_shader_state(shader) != SG_RESOURCESTATE_VALID) {
        SetError(error, "Failed to create lightmap compute shader on backend %s.", BackendName(sg_query_backend()));
        goto cleanup;
    }

    pipelineDesc.shader = shader;
    pipelineDesc.compute = true;
    pipelineDesc.label = "lightmap-bake-compute";
    pipeline = sg_make_pipeline(&pipelineDesc);
    if (sg_query_pipeline_state(pipeline) != SG_RESOURCESTATE_VALID) {
        SetError(error, "Failed to create lightmap compute pipeline on backend %s.", BackendName(sg_query_backend()));
        goto cleanup;
    }

    lightsDesc.size = gpuLights.size() * sizeof(GpuPointLight);
    lightsDesc.usage.storage_buffer = true;
    lightsDesc.usage.immutable = true;
    lightsDesc.data = ByteRange(gpuLights);
    lightsDesc.label = "lightmap-lights";
    lightsBuffer = sg_make_buffer(&lightsDesc);

    surfaceEmitterDesc.size = gpuSurfaceEmitters.size() * sizeof(GpuSurfaceEmitter);
    surfaceEmitterDesc.usage.storage_buffer = true;
    surfaceEmitterDesc.usage.immutable = true;
    surfaceEmitterDesc.data = ByteRange(gpuSurfaceEmitters);
    surfaceEmitterDesc.label = "lightmap-surface-emitters";
    surfaceEmitterBuffer = sg_make_buffer(&surfaceEmitterDesc);

    surfaceEmitterSampleDesc.size = gpuSurfaceEmitterSamples.size() * sizeof(GpuSurfaceEmitterSample);
    surfaceEmitterSampleDesc.usage.storage_buffer = true;
    surfaceEmitterSampleDesc.usage.immutable = true;
    surfaceEmitterSampleDesc.data = ByteRange(gpuSurfaceEmitterSamples);
    surfaceEmitterSampleDesc.label = "lightmap-surface-emitter-samples";
    surfaceEmitterSampleBuffer = sg_make_buffer(&surfaceEmitterSampleDesc);

    rectSurfaceEmitterIndexDesc.size = gpuRectSurfaceEmitterIndices.size() * sizeof(GpuSurfaceEmitterIndex);
    rectSurfaceEmitterIndexDesc.usage.storage_buffer = true;
    rectSurfaceEmitterIndexDesc.usage.immutable = true;
    rectSurfaceEmitterIndexDesc.data = ByteRange(gpuRectSurfaceEmitterIndices);
    rectSurfaceEmitterIndexDesc.label = "lightmap-rect-surface-emitter-indices";
    rectSurfaceEmitterIndexBuffer = sg_make_buffer(&rectSurfaceEmitterIndexDesc);

    occluderDesc.size = gpuOccluders.size() * sizeof(GpuOccluderTri);
    occluderDesc.usage.storage_buffer = true;
    occluderDesc.usage.immutable = true;
    occluderDesc.data = ByteRange(gpuOccluders);
    occluderDesc.label = "lightmap-occluders";
    occluderBuffer = sg_make_buffer(&occluderDesc);

    bvhNodeDesc.size = gpuBvhNodes.size() * sizeof(GpuBvhNode);
    bvhNodeDesc.usage.storage_buffer = true;
    bvhNodeDesc.usage.immutable = true;
    bvhNodeDesc.data = ByteRange(gpuBvhNodes);
    bvhNodeDesc.label = "lightmap-bvh-nodes";
    bvhNodeBuffer = sg_make_buffer(&bvhNodeDesc);

    bvhTriIndexDesc.size = gpuBvhTriIndices.size() * sizeof(GpuBvhTriIndex);
    bvhTriIndexDesc.usage.storage_buffer = true;
    bvhTriIndexDesc.usage.immutable = true;
    bvhTriIndexDesc.data = ByteRange(gpuBvhTriIndices);
    bvhTriIndexDesc.label = "lightmap-bvh-tri-indices";
    bvhTriIndexBuffer = sg_make_buffer(&bvhTriIndexDesc);

    solidDesc.size = gpuBrushSolids.size() * sizeof(GpuBrushSolid);
    solidDesc.usage.storage_buffer = true;
    solidDesc.usage.immutable = true;
    solidDesc.data = ByteRange(gpuBrushSolids);
    solidDesc.label = "lightmap-solids";
    solidBuffer = sg_make_buffer(&solidDesc);

    solidPlaneDesc.size = gpuSolidPlanes.size() * sizeof(GpuSolidPlane);
    solidPlaneDesc.usage.storage_buffer = true;
    solidPlaneDesc.usage.immutable = true;
    solidPlaneDesc.data = ByteRange(gpuSolidPlanes);
    solidPlaneDesc.label = "lightmap-solid-planes";
    solidPlaneBuffer = sg_make_buffer(&solidPlaneDesc);

    repairSourcePolyDesc.size = gpuRepairSourcePolys.size() * sizeof(GpuRepairSourcePoly);
    repairSourcePolyDesc.usage.storage_buffer = true;
    repairSourcePolyDesc.usage.immutable = true;
    repairSourcePolyDesc.data = ByteRange(gpuRepairSourcePolys);
    repairSourcePolyDesc.label = "lightmap-repair-source-polys";
    repairSourcePolyBuffer = sg_make_buffer(&repairSourcePolyDesc);

    repairSourceNeighborDesc.size = gpuRepairSourceNeighbors.size() * sizeof(GpuRepairSourceNeighbor);
    repairSourceNeighborDesc.usage.storage_buffer = true;
    repairSourceNeighborDesc.usage.immutable = true;
    repairSourceNeighborDesc.data = ByteRange(gpuRepairSourceNeighbors);
    repairSourceNeighborDesc.label = "lightmap-repair-source-neighbors";
    repairSourceNeighborBuffer = sg_make_buffer(&repairSourceNeighborDesc);

    phongSourcePolyDesc.size = gpuPhongSourcePolys.size() * sizeof(GpuPhongSourcePoly);
    phongSourcePolyDesc.usage.storage_buffer = true;
    phongSourcePolyDesc.usage.immutable = true;
    phongSourcePolyDesc.data = ByteRange(gpuPhongSourcePolys);
    phongSourcePolyDesc.label = "lightmap-phong-source-polys";
    phongSourcePolyBuffer = sg_make_buffer(&phongSourcePolyDesc);

    phongNeighborDesc.size = gpuPhongNeighbors.size() * sizeof(GpuPhongNeighbor);
    phongNeighborDesc.usage.storage_buffer = true;
    phongNeighborDesc.usage.immutable = true;
    phongNeighborDesc.data = ByteRange(gpuPhongNeighbors);
    phongNeighborDesc.label = "lightmap-phong-neighbors";
    phongNeighborBuffer = sg_make_buffer(&phongNeighborDesc);

    outputDesc.size = bakedPixels.size() * sizeof(GpuBakedPixel);
    outputDesc.usage.storage_buffer = true;
    outputDesc.usage.immutable = true;
    outputDesc.data = ByteRange(bakedPixels);
    outputDesc.label = "lightmap-output";
    outputBuffer = sg_make_buffer(&outputDesc);
    if ((sg_query_buffer_state(lightsBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(surfaceEmitterBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(surfaceEmitterSampleBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(rectSurfaceEmitterIndexBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(occluderBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(bvhNodeBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(bvhTriIndexBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(solidBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(solidPlaneBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(repairSourcePolyBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(repairSourceNeighborBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(phongSourcePolyBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(phongNeighborBuffer) != SG_RESOURCESTATE_VALID) ||
        (sg_query_buffer_state(outputBuffer) != SG_RESOURCESTATE_VALID))
    {
        SetError(error,
                 "Failed to create compute buffers: lights=%s surface_emitters=%s surface_samples=%s rect_surface_indices=%s occluders=%s bvh_nodes=%s bvh_tri_indices=%s solids=%s solid_planes=%s repair_polys=%s repair_neighbors=%s phong_polys=%s phong_neighbors=%s output=%s.",
                 ResourceStateName(sg_query_buffer_state(lightsBuffer)),
                 ResourceStateName(sg_query_buffer_state(surfaceEmitterBuffer)),
                 ResourceStateName(sg_query_buffer_state(surfaceEmitterSampleBuffer)),
                 ResourceStateName(sg_query_buffer_state(rectSurfaceEmitterIndexBuffer)),
                 ResourceStateName(sg_query_buffer_state(occluderBuffer)),
                 ResourceStateName(sg_query_buffer_state(bvhNodeBuffer)),
                 ResourceStateName(sg_query_buffer_state(bvhTriIndexBuffer)),
                 ResourceStateName(sg_query_buffer_state(solidBuffer)),
                 ResourceStateName(sg_query_buffer_state(solidPlaneBuffer)),
                 ResourceStateName(sg_query_buffer_state(repairSourcePolyBuffer)),
                 ResourceStateName(sg_query_buffer_state(repairSourceNeighborBuffer)),
                 ResourceStateName(sg_query_buffer_state(phongSourcePolyBuffer)),
                 ResourceStateName(sg_query_buffer_state(phongNeighborBuffer)),
                 ResourceStateName(sg_query_buffer_state(outputBuffer)));
        goto cleanup;
    }

    lightsViewDesc.storage_buffer.buffer = lightsBuffer;
    lightsViewDesc.label = "lightmap-lights-view";
    lightsView = sg_make_view(&lightsViewDesc);

    surfaceEmitterViewDesc.storage_buffer.buffer = surfaceEmitterBuffer;
    surfaceEmitterViewDesc.label = "lightmap-surface-emitters-view";
    surfaceEmitterView = sg_make_view(&surfaceEmitterViewDesc);

    surfaceEmitterSampleViewDesc.storage_buffer.buffer = surfaceEmitterSampleBuffer;
    surfaceEmitterSampleViewDesc.label = "lightmap-surface-emitter-samples-view";
    surfaceEmitterSampleView = sg_make_view(&surfaceEmitterSampleViewDesc);

    rectSurfaceEmitterIndexViewDesc.storage_buffer.buffer = rectSurfaceEmitterIndexBuffer;
    rectSurfaceEmitterIndexViewDesc.label = "lightmap-rect-surface-emitter-indices-view";
    rectSurfaceEmitterIndexView = sg_make_view(&rectSurfaceEmitterIndexViewDesc);

    occluderViewDesc.storage_buffer.buffer = occluderBuffer;
    occluderViewDesc.label = "lightmap-occluders-view";
    occluderView = sg_make_view(&occluderViewDesc);

    bvhNodeViewDesc.storage_buffer.buffer = bvhNodeBuffer;
    bvhNodeViewDesc.label = "lightmap-bvh-nodes-view";
    bvhNodeView = sg_make_view(&bvhNodeViewDesc);

    bvhTriIndexViewDesc.storage_buffer.buffer = bvhTriIndexBuffer;
    bvhTriIndexViewDesc.label = "lightmap-bvh-tri-indices-view";
    bvhTriIndexView = sg_make_view(&bvhTriIndexViewDesc);

    solidViewDesc.storage_buffer.buffer = solidBuffer;
    solidViewDesc.label = "lightmap-solids-view";
    solidView = sg_make_view(&solidViewDesc);

    solidPlaneViewDesc.storage_buffer.buffer = solidPlaneBuffer;
    solidPlaneViewDesc.label = "lightmap-solid-planes-view";
    solidPlaneView = sg_make_view(&solidPlaneViewDesc);

    repairSourcePolyViewDesc.storage_buffer.buffer = repairSourcePolyBuffer;
    repairSourcePolyViewDesc.label = "lightmap-repair-source-polys-view";
    repairSourcePolyView = sg_make_view(&repairSourcePolyViewDesc);

    repairSourceNeighborViewDesc.storage_buffer.buffer = repairSourceNeighborBuffer;
    repairSourceNeighborViewDesc.label = "lightmap-repair-source-neighbors-view";
    repairSourceNeighborView = sg_make_view(&repairSourceNeighborViewDesc);

    phongSourcePolyViewDesc.storage_buffer.buffer = phongSourcePolyBuffer;
    phongSourcePolyViewDesc.label = "lightmap-phong-source-polys-view";
    phongSourcePolyView = sg_make_view(&phongSourcePolyViewDesc);

    phongNeighborViewDesc.storage_buffer.buffer = phongNeighborBuffer;
    phongNeighborViewDesc.label = "lightmap-phong-neighbors-view";
    phongNeighborView = sg_make_view(&phongNeighborViewDesc);

    outputViewDesc.storage_buffer.buffer = outputBuffer;
    outputViewDesc.label = "lightmap-output-view";
    outputView = sg_make_view(&outputViewDesc);
    if ((sg_query_view_state(lightsView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(surfaceEmitterView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(surfaceEmitterSampleView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(rectSurfaceEmitterIndexView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(occluderView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(bvhNodeView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(bvhTriIndexView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(solidView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(solidPlaneView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(repairSourcePolyView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(repairSourceNeighborView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(phongSourcePolyView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(phongNeighborView) != SG_RESOURCESTATE_VALID) ||
        (sg_query_view_state(outputView) != SG_RESOURCESTATE_VALID))
    {
        SetError(error,
                 "Failed to create compute buffer views: lights=%s surface_emitters=%s surface_samples=%s rect_surface_indices=%s occluders=%s bvh_nodes=%s bvh_tri_indices=%s solids=%s solid_planes=%s repair_polys=%s repair_neighbors=%s phong_polys=%s phong_neighbors=%s output=%s.",
                 ResourceStateName(sg_query_view_state(lightsView)),
                 ResourceStateName(sg_query_view_state(surfaceEmitterView)),
                 ResourceStateName(sg_query_view_state(surfaceEmitterSampleView)),
                 ResourceStateName(sg_query_view_state(rectSurfaceEmitterIndexView)),
                 ResourceStateName(sg_query_view_state(occluderView)),
                 ResourceStateName(sg_query_view_state(bvhNodeView)),
                 ResourceStateName(sg_query_view_state(bvhTriIndexView)),
                 ResourceStateName(sg_query_view_state(solidView)),
                 ResourceStateName(sg_query_view_state(solidPlaneView)),
                 ResourceStateName(sg_query_view_state(repairSourcePolyView)),
                 ResourceStateName(sg_query_view_state(repairSourceNeighborView)),
                 ResourceStateName(sg_query_view_state(phongSourcePolyView)),
                 ResourceStateName(sg_query_view_state(phongNeighborView)),
                 ResourceStateName(sg_query_view_state(outputView)));
        goto cleanup;
    }

    pass.compute = true;
    printf("[Lightmap] compute pass begin (%zu rects, %dx%d)\n", rects.size(), atlasWidth, atlasHeight);
    fflush(stdout);

    bindings.views[VIEW_warped_lightmap_bake_cs_lights] = lightsView;
    bindings.views[VIEW_warped_lightmap_bake_cs_occluders] = occluderView;
    bindings.views[VIEW_warped_lightmap_bake_cs_bvh_nodes] = bvhNodeView;
    bindings.views[VIEW_warped_lightmap_bake_cs_bvh_tri_indices] = bvhTriIndexView;
    bindings.views[VIEW_warped_lightmap_bake_cs_solids] = solidView;
    bindings.views[VIEW_warped_lightmap_bake_cs_solid_planes] = solidPlaneView;
    bindings.views[VIEW_warped_lightmap_bake_cs_repair_source_polys] = repairSourcePolyView;
    bindings.views[VIEW_warped_lightmap_bake_cs_repair_source_neighbors] = repairSourceNeighborView;
    bindings.views[VIEW_warped_lightmap_bake_cs_phong_source_polys] = phongSourcePolyView;
    bindings.views[VIEW_warped_lightmap_bake_cs_phong_neighbors] = phongNeighborView;
    bindings.views[VIEW_warped_lightmap_bake_cs_output] = outputView;
    bindings.views[VIEW_warped_lightmap_bake_cs_surface_emitters] = surfaceEmitterView;
    bindings.views[VIEW_warped_lightmap_bake_cs_surface_emitter_samples] = surfaceEmitterSampleView;
    bindings.views[VIEW_warped_lightmap_bake_cs_rect_surface_emitter_indices] = rectSurfaceEmitterIndexView;

    for (size_t batchStart = 0; batchStart < rects.size(); batchStart += kDispatchBatchSize) {
        const size_t batchEnd = std::min(batchStart + (size_t)kDispatchBatchSize, rects.size());
        sg_begin_pass(&pass);
        sg_apply_pipeline(pipeline);
        sg_apply_bindings(&bindings);

        for (size_t rectIndex = batchStart; rectIndex < batchEnd; ++rectIndex) {
            const LightmapComputeFaceRect& rect = rects[rectIndex];
            GpuFaceParams params{};
            params.atlas_width = atlasWidth;
            params.atlas_height = atlasHeight;
            params.rect_x = rect.x;
            params.rect_y = rect.y;
            params.rect_w = rect.w;
            params.rect_h = rect.h;
            params.light_count = (int) lights.size();
            params.surface_emitter_count_total = (int)surfaceEmitters.size();
            params.surface_emitter_sample_count_total = (int)surfaceEmitterSamples.size();
            params.rect_surface_emitter_index_count_total = (int)rectSurfaceEmitterIndices.size();
            params.tri_count = (int) occluders.size();
            params.bvh_node_count = (int)bvh.nodes.size();
            params.bvh_tri_index_count = (int)bvh.triIndices.size();
            params.solid_count = (int) brushSolids.size();
            params.solid_plane_count = (int) solidPlanes.size();
            params.repair_poly_count = (int) repairSourcePolys.size();
            params.repair_link_count = (int) repairSourceNeighbors.size();
            params.phong_poly_count = (int) phongSourcePolys.size();
            params.phong_neighbor_count_total = (int) phongNeighbors.size();
            params.repair_source_poly_index = rect.sourcePolyIndex;
            params._pad_repair0 = 0;
            params.min_u = rect.minU;
            params.min_v = rect.minV;
            params.luxel_size = rect.luxelSize;
            params.shadow_bias = kShadowBias;
            CopyVec3(params.ambient_color, settings.ambientColor);
            params.ray_eps = kRayEps;
            params.global_dirt = settings.dirt;
            params.dirt_mode = settings.dirtMode;
            params.skylight_dirt = ((settings.sunlight2Dirt == -2) ? settings.dirt : settings.sunlight2Dirt) == 1 ? 1 : 0;
            params.phong_neighbor_count = rect.phongNeighborCount;
            if (rectIndex < rectSurfaceEmitterRanges.size()) {
                params.rect_surface_emitter_index_first = rectSurfaceEmitterRanges[rectIndex].firstEmitterIndex;
                params.rect_surface_emitter_index_count = rectSurfaceEmitterRanges[rectIndex].emitterCount;
            } else {
                params.rect_surface_emitter_index_first = 0;
                params.rect_surface_emitter_index_count = 0;
            }
            params.dirt_depth = settings.dirtDepth;
            params.dirt_scale = settings.dirtScale;
            params.dirt_gain = settings.dirtGain;
            params.dirt_angle = settings.dirtAngle;
            params.skylight_angle_scale = settings.sunlightAngleScale;
            params.sky_trace_distance = skyTraceDistance;
            params.sunlight_nosky = settings.sunlightNoSky;
            params.aa_grid = ComputeLightmapAAGridSize(settings.extraSamples);
            params.oversampled_output = oversampledOutput ? 1 : 0;
            params.surface_sample_offset = settings.surfaceSampleOffset;
            params._pad_scalar0 = 0.0f;
            CopyVec3(params.sunlight2_color, settings.sunlight2Color);
            params.sunlight2_intensity = settings.sunlight2Intensity;
            CopyVec3(params.sunlight3_color, settings.sunlight3Color);
            params.sunlight3_intensity = settings.sunlight3Intensity;
            CopyVec3(params.origin, rect.origin);
            params._pad0 = 0.0f;
            CopyVec3(params.axis_u, rect.axisU);
            params._pad1 = 0.0f;
            CopyVec3(params.axis_v, rect.axisV);
            params._pad2 = 0.0f;
            CopyVec3(params.normal, rect.normal);
            params._pad3 = 0.0f;
            params.poly_count = rect.polyCount;
            params._pad_poly_count = 0;
            params.phong_base_normal_weight[0] = rect.phongBaseNormal.x;
            params.phong_base_normal_weight[1] = rect.phongBaseNormal.y;
            params.phong_base_normal_weight[2] = rect.phongBaseNormal.z;
            params.phong_base_normal_weight[3] = rect.phongBaseAreaWeight;
            for (int i = 0; i < LIGHTMAP_COMPUTE_MAX_POLY_VERTS; ++i) {
                params.poly_verts[i][0] = rect.polyVerts[i][0];
                params.poly_verts[i][1] = rect.polyVerts[i][1];
                params.poly_verts[i][2] = rect.polyVerts[i][2];
                params.poly_verts[i][3] = rect.polyVerts[i][3];
            }
            for (int i = 0; i < LIGHTMAP_COMPUTE_MAX_PHONG_NEIGHBORS; ++i) {
                params.phong_neighbor_edge_a[i][0] = rect.phongNeighborEdgeA[i][0];
                params.phong_neighbor_edge_a[i][1] = rect.phongNeighborEdgeA[i][1];
                params.phong_neighbor_edge_a[i][2] = rect.phongNeighborEdgeA[i][2];
                params.phong_neighbor_edge_a[i][3] = rect.phongNeighborEdgeA[i][3];
                params.phong_neighbor_edge_b[i][0] = rect.phongNeighborEdgeB[i][0];
                params.phong_neighbor_edge_b[i][1] = rect.phongNeighborEdgeB[i][1];
                params.phong_neighbor_edge_b[i][2] = rect.phongNeighborEdgeB[i][2];
                params.phong_neighbor_edge_b[i][3] = rect.phongNeighborEdgeB[i][3];
                params.phong_neighbor_normal_weight[i][0] = rect.phongNeighborNormalWeight[i][0];
                params.phong_neighbor_normal_weight[i][1] = rect.phongNeighborNormalWeight[i][1];
                params.phong_neighbor_normal_weight[i][2] = rect.phongNeighborNormalWeight[i][2];
                params.phong_neighbor_normal_weight[i][3] = rect.phongNeighborNormalWeight[i][3];
            }

            const sg_range paramsRange = ByteRange(&params, sizeof(params));
            sg_apply_uniforms(UB_warped_lightmap_bake_cs_face_params, &paramsRange);
            const int groupsX = (rect.w + (kWorkgroupSize - 1)) / kWorkgroupSize;
            const int groupsY = (rect.h + (kWorkgroupSize - 1)) / kWorkgroupSize;
            sg_dispatch(groupsX, groupsY, 1);
        }

        sg_end_pass();
        sg_commit();
        printf("[Lightmap] compute batch %zu-%zu/%zu committed\n",
               batchStart + 1, batchEnd, rects.size());
        fflush(stdout);
    }

    printf("[Lightmap] compute pass committed, reading back buffer\n");
    fflush(stdout);

    if (!LightmapComputePlatform_ReadbackBuffer(outputBuffer, bakedPixels.size() * sizeof(GpuBakedPixel), bakedPixels.data(), error)) {
        goto cleanup;
    }
    printf("[Lightmap] compute readback complete\n");
    fflush(stdout);

    for (size_t i = 0; i < bakedPixels.size(); ++i) {
        outPixels[i * 4 + 0] = bakedPixels[i].value[0];
        outPixels[i * 4 + 1] = bakedPixels[i].value[1];
        outPixels[i * 4 + 2] = bakedPixels[i].value[2];
        outPixels[i * 4 + 3] = bakedPixels[i].value[3];
    }

    printf("[Lightmap] compute bake succeeded on backend %s (%zu rects).\n",
           BackendName(sg_query_backend()), rects.size());
    fflush(stdout);
    success = true;

cleanup:
    if (outputView.id) {
        sg_destroy_view(outputView);
    }
    if (rectSurfaceEmitterIndexView.id) {
        sg_destroy_view(rectSurfaceEmitterIndexView);
    }
    if (surfaceEmitterSampleView.id) {
        sg_destroy_view(surfaceEmitterSampleView);
    }
    if (surfaceEmitterView.id) {
        sg_destroy_view(surfaceEmitterView);
    }
    if (phongNeighborView.id) {
        sg_destroy_view(phongNeighborView);
    }
    if (phongSourcePolyView.id) {
        sg_destroy_view(phongSourcePolyView);
    }
    if (repairSourceNeighborView.id) {
        sg_destroy_view(repairSourceNeighborView);
    }
    if (repairSourcePolyView.id) {
        sg_destroy_view(repairSourcePolyView);
    }
    if (solidPlaneView.id) {
        sg_destroy_view(solidPlaneView);
    }
    if (solidView.id) {
        sg_destroy_view(solidView);
    }
    if (bvhTriIndexView.id) {
        sg_destroy_view(bvhTriIndexView);
    }
    if (bvhNodeView.id) {
        sg_destroy_view(bvhNodeView);
    }
    if (occluderView.id) {
        sg_destroy_view(occluderView);
    }
    if (lightsView.id) {
        sg_destroy_view(lightsView);
    }
    if (outputBuffer.id) {
        sg_destroy_buffer(outputBuffer);
    }
    if (rectSurfaceEmitterIndexBuffer.id) {
        sg_destroy_buffer(rectSurfaceEmitterIndexBuffer);
    }
    if (surfaceEmitterSampleBuffer.id) {
        sg_destroy_buffer(surfaceEmitterSampleBuffer);
    }
    if (surfaceEmitterBuffer.id) {
        sg_destroy_buffer(surfaceEmitterBuffer);
    }
    if (phongNeighborBuffer.id) {
        sg_destroy_buffer(phongNeighborBuffer);
    }
    if (phongSourcePolyBuffer.id) {
        sg_destroy_buffer(phongSourcePolyBuffer);
    }
    if (repairSourceNeighborBuffer.id) {
        sg_destroy_buffer(repairSourceNeighborBuffer);
    }
    if (repairSourcePolyBuffer.id) {
        sg_destroy_buffer(repairSourcePolyBuffer);
    }
    if (solidPlaneBuffer.id) {
        sg_destroy_buffer(solidPlaneBuffer);
    }
    if (solidBuffer.id) {
        sg_destroy_buffer(solidBuffer);
    }
    if (bvhTriIndexBuffer.id) {
        sg_destroy_buffer(bvhTriIndexBuffer);
    }
    if (bvhNodeBuffer.id) {
        sg_destroy_buffer(bvhNodeBuffer);
    }
    if (occluderBuffer.id) {
        sg_destroy_buffer(occluderBuffer);
    }
    if (lightsBuffer.id) {
        sg_destroy_buffer(lightsBuffer);
    }
    if (pipeline.id) {
        sg_destroy_pipeline(pipeline);
    }
    if (shader.id) {
        sg_destroy_shader(shader);
    }
    if (sg_isvalid()) {
        sg_shutdown();
    }
    LightmapComputePlatform_Shutdown();
    return success;
}
