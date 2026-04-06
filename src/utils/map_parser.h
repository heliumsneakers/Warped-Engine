// map_parser.h
#pragma once
#include "../math/wmath.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <cstdint>

struct TextureManager;   // forward — lives in render/renderer.h

/*
 * Epsilon used in intersection tests and duplicate checks.
 * Adjust if needed for large or extremely small geometry.
 */
const double epsilon = 1e-2f;

typedef struct Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 texcoord;
} Vertex;

struct Face {
    std::vector<Vector3> vertices;
    std::string texture;
    Vector3 textureAxes1;
    Vector3 textureAxes2;
    float offsetX;
    float offsetY;
    float rotation;
    float scaleX;
    float scaleY;
    Vector3 normal;
};

struct Plane {
    Vector3 normal;
    double d;
};

typedef struct Brush {
    std::vector<Face> faces;
} Brush;

typedef struct Entity {
    std::unordered_map<std::string, std::string> properties;
    std::vector<Brush> brushes;
} Entity;

typedef struct Map {
    std::vector<Entity> entities;
} Map;

typedef struct PlayerStart {
    Vector3 position;
} PlayerStart;

enum PointLightAttenuationMode : uint8_t {
    POINT_LIGHT_ATTEN_QUADRATIC = 0,
    POINT_LIGHT_ATTEN_LINEAR,
    POINT_LIGHT_ATTEN_INVERSE,
    POINT_LIGHT_ATTEN_INVERSE_SQUARE,
    POINT_LIGHT_ATTEN_NONE,
    POINT_LIGHT_ATTEN_LOCAL_MINLIGHT,
    POINT_LIGHT_ATTEN_INVERSE_SQUARE_B,
};

struct PointLight {
    Vector3 position;     // world-space (GL coords)
    Vector3 color;        // 0..1
    float   intensity;    // radius in world units, or max trace distance for parallel suns
    Vector3 emissionNormal{0.0f, 0.0f, 0.0f};
    int     directional = 0;
    Vector3 parallelDirection{0.0f, 0.0f, 0.0f}; // surface -> light direction for parallel sun/skydome samples
    int     parallel = 0;
    int     ignoreOccluderGroup = -1;
    uint8_t attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
    float   angleScale = 1.0f;
    int8_t  dirt = -2;    // -2 defer, -1 disable, 1 enable
    float   dirtScale = -1.0f;
    float   dirtGain = -1.0f;
    Vector3 spotDirection{0.0f, 0.0f, 0.0f};
    float   spotOuterCos = -2.0f;
    float   spotInnerCos = -2.0f;
};

struct SurfaceLightTemplate {
    std::string texture;
    int         surfaceLightGroup = 0;
    float       surfaceOffset = 2.0f;
    int         surfaceSpotlight = 0;
    float       deviance = 0.0f;
    int         devianceSamples = 16;
    PointLight  light;
};

struct LightBakeSettings {
    Vector3 ambientColor{0.12f, 0.12f, 0.12f};
    float   luxelSize = 1.0f;
    int     bounceCount = 1;
    float   bounceScale = 1.0f;
    float   sunlightIntensity = 0.0f;
    Vector3 sunlightColor{1.0f, 1.0f, 1.0f};
    Vector3 sunlightDirection{0.0f, 1.0f, 0.0f}; // surface -> light
    float   sunlightPenumbra = 0.0f;
    float   sunlightAngleScale = 0.5f;
    float   sunlight2Intensity = 0.0f;
    Vector3 sunlight2Color{1.0f, 1.0f, 1.0f};
    float   sunlight3Intensity = 0.0f;
    Vector3 sunlight3Color{1.0f, 1.0f, 1.0f};
    int     dirt = -1;
    int     sunlightDirt = -2;
    int     sunlight2Dirt = -2;
    int     dirtMode = 0;
    float   dirtDepth = 128.0f;
    float   dirtScale = 1.0f;
    float   dirtGain = 1.0f;
    float   dirtAngle = 88.0f;
    int     lmAAScale = 0;
};

// --------------------------------------------------------------------------
//  CPU-side render geometry (fed to Renderer_UploadMap / written to .bsp)
// --------------------------------------------------------------------------
struct MapVertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;       // diffuse UV
    float lu, lv;     // lightmap UV (0 if unlit / legacy path)
};

struct MapMeshBucket {
    std::string            texture;
    uint32_t               lightmapPage = 0;
    std::vector<MapVertex> vertices;
    std::vector<uint32_t>  indices;
};

// --------------------------------------------------------------------------
//  Renderer-agnostic polygon output (used by the offline .bsp compiler —
//  no TextureManager / sokol dependency).
// --------------------------------------------------------------------------
struct MapPolygon {
    std::vector<Vector3> verts;      // world-space (GL coords), CCW, ≥3
    Vector3              normal;     // world-space
    std::string          texture;
    int                  occluderGroup = -1;
    int                  sourceBrushId = -1;
    int                  sourceEntityId = -1;
    int                  sourceFaceIndex = -1;
    int                  surfaceLightGroup = 0;
    uint8_t              phong = 0;
    float                phongAngle = 89.0f;
    float                phongAngleConcave = 0.0f;
    int                  phongGroup = 0;
    // Texture projection params (already converted to GL axes):
    Vector3 texAxisU, texAxisV;
    float   offU, offV;
    float   rot, scaleU, scaleV;
};

// --------------------------------------------------------------------------
//  Public API
// --------------------------------------------------------------------------
Map                        ParseMapFile(const std::string& filePath);
std::vector<PlayerStart>   GetPlayerStarts(const Map& map);
std::vector<PointLight>    GetPointLights(const Map& map);
std::vector<SurfaceLightTemplate> GetSurfaceLightTemplates(const Map& map);
LightBakeSettings          GetLightBakeSettings(const Map& map);
std::vector<MapPolygon>    BuildMapPolygons(const Map& map, bool devMode);
std::vector<MapPolygon>    BuildExteriorMapPolygons(const Map& map, bool devMode);

// Legacy path: builds GPU-ready buckets directly (still used until a
// .bsp is available).  Pulls texture dims from TextureManager.
std::vector<MapMeshBucket> BuildMapGeometry(const Map& map, TextureManager& textureManager);

// Compute diffuse UV for a world-space vertex given face projection params.
// texW/texH normalise into [0..1]; pass 0 to skip normalisation.
Vector2 ComputeFaceUV(const Vector3& vert,
                      const Vector3& axisU, const Vector3& axisV,
                      float offU, float offV, float rot,
                      float scaleU, float scaleV,
                      float texW, float texH);

// Geometry helpers (reused by collision_data.cpp)
Vector3 ConvertTBtoRaylib(const Vector3& in);
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3);
void    RemoveDuplicatePoints(std::vector<Vector3>& points, float eps);
bool    GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& out);
void    SortPolygonVertices(std::vector<Vector3>& poly, const Vector3& normal);
