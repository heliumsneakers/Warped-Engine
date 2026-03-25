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

struct PointLight {
    Vector3 position;     // world-space (GL coords)
    Vector3 color;        // 0..1
    float   intensity;    // radius in world units
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
std::vector<MapPolygon>    BuildMapPolygons(const Map& map, bool devMode);

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
