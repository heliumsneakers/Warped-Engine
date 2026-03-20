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

// --------------------------------------------------------------------------
//  CPU-side render geometry (fed to Renderer_UploadMap)
// --------------------------------------------------------------------------
struct MapVertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

struct MapMeshBucket {
    std::string            texture;
    std::vector<MapVertex> vertices;
    std::vector<uint32_t>  indices;
};

// --------------------------------------------------------------------------
//  Public API
// --------------------------------------------------------------------------
Map                       ParseMapFile(const std::string& filePath);
std::vector<PlayerStart>  GetPlayerStarts(const Map& map);
std::vector<MapMeshBucket> BuildMapGeometry(const Map& map, TextureManager& textureManager);

// Geometry helpers (reused by collision_data.cpp)
Vector3 ConvertTBtoRaylib(const Vector3& in);
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3);
void    RemoveDuplicatePoints(std::vector<Vector3>& points, float eps);
bool    GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& out);
void    SortPolygonVertices(std::vector<Vector3>& poly, const Vector3& normal);
