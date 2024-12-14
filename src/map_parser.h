// map_parser.h
#pragma once
#include "raylib.h"
#include <vector>
#include <string>
#include <unordered_map>

typedef struct Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 texcoord;
} Vertex;

typedef struct Face {
    std::string texture;
    Vector3 textureAxes1; // [ux, uy, uz]
    float offsetX;
    Vector3 textureAxes2; // [vx, vy, vz]
    float offsetY;
    float rotation;
    float scaleX;
    float scaleY;
    int someAttribute1;
    int someAttribute2;
    int someAttribute3;
    std::vector<Vector3> vertices;
} Face;

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

typedef struct TextureManager {
    std::unordered_map<std::string, Texture2D> textures;
} TextureManager;

// Function declarations
void InitTextureManager(TextureManager& manager);
void UnloadAllTextures(TextureManager& manager);
Map ParseMapFile(const std::string& filePath);
std::vector<PlayerStart> GetPlayerStarts(const Map& map);
Texture2D LoadTextureByName(TextureManager& manager, const std::string& textureName);
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3);
Model MapToMesh(const Map& map, TextureManager& textureManager);
