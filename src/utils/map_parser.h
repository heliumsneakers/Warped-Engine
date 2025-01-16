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

typedef struct TextureManager {
    std::unordered_map<std::string, Texture2D> textures;
} TextureManager;

void InitTextureManager(TextureManager& manager);
void UnloadAllTextures(TextureManager& manager);
Map ParseMapFile(const std::string& filePath);
std::vector<PlayerStart> GetPlayerStarts(const Map& map);
Texture2D LoadTextureByName(TextureManager& manager, const std::string& textureName);
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3);
Model MapToMesh(const Map& map, TextureManager& textureManager);
