// map_parser.h
#ifndef MAP_PARSER_H
#define MAP_PARSER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <raylib.h>

// ------------------ Data Structures ------------------ //

// Structure to represent a vertex
struct Vertex {
    Vector3 position; // Position of the vertex
    Vector3 normal;   // Normal vector
    Vector2 texcoord; // Texture coordinates
};

// Structure to represent a face
struct Face {
    std::vector<Vector3> vertices; // List of vertices (should have exactly 3 for a triangle)
    std::string texture;           // Texture name
    Vector2 textureAxes1;          // First texture axis
    Vector2 textureAxes2;          // Second texture axis
    int someAttribute1;            // Placeholder for other attributes
    int someAttribute2;
    int someAttribute3;
};

// Structure to represent a brush
struct Brush {
    std::vector<Face> faces; // List of faces in the brush
};

// Structure to represent an entity
struct Entity {
    std::unordered_map<std::string, std::string> properties; // Key-value properties of the entity
    std::vector<Brush> brushes;                             // List of brushes in the entity
};

// Structure to represent the entire map
struct Map {
    std::vector<Entity> entities; // List of entities in the map
};

// Structure to hold player start information
struct PlayerStart {
    Vector3 position; // Position where the player starts
};

// ------------------ Texture Manager Structure ------------------ //

// Structure to manage textures
struct TextureManager {
    std::unordered_map<std::string, Texture2D> textures; // Cache of loaded textures
};

// ------------------ Parser Functions ------------------ //

// Function to parse the .map file and return a Map structure
Map ParseMapFile(const std::string& filename);

// Function to extract player start positions from the parsed map
std::vector<PlayerStart> GetPlayerStarts(const Map& map);

// Function to convert the parsed map into a Raylib Mesh
Mesh MapToMesh(const Map& map, TextureManager& textureManager);

// TextureManager management functions

// Initialize the TextureManager
void InitTextureManager(TextureManager& manager);

// Load texture by name using the TextureManager
Texture2D LoadTextureByName(TextureManager& manager, const std::string& textureName);

// Unload all textures from the TextureManager
void UnloadAllTextures(TextureManager& manager);

#endif // MAP_PARSER_H
