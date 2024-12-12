// map_parser.cpp
#include "map_parser.h"
#include "raymath.h"
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdio>
#include <cstdlib> // For malloc and free
#include <unordered_map>

// ------------------ Helper Functions ------------------ //

// Trim function to remove leading and trailing whitespace
std::string Trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

// Split a string by spaces into tokens
std::vector<std::string> SplitBySpace(const std::string& s) {
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

// Calculate normal vector for a triangle
Vector3 CalculateNormal(Vector3 v1, Vector3 v2, Vector3 v3) {
    Vector3 edge1 = Vector3Subtract(v2, v1);
    Vector3 edge2 = Vector3Subtract(v3, v1);
    Vector3 normal = Vector3CrossProduct(edge1, edge2);
    return Vector3Normalize(normal);
}

// ------------------ TextureManager Functions ------------------ //

// Initialize the TextureManager
void InitTextureManager(TextureManager& manager) {
    manager.textures.clear();
    printf("TextureManager initialized and cleared.\n");
}

// Load texture by name using the TextureManager
Texture2D LoadTextureByName(TextureManager& manager, const std::string& textureName) {
    // Check if texture is already loaded
    auto it = manager.textures.find(textureName);
    if (it != manager.textures.end()) {
        printf("Texture '%s' already loaded.\n", textureName.c_str());
        return it->second;
    } else {
        // Construct file path (adjust based on your assets directory structure)
        std::string filePath = "../../assets/textures/" + textureName + ".png"; // Assuming PNG format
        Texture2D tex = LoadTexture(filePath.c_str());
        if (tex.id != 0) { // Check if texture loaded successfully
            manager.textures[textureName] = tex;
            printf("Successfully loaded texture: %s\n", filePath.c_str());
        } else {
            printf("Failed to load texture: %s\n", filePath.c_str());
            // Load default texture
            std::string defaultPath = "../../assets/textures/default.png";
            tex = LoadTexture(defaultPath.c_str());
            if (tex.id != 0) {
                manager.textures["default"] = tex;
                printf("Loaded default texture: %s\n", defaultPath.c_str());
            } else {
                printf("Failed to load default texture: %s\n", defaultPath.c_str());
            }
        }
        return tex;
    }
}

// Unload all textures from the TextureManager
void UnloadAllTextures(TextureManager& manager) {
    for (auto& pair : manager.textures) {
        UnloadTexture(pair.second);
        printf("Unloaded texture: %s\n", pair.first.c_str());
    }
    manager.textures.clear();
    printf("All textures unloaded and TextureManager cleared.\n");
}

// ------------------ Parser Functions ------------------ //

// Function to parse the .map file and return a Map structure
Map ParseMapFile(const std::string& filename) {
    Map map;
    std::ifstream file(filename);
    if (!file.is_open()) {
        // Handle error
        printf("Failed to open file: %s\n", filename.c_str());
        return map;
    }

    std::string line;
    Entity currentEntity;
    Brush currentBrush;
    bool inEntity = false;
    bool inBrush = false;
    int entityCount = 0;
    int brushCount = 0;
    int faceCount = 0;

    while (std::getline(file, line)) {
        line = Trim(line);

        if (line.empty() || line[0] == '/') {
            // Skip empty lines and comments
            continue;
        }

        if (line == "{") {
            if (!inEntity) {
                // Start of a new entity
                currentEntity = Entity();
                inEntity = true;
                printf("Started parsing a new entity.\n");
            } else if (!inBrush) {
                // Start of a new brush within the current entity
                currentBrush = Brush();
                inBrush = true;
                brushCount++;
                printf("Started parsing a new brush.\n");
            }
            continue;
        }

        if (line == "}") {
            if (inBrush) {
                // End of brush
                currentEntity.brushes.push_back(currentBrush); // Correctly add brush to entity
                inBrush = false;
                printf("Finished parsing a brush. Total brushes: %d\n", brushCount);
            } else if (inEntity) {
                // End of entity
                map.entities.push_back(currentEntity); // Correctly add entity to map
                inEntity = false;
                entityCount++;
                printf("Finished parsing an entity. Total entities: %d\n", entityCount);
            }
            continue;
        }

        if (inEntity && !inBrush) {
            // Parsing entity properties
            size_t firstQuote = line.find('"');
            if (firstQuote != std::string::npos) {
                size_t secondQuote = line.find('"', firstQuote + 1);
                if (secondQuote != std::string::npos) {
                    std::string key = line.substr(firstQuote + 1, secondQuote - firstQuote - 1);
                    size_t thirdQuote = line.find('"', secondQuote + 1);
                    if (thirdQuote != std::string::npos) {
                        size_t fourthQuote = line.find('"', thirdQuote + 1);
                        if (fourthQuote != std::string::npos) {
                            std::string value = line.substr(thirdQuote + 1, fourthQuote - thirdQuote - 1);
                            currentEntity.properties[key] = value;
                            printf("Entity Property: %s = %s\n", key.c_str(), value.c_str());
                        }
                    }
                }
            }
            continue;
        }

        if (inBrush) {
            // Parsing brush faces
            // Detect start of a face
            if (line.find('(') != std::string::npos) {
                // Start accumulating face tokens
                std::string faceLine = line;
                // Check if face is completed on the same line
                if (line.find(')') == std::string::npos) {
                    // Read additional lines until ')' is found
                    while (std::getline(file, line)) {
                        line = Trim(line);
                        faceLine += " " + line;
                        if (line.find(')') != std::string::npos) {
                            break;
                        }
                    }
                }

                // Now, parse faceLine as a complete face
                Face face;
                std::istringstream iss(faceLine);
                std::string token;

                // Parse three vertices
                for (int i = 0; i < 3; ++i) {
                    iss >> token;
                    if (token.empty()) continue;
                    if (token.front() == '(') {
                        size_t closingParen = token.find(')');
                        std::string vertexStr;
                        if (closingParen != std::string::npos) {
                            vertexStr = token.substr(1, closingParen - 1);
                        } else {
                            vertexStr = token.substr(1);
                            // Read until ')' is found
                            while (iss >> token && token.find(')') == std::string::npos) {
                                vertexStr += " " + token;
                            }
                            if (token.find(')') != std::string::npos) {
                                vertexStr += " " + token.substr(0, token.find(')'));
                            }
                        }
                        std::vector<std::string> coords = SplitBySpace(vertexStr);
                        if (coords.size() == 3) {
                            try {
                                Vector3 vertex = { std::stof(coords[0]), std::stof(coords[1]), std::stof(coords[2]) };
                                face.vertices.push_back(vertex);
                            } catch (const std::invalid_argument& e) {
                                printf("Invalid vertex coordinates: %s\n", vertexStr.c_str());
                                face.vertices.clear();
                                break;
                            }
                        }
                    }
                }

                if (face.vertices.size() < 3) {
                    printf("Skipped a face due to insufficient vertices.\n");
                    continue;
                }

                // Parse texture
                iss >> token;
                face.texture = token;
                printf("Parsed face with texture: %s\n", face.texture.c_str());

                // Parse texture axes
                for (int i = 0; i < 2; ++i) {
                    iss >> token;
                    if (token.empty()) continue;
                    if (token.front() == '[') {
                        // Collect tokens until ']' is found
                        std::string axesStr = token.substr(1);
                        if (token.find(']') == std::string::npos) {
                            while (iss >> token) {
                                axesStr += " " + token;
                                if (token.find(']') != std::string::npos) {
                                    axesStr = axesStr.substr(0, axesStr.find(']'));
                                    break;
                                }
                            }
                        }
                        std::vector<std::string> axes = SplitBySpace(axesStr);
                        if (axes.size() == 4) {
                            try {
                                Vector2 axis1 = { std::stof(axes[0]), std::stof(axes[1]) };
                                Vector2 axis2 = { std::stof(axes[2]), std::stof(axes[3]) };
                                if (i == 0) {
                                    face.textureAxes1 = axis1;
                                } else {
                                    face.textureAxes2 = axis2;
                                }
                                printf("Texture Axes %d: [%f, %f], [%f, %f]\n", 
                                       i + 1, axis1.x, axis1.y, axis2.x, axis2.y);
                            } catch (const std::invalid_argument& e) {
                                printf("Invalid texture axes: %s\n", axesStr.c_str());
                                // Assign default texture axes or handle the error
                                face.textureAxes1 = { 0.0f, 0.0f };
                                face.textureAxes2 = { 0.0f, 0.0f };
                            }
                        } else {
                            printf("Unexpected number of values in texture axes: %s\n", token.c_str());
                        }
                    }
                }

                // Parse remaining attributes
                for (int i = 0; i < 3; ++i) {
                    iss >> token;
                    if (token.empty()) {
                        printf("Missing attribute in face.\n");
                        face.someAttribute1 = 0;
                        face.someAttribute2 = 0;
                        face.someAttribute3 = 0;
                        break;
                    }
                    try {
                        if (i == 0) {
                            face.someAttribute1 = std::stoi(token);
                        } else if (i == 1) {
                            face.someAttribute2 = std::stoi(token);
                        } else if (i == 2) {
                            face.someAttribute3 = std::stoi(token);
                        }
                    } catch (const std::invalid_argument& e) {
                        printf("Invalid attribute in face: %s\n", token.c_str());
                        if (i == 0) face.someAttribute1 = 0;
                        if (i <=1 ) face.someAttribute2 = 0;
                        face.someAttribute3 = 0;
                    }
                }
                printf("Face Attributes: %d, %d, %d\n", face.someAttribute1, face.someAttribute2, face.someAttribute3);

                // Add face to current brush
                currentBrush.faces.push_back(face);
                faceCount++;
                printf("Added face %d to brush %d.\n", faceCount, brushCount);
            }
        }

    }
    file.close();
    printf("Finished parsing map. Total Entities: %d, Total Brushes: %d, Total Faces: %d\n", entityCount, brushCount, faceCount);
    return map;
} // <-- Closing brace for ParseMapFile function

// Function to extract player start positions from the parsed map
std::vector<PlayerStart> GetPlayerStarts(const Map& map) {
    std::vector<PlayerStart> playerStarts;
    for (const auto& entity : map.entities) {
        auto it = entity.properties.find("classname");
        if (it != entity.properties.end() && it->second == "info_player_start") {
            auto originIt = entity.properties.find("origin");
            if (originIt != entity.properties.end()) {
                std::vector<std::string> coords = SplitBySpace(originIt->second);
                if (coords.size() == 3) {
                    try {
                        Vector3 position = { std::stof(coords[0]), std::stof(coords[1]), std::stof(coords[2]) };
                        playerStarts.push_back(PlayerStart{ position });
                        printf("Player Start Position: (%f, %f, %f)\n", position.x, position.y, position.z);
                    } catch (const std::invalid_argument& e) {
                        printf("Invalid origin coordinates for player start.\n");
                        // Handle the error as needed
                    }
                } else {
                    printf("Unexpected number of origin coordinates: %zu\n", coords.size());
                }
            } else {
                printf("Entity with 'info_player_start' missing 'origin' property.\n");
            }
        }
    }
    return playerStarts;
}

// Function to convert the parsed map into a Raylib Mesh
Mesh MapToMesh(const Map& map, TextureManager& textureManager) {
    // Structure to hold vertices and indices per texture
    struct TextureMeshData {
        std::vector<Vertex> vertices;
        std::vector<unsigned short> indices;
    };

    // Map from texture name to its mesh data
    std::unordered_map<std::string, TextureMeshData> textureMeshesMap;

    for (const auto& entity : map.entities) {
        for (const auto& brush : entity.brushes) {
            for (const auto& face : brush.faces) {
                // Assuming each face is a triangle
                if (face.vertices.size() < 3)
                    continue; // Not enough vertices for a face

                // Calculate normal
                Vector3 normal = CalculateNormal(face.vertices[0], face.vertices[1], face.vertices[2]);

                // Get texture
                std::string textureName = face.texture;
                Texture2D texture = LoadTextureByName(textureManager, textureName);
                if (texture.id == 0) {
                    // Handle missing texture (assign default texture if available)
                    if (textureManager.textures.find("default") != textureManager.textures.end()) {
                        texture = textureManager.textures["default"];
                        printf("Using default texture for face with missing texture '%s'.\n", textureName.c_str());
                    } else {
                        printf("No valid texture available for face. Skipping.\n");
                        continue;
                    }
                } else {
                    printf("Using texture '%s' for face.\n", textureName.c_str());
                }

                // Initialize if texture not present
                if (textureMeshesMap.find(textureName) == textureMeshesMap.end()) {
                    textureMeshesMap[textureName] = TextureMeshData();
                }

                // Calculate UV coordinates based on texture axes
                for (const auto& vertexPos : face.vertices) {
                    Vector2 uv;
                    // Project the vertex position onto the texture axes
                    uv.x = Vector3DotProduct(vertexPos, (Vector3){ face.textureAxes1.x, face.textureAxes1.y, 0.0f }) * 0.1f; // Scaling factor
                    uv.y = Vector3DotProduct(vertexPos, (Vector3){ face.textureAxes2.x, face.textureAxes2.y, 0.0f }) * 0.1f;

                    Vertex vertex;
                    vertex.position = vertexPos;
                    vertex.normal = normal;
                    vertex.texcoord = uv;
                    textureMeshesMap[textureName].vertices.push_back(vertex);
                }

                // Add indices
                unsigned short base = static_cast<unsigned short>(textureMeshesMap[textureName].vertices.size()) - 3;
                textureMeshesMap[textureName].indices.push_back(base);
                textureMeshesMap[textureName].indices.push_back(base + 1);
                textureMeshesMap[textureName].indices.push_back(base + 2);

                printf("Added vertices and indices for face to texture '%s'.\n", textureName.c_str());
            }
        }
    }

    // Combine all texture meshes into a single mesh (for simplicity)
    Mesh combinedMesh = { 0 };
    std::vector<Vertex> combinedVertices;
    std::vector<unsigned short> combinedIndices;
    unsigned short indexOffset = 0;

    for (const auto& pair : textureMeshesMap) {
        const std::string& textureName = pair.first;
        const TextureMeshData& meshData = pair.second;

        // Append vertices
        combinedVertices.insert(combinedVertices.end(), meshData.vertices.begin(), meshData.vertices.end());
        printf("Appended %zu vertices from texture '%s'.\n", meshData.vertices.size(), textureName.c_str());

        // Append indices with offset
        for (const auto& idx : meshData.indices) {
            combinedIndices.push_back(idx + indexOffset);
        }
        printf("Appended %zu indices from texture '%s'.\n", meshData.indices.size(), textureName.c_str());

        indexOffset += static_cast<unsigned short>(meshData.vertices.size());
    }

    // Assign to mesh
    combinedMesh.vertexCount = static_cast<int>(combinedVertices.size());
    combinedMesh.vertices = (float*)malloc(sizeof(float) * combinedMesh.vertexCount * 3);
    combinedMesh.texcoords = (float*)malloc(sizeof(float) * combinedMesh.vertexCount * 2);
    combinedMesh.normals = (float*)malloc(sizeof(float) * combinedMesh.vertexCount * 3);
    combinedMesh.indices = (unsigned short*)malloc(sizeof(unsigned short) * combinedIndices.size());
    combinedMesh.triangleCount = static_cast<int>(combinedIndices.size() / 3);

    if (!combinedMesh.vertices || !combinedMesh.texcoords || !combinedMesh.normals || !combinedMesh.indices) {
        printf("Memory allocation failed for combined mesh.\n");
        // Handle allocation failure (e.g., free allocated memory and return an empty mesh)
        free(combinedMesh.vertices);
        free(combinedMesh.texcoords);
        free(combinedMesh.normals);
        free(combinedMesh.indices);
        combinedMesh.vertexCount = 0;
        combinedMesh.triangleCount = 0;
        return combinedMesh;
    }

    for (size_t i = 0; i < combinedVertices.size(); ++i) {
        combinedMesh.vertices[i * 3 + 0] = combinedVertices[i].position.x;
        combinedMesh.vertices[i * 3 + 1] = combinedVertices[i].position.y;
        combinedMesh.vertices[i * 3 + 2] = combinedVertices[i].position.z;

        combinedMesh.texcoords[i * 2 + 0] = combinedVertices[i].texcoord.x;
        combinedMesh.texcoords[i * 2 + 1] = combinedVertices[i].texcoord.y;

        combinedMesh.normals[i * 3 + 0] = combinedVertices[i].normal.x;
        combinedMesh.normals[i * 3 + 1] = combinedVertices[i].normal.y;
        combinedMesh.normals[i * 3 + 2] = combinedVertices[i].normal.z;
    }

    for (size_t i = 0; i < combinedIndices.size(); ++i) {
        combinedMesh.indices[i] = combinedIndices[i];
    }

    printf("Combined Mesh: %d vertices, %d triangles\n", combinedMesh.vertexCount, combinedMesh.triangleCount);

    return combinedMesh;
}
