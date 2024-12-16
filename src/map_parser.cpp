// map_parser.cpp
#include "map_parser.h"
#include "raymath.h"
#include "raylib.h"
#include "rlgl.h"
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdio>
#include <cstdlib> // For malloc and free
#include <unordered_map>
#include <vector>
#include <string>
#include <cmath> // For fabs

const double epsilon = 1e-5f;

// ------------------ Helper Functions ------------------ //

// Trim function to remove leading nd trailing whitespace
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
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
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

// ------------------ Intersection Function ------------------ //


// Function to calculate the intersection point of three planes
bool GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& intersection) {
    Vector3 cross23 = Vector3CrossProduct(p2.normal, p3.normal);
    float denom = Vector3DotProduct(p1.normal, cross23);

    if (fabs(denom) < epsilon) {
        // The planes do not intersect at a single point
        return false;
    }

    Vector3 term1 = Vector3Scale(cross23, -p1.d); // -d1 * (n2 × n3)
    Vector3 cross31 = Vector3CrossProduct(p3.normal, p1.normal);
    Vector3 term2 = Vector3Scale(cross31, -p2.d); // -d2 * (n3 × n1)
    Vector3 cross12 = Vector3CrossProduct(p1.normal, p2.normal);
    Vector3 term3 = Vector3Scale(cross12, -p3.d); // -d3 * (n1 × n2)

    Vector3 numerator = Vector3Add(Vector3Add(term1, term2), term3);
    intersection = Vector3Scale(numerator, 1.0f / denom);

    printf("Computed Intersection Point: [%f, %f, %f]\n", intersection.x, intersection.y, intersection.z);

    return true;
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

                // Parse texture axes and other attributes
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
                                Vector3 axis = { std::stof(axes[0]), std::stof(axes[1]), std::stof(axes[2]) };
                                float offset = std::stof(axes[3]);
                                if (i == 0) {
                                    face.textureAxes1 = axis;
                                    face.offsetX = offset;
                                    printf("Texture Axes 1: [%f, %f, %f], OffsetX: %f\n", 
                                           axis.x, axis.y, axis.z, face.offsetX);
                                } else {
                                    face.textureAxes2 = axis;
                                    face.offsetY = offset;
                                    printf("Texture Axes 2: [%f, %f, %f], OffsetY: %f\n", 
                                           axis.x, axis.y, axis.z, face.offsetY);
                                }
                            } catch (const std::invalid_argument& e) {
                                printf("Invalid texture axes: %s\n", axesStr.c_str());
                                // Assign default texture axes or handle the error
                                if (i == 0) {
                                    face.textureAxes1 = { 0.0f, 0.0f, 0.0f };
                                    face.offsetX = 0.0f;
                                } else {
                                    face.textureAxes2 = { 0.0f, 0.0f, 0.0f };
                                    face.offsetY = 0.0f;
                                }
                            }
                        } else {
                            printf("Unexpected number of values in texture axes: %s\n", token.c_str());
                            // Assign default texture axes or handle the error
                            if (i == 0) {
                                face.textureAxes1 = { 0.0f, 0.0f, 0.0f };
                                face.offsetX = 0.0f;
                            } else {
                                face.textureAxes2 = { 0.0f, 0.0f, 0.0f };
                                face.offsetY = 0.0f;
                            }
                        }
                    }
                }

                // Parse rotation, scaleX, scaleY
                iss >> token;
                if (!token.empty()) {
                    try {
                        face.rotation = std::stof(token);
                    } catch (const std::invalid_argument& e) {
                        printf("Invalid rotation value: %s\n", token.c_str());
                        face.rotation = 0.0f;
                    }
                }

                iss >> token;
                if (!token.empty()) {
                    try {
                        face.scaleX = std::stof(token);
                    } catch (const std::invalid_argument& e) {
                        printf("Invalid scaleX value: %s\n", token.c_str());
                        face.scaleX = 1.0f;
                    }
                }

                iss >> token;
                if (!token.empty()) {
                    try {
                        face.scaleY = std::stof(token);
                    } catch (const std::invalid_argument& e) {
                        printf("Invalid scaleY value: %s\n", token.c_str());
                        face.scaleY = 1.0f;
                    }
                }

                printf("Parsed Face Rotation: %f, ScaleX: %f, ScaleY: %f\n", face.rotation, face.scaleX, face.scaleY);

                // Calculate and assign the normal vector
                if (face.vertices.size() >= 3) {
                    face.normal = CalculateNormal(face.vertices[0], face.vertices[1], face.vertices[2]);
                    printf("Calculated Face Normal: [%f, %f, %f]\n", face.normal.x, face.normal.y, face.normal.z);
                } else {
                    printf("Cannot calculate normal for face with fewer than 3 vertices.\n");
                    face.normal = {0.0f, 0.0f, 0.0f}; // Assign a default normal
                }

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
}

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


// Convert Map to Raylib Model with Multiple Meshes and Materials using Intersection Method
Model MapToMesh(const Map& map, TextureManager& textureManager) {
    // Temporary storage for meshes per texture
    struct TextureMeshData {
        std::vector<Vector3> vertices;
        std::vector<Vector3> normals;
        std::vector<Vector2> texcoords;
        std::vector<unsigned short> indices;
    };
    
    std::unordered_map<std::string, TextureMeshData> textureMeshesMap;

    // Iterate through all entities, brushes, and faces
    for (const auto& entity : map.entities) {
        for (const auto& brush : entity.brushes) {
            const int numFaces = brush.faces.size();
            if (numFaces < 3) {
                printf("Brush has fewer than 3 faces. Skipping.\n");
                continue;
            }

            // Convert brush faces to planes
            std::vector<Plane> planes;
            for (const auto& face : brush.faces) {
                Plane plane;
                plane.normal = face.normal; // Ensure normals are normalized
                plane.d = Vector3DotProduct(face.normal, face.vertices[0]);
                planes.push_back(plane);
            }

            // Generate polygons by intersecting three planes
            // One polygon per face
            std::vector<std::vector<Vector3>> polys(numFaces, std::vector<Vector3>());

            for (int i = 0; i < numFaces - 2; ++i) {
                for (int j = i + 1; j < numFaces - 1; ++j) {
                    for (int k = j + 1; k < numFaces; ++k) {
                        Vector3 intersectionPoint;
                        if (GetIntersection(planes[i], planes[j], planes[k], intersectionPoint)) {
                            bool isInside = true;
                            for (int m = 0; m < numFaces; ++m) {
                                // If point is in front of any plane, it's outside the convex brush
                                float distance = Vector3DotProduct(brush.faces[m].normal, intersectionPoint) + planes[m].d;
                                printf("Plane %d: distance = %f\n", m, distance);
                                if (distance > epsilon) { // Adjust epsilon as needed
                                    isInside = false;
                                    printf("DISTANCE > EPSILON\n");
                                    break;
                                }
                            }
                            if (isInside) {
                                printf("Adding intersection point [%f, %f, %f] to polygons %d, %d, %d.\n", 
                                intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, i, j, k);
                                polys[i].push_back(intersectionPoint);
                                polys[j].push_back(intersectionPoint);
                                polys[k].push_back(intersectionPoint);
                            }   else {
                                printf("Intersection Point: [%f, %f, %f] is outside brush.\n", 
                                       intersectionPoint.x, intersectionPoint.y, intersectionPoint.z);
                            }
                        }
                    }
                } 
            }

            
    
            // Now, for each polygon, triangulate and add to mesh
            for (int i = 0; i < numFaces; ++i) {
                if (polys[i].size() < 3) {
                        printf("Skipping polygon %d due to insufficient vertices (%zu).\n", i, polys[i].size());
                        continue; // Not enough points to form a polygon
                }

                printf("Triangulating polygon %d with %zu vertices.\n", i, polys[i].size());

                // Use the first vertex as the fan's center
                Vector3 v0 = polys[i][0];

                for (size_t t = 1; t + 1 < polys[i].size(); ++t) {
                        Vector3 v1 = polys[i][t];
                        Vector3 v2 = polys[i][t + 1];

                        printf("Adding triangle: [%f, %f, %f] - [%f, %f, %f] - [%f, %f, %f]\n",
                                   v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);

                        // Load texture
                        std::string textureName = brush.faces[i].texture;
                        Texture2D texture = LoadTextureByName(textureManager, textureName);
                        if (texture.id == 0) {
                                // Fallback to default texture
                                auto defaultIt = textureManager.textures.find("default");
                                if (defaultIt != textureManager.textures.end()) {
                                                texture = defaultIt->second;
                                                textureName = "default"; // Use default texture name
                                                printf("Using default texture for face with missing texture '%s'.\n", brush.faces[i].texture.c_str());
                                } else {
                                                printf("No valid texture available for face. Skipping.\n");
                                                continue;
                                }
                        }

                        // Initialize texture mesh data if not present
                        if (textureMeshesMap.find(textureName) == textureMeshesMap.end()) {
                                textureMeshesMap[textureName] = TextureMeshData();
                                printf("Initialized TextureMeshData for texture '%s'.\n", textureName.c_str());
                        }

                        // Compute texture coordinates
                        auto ComputeUV = [&](const Vector3& vertex) -> Vector2 {
                                float u = Vector3DotProduct(vertex, brush.faces[i].textureAxes1) + brush.faces[i].offsetX;
                                float v = Vector3DotProduct(vertex, brush.faces[i].textureAxes2) + brush.faces[i].offsetY;
                                u /= static_cast<float>(texture.width);
                                v /= static_cast<float>(texture.height);
                                return Vector2{ u, v };
                        };

                        // Add vertices, normals, texcoords
                        size_t baseVertexIndex = textureMeshesMap[textureName].vertices.size();
                        textureMeshesMap[textureName].vertices.push_back(v0);
                        textureMeshesMap[textureName].normals.push_back(brush.faces[i].normal);
                        textureMeshesMap[textureName].texcoords.push_back(ComputeUV(v0));

                        textureMeshesMap[textureName].vertices.push_back(v1);
                        textureMeshesMap[textureName].normals.push_back(brush.faces[i].normal);
                        textureMeshesMap[textureName].texcoords.push_back(ComputeUV(v1));

                        textureMeshesMap[textureName].vertices.push_back(v2);
                        textureMeshesMap[textureName].normals.push_back(brush.faces[i].normal);
                        textureMeshesMap[textureName].texcoords.push_back(ComputeUV(v2));

                        // Add indices
                        textureMeshesMap[textureName].indices.push_back(static_cast<unsigned short>(baseVertexIndex));
                        textureMeshesMap[textureName].indices.push_back(static_cast<unsigned short>(baseVertexIndex + 1));
                        textureMeshesMap[textureName].indices.push_back(static_cast<unsigned short>(baseVertexIndex + 2));
                }
            }

            printf("Finished processing brush with %d faces.\n", numFaces);
        }
    }

    // Create separate meshes for each texture
    std::vector<Mesh> meshes;
    std::vector<Texture2D> textures;

    for (const auto& pair : textureMeshesMap) {
        const std::string& textureName = pair.first;
        const TextureMeshData& meshData = pair.second;

        if (meshData.indices.empty()) {
            printf("No indices for texture '%s'. Skipping mesh creation.\n", textureName.c_str());
            continue;
        }

        Mesh mesh = { 0 };
        mesh.vertexCount = static_cast<int>(meshData.vertices.size());
        mesh.triangleCount = static_cast<int>(meshData.indices.size() / 3);
        mesh.vertices = (float*)malloc(sizeof(float) * mesh.vertexCount * 3);
        mesh.normals = (float*)malloc(sizeof(float) * mesh.vertexCount * 3);
        mesh.texcoords = (float*)malloc(sizeof(float) * mesh.vertexCount * 2);
        mesh.indices = (unsigned short*)malloc(sizeof(unsigned short) * meshData.indices.size());

        if (!mesh.vertices || !mesh.normals || !mesh.texcoords || !mesh.indices) {
            printf("Memory allocation failed for mesh with texture '%s'.\n", textureName.c_str());
            RL_FREE(mesh.vertices);
            RL_FREE(mesh.normals);
            RL_FREE(mesh.texcoords);
            RL_FREE(mesh.indices);
            continue;
        }

        // Populate vertex data
        for (int i = 0; i < mesh.vertexCount; ++i) {
            mesh.vertices[i * 3 + 0] = meshData.vertices[i].x;
            mesh.vertices[i * 3 + 1] = meshData.vertices[i].y;
            mesh.vertices[i * 3 + 2] = meshData.vertices[i].z;

            mesh.normals[i * 3 + 0] = meshData.normals[i].x;
            mesh.normals[i * 3 + 1] = meshData.normals[i].y;
            mesh.normals[i * 3 + 2] = meshData.normals[i].z;

            mesh.texcoords[i * 2 + 0] = meshData.texcoords[i].x;
            mesh.texcoords[i * 2 + 1] = meshData.texcoords[i].y;
        }

        // Populate index data
        for (size_t i = 0; i < meshData.indices.size(); ++i) {
            mesh.indices[i] = meshData.indices[i];
        }

        // Upload mesh data to GPU
        UploadMesh(&mesh, false); // 'false' indicates the mesh isn't dynamic
        printf("Uploaded mesh for texture '%s' to GPU.\n", textureName.c_str());

        meshes.push_back(mesh);
        textures.push_back(textureManager.textures.at(textureName));

        printf("Created mesh for texture '%s' with %d vertices and %d triangles.\n",
               textureName.c_str(), mesh.vertexCount, mesh.triangleCount);

        // Free the CPU-side mesh data as it's now on the GPU
        RL_FREE(mesh.vertices);
        RL_FREE(mesh.normals);
        RL_FREE(mesh.texcoords);
        RL_FREE(mesh.indices);
    }

    // Create a Model with multiple meshes and materials
    Model model = { 0 };
    model.transform = MatrixIdentity();

    model.meshCount = static_cast<int>(meshes.size());
    model.materialCount = static_cast<int>(textures.size());

    if (model.meshCount > 0 && model.materialCount > 0) {
        model.meshes = (Mesh*)malloc(sizeof(Mesh) * model.meshCount);
        model.materials = (Material*)malloc(sizeof(Material) * model.materialCount);
        model.meshMaterial = (int*)malloc(sizeof(int) * model.meshCount);

        for (int i = 0; i < model.meshCount; ++i) {
            model.meshes[i] = meshes[i];
            model.meshMaterial[i] = i; // Assign each mesh to its corresponding material
        }

        for (int i = 0; i < model.materialCount; ++i) {
            model.materials[i] = LoadMaterialDefault(); // Initialize with default material
            model.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture = textures[i];
        }

        printf("Model created with %d meshes and %d materials.\n", model.meshCount, model.materialCount);
    } else {
        printf("No meshes or textures available to create the model.\n");
    } 
    return model;
}

