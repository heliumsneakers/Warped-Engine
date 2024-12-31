// map_parser.cpp
#include "map_parser.h"
#include "raymath.h"
#include "raylib.h"
#include "rlgl.h"
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdio>
#include <cstdlib> 
#include <unordered_map>
#include <vector>
#include <string>
#include <cmath> 
#include <algorithm> 


/*
 * NOTE: Adjust if needed, test many curves in a detailed arch to see if we can handle such structures.
 * UPDATE: Increased from 1e-6 to 1e-4. There was an issue when sloped surfaces were on the -x / -y coords
 *          in trenchbroom, keep an eye on this when testing larger scale levels.
*/
const double epsilon = 1e-4f; 

// ------------------ Helper Functions ------------------ //

// Coordinate system conversion from TrenchBroom (Z-up) to Raylib (Y-up):
// TrenchBroom: X=right, Y=forward, Z=up
// Raylib:      X=right, Y=up, Z=forward
static void ConvertAxisZY(Vector3 &v) {
    float x = v.x;
    float y = v.y;
    float z = v.z;

    v.y = z;
    v.z = y;
}

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
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
    Vector3 edge1 = Vector3Subtract(v2, v1);
    Vector3 edge2 = Vector3Subtract(v3, v1);
    Vector3 normal = Vector3CrossProduct(edge1, edge2);
    return Vector3Normalize(normal);
}

// Remove duplicate points from a vector of Vector3
static void RemoveDuplicatePoints(std::vector<Vector3>& points, float eps) {
    std::vector<Vector3> unique;
    for (auto& p : points) {
        bool found = false;
        for (auto& u : unique) {
            float dx = p.x - u.x;
            float dy = p.y - u.y;
            float dz = p.z - u.z;
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < eps*eps) {
                found = true;
                break;
            }
        }
        if (!found) unique.push_back(p);
    }
    points = std::move(unique);
}

// Sort polygon vertices by angle around the centroid
static void SortPolygonVertices(std::vector<Vector3>& poly, Vector3 normal) {
    Vector3 centroid = {0,0,0};
    for (auto& p : poly) centroid = Vector3Add(centroid, p);
    centroid = Vector3Scale(centroid, 1.0f / (float)poly.size());

    // Create a local coordinate system in the plane of the polygon
    // Choose an arbitrary vector that is not parallel to normal
    Vector3 arbitrary = {1,0,0};
    if (fabs(Vector3DotProduct(normal, arbitrary)) > 0.9f) {
        arbitrary = (Vector3){0,1,0};
    }

    Vector3 U = Vector3Normalize(Vector3CrossProduct(normal, arbitrary));
    Vector3 V = Vector3CrossProduct(normal, U);

    struct AnglePoint {
        float angle;
        Vector3 p;
    };

    std::vector<AnglePoint> ap;
    ap.reserve(poly.size());
    for (auto& p : poly) {
        Vector3 dir = Vector3Subtract(p, centroid);
        float u = Vector3DotProduct(dir, U);
        float v = Vector3DotProduct(dir, V);
        float angle = atan2f(v, u);
        ap.push_back({angle, p});
    }

    std::sort(ap.begin(), ap.end(), [](const AnglePoint& a, const AnglePoint& b){
        return a.angle < b.angle;
    });

    for (size_t i=0; i<poly.size(); i++) {
        poly[i] = ap[i].p;
    }
}

// ------------------ TextureManager Functions ------------------ //

void InitTextureManager(TextureManager& manager) {
    manager.textures.clear();
    printf("TextureManager initialized and cleared.\n");
}

// Load texture by name using the TextureManager
Texture2D LoadTextureByName(TextureManager& manager, const std::string& textureName) { 
    auto it = manager.textures.find(textureName);
    if (it != manager.textures.end()) {
        printf("Texture '%s' already loaded.\n", textureName.c_str());
        return it->second;
    } else {
        std::string filePath = "../../assets/textures/" + textureName + ".png"; // Assuming PNG format
        Texture2D tex = LoadTexture(filePath.c_str());
        if (tex.id != 0) {
            manager.textures[textureName] = tex;
            printf("Successfully loaded texture: %s\n", filePath.c_str());
        } else {
            printf("Failed to load texture: %s\n", filePath.c_str());
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

    Vector3 term1 = Vector3Scale(cross23, -p1.d);
    Vector3 cross31 = Vector3CrossProduct(p3.normal, p1.normal);
    Vector3 term2 = Vector3Scale(cross31, -p2.d);
    Vector3 cross12 = Vector3CrossProduct(p1.normal, p2.normal);
    Vector3 term3 = Vector3Scale(cross12, -p3.d);

    Vector3 numerator = Vector3Add(Vector3Add(term1, term2), term3);
    intersection = Vector3Scale(numerator, 1.0f / denom);

    printf("Computed Intersection Point: [%f, %f, %f]\n", intersection.x, intersection.y, intersection.z);

    return true;
}

// ------------------ Parser Functions ------------------ //

//parse the .map file and return a Map structure
Map ParseMapFile(const std::string& filename) {
    Map map;
    std::ifstream file(filename);
    if (!file.is_open()) {
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
            continue;
        }

        if (line == "{") {
            if (!inEntity) {
                currentEntity = Entity();
                inEntity = true;
                printf("Started parsing a new entity.\n");
            } else if (!inBrush) {
                currentBrush = Brush();
                inBrush = true;
                brushCount++;
                printf("Started parsing a new brush.\n");
            }
            continue;
        }

        if (line == "}") {
            if (inBrush) {
                currentEntity.brushes.push_back(currentBrush);
                inBrush = false;
                printf("Finished parsing a brush. Total brushes: %d\n", brushCount);
            } else if (inEntity) {
                map.entities.push_back(currentEntity);
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
            if (line.find('(') != std::string::npos) {
                std::string faceLine = line;
                if (line.find(')') == std::string::npos) {
                    while (std::getline(file, line)) {
                        line = Trim(line);
                        faceLine += " " + line;
                        if (line.find(')') != std::string::npos) {
                            break;
                        }
                    }
                }

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
                                // Transform the vertex coordinate system
                                ConvertAxisZY(vertex);
                                face.vertices.push_back(vertex);
                            } catch (const std::invalid_argument&) {
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
                                // Transform texture axis as well
                                ConvertAxisZY(axis);
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
                            } catch (const std::invalid_argument&) {
                                printf("Invalid texture axes.\n");
                                if (i == 0) {
                                    face.textureAxes1 = { 0.0f, 0.0f, 0.0f };
                                    face.offsetX = 0.0f;
                                } else {
                                    face.textureAxes2 = { 0.0f, 0.0f, 0.0f };
                                    face.offsetY = 0.0f;
                                }
                            }
                        } else {
                            printf("Unexpected number of values in texture axes.\n");
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
                    } catch (...) {
                        face.rotation = 0.0f;
                    }
                }

                iss >> token;
                if (!token.empty()) {
                    try {
                        face.scaleX = std::stof(token);
                    } catch (...) {
                        face.scaleX = 1.0f;
                    }
                }

                iss >> token;
                if (!token.empty()) {
                    try {
                        face.scaleY = std::stof(token);
                    } catch (...) {
                        face.scaleY = 1.0f;
                    }
                }

                /*
                if (face.scaleX < 0.0f) {
                    face.textureAxes1 = Vector3Negate(face.textureAxes1);
                    face.scaleX = -face.scaleX;
                    face.offsetX = -face.offsetX;
                    printf("\nNEGATIVE SCALE.X\n");
                }
                if (face.scaleY < 0.0f) {
                    face.textureAxes2 = Vector3Negate(face.textureAxes2);
                    face.scaleY = -face.scaleY;
                    face.offsetY = -face.offsetY;
                    printf("\nNEGATIVE SCALE.Y\n");
                } */

                printf("Parsed Face Rotation: %f, ScaleX: %f, ScaleY: %f\n", face.rotation, face.scaleX, face.scaleY);

                // calculate normal with already transformed vertices
                face.normal = CalculateNormal(face.vertices[0], face.vertices[1], face.vertices[2]);
                printf("Calculated Face Normal: [%f, %f, %f]\n", face.normal.x, face.normal.y, face.normal.z);

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

// NOTE: Implement entities.cpp and move this
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
                        // Transform player start position
                        ConvertAxisZY(position);
                        playerStarts.push_back(PlayerStart{ position });
                        printf("Player Start Position: (%f, %f, %f)\n", position.x, position.y, position.z);
                    } catch (...) {
                        printf("Invalid origin coordinates for player start.\n");
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


// Convert Map to Raylib Model with Multiple Meshes and Materials
Model MapToMesh(const Map& map, TextureManager& textureManager) {
    struct TextureMeshData {
        std::vector<Vector3> vertices;
        std::vector<Vector3> normals;
        std::vector<Vector2> texcoords;
        std::vector<unsigned short> indices;
    };
    
    std::unordered_map<std::string, TextureMeshData> textureMeshesMap;

    for (const auto& entity : map.entities) {
        for (const auto& brush : entity.brushes) {
            int numFaces = (int)brush.faces.size();
            if (numFaces < 3) {
                printf("Brush has fewer than 3 faces. Skipping.\n");
                continue;
            }

            std::vector<Plane> planes;
            planes.reserve(numFaces);
            for (const auto& face : brush.faces) {
                Plane plane;
                plane.normal = Vector3Negate(face.normal); // NOTE: Negate the face normals for the planes to face outwards.
                plane.d = Vector3DotProduct(face.normal, face.vertices[0]);
                planes.push_back(plane);
            }

            std::vector<std::vector<Vector3>> polys(numFaces);

            // Find intersection points
            for (int i = 0; i < numFaces - 2; ++i) {
                for (int j = i + 1; j < numFaces - 1; ++j) {
                    for (int k = j + 1; k < numFaces; ++k) {
                        Vector3 intersectionPoint;
                        if (GetIntersection(planes[i], planes[j], planes[k], intersectionPoint)) {
                            bool isInside = true;
                            for (int m = 0; m < numFaces; ++m) {
                                float distance = Vector3DotProduct(brush.faces[m].normal, intersectionPoint) - planes[m].d;
                                if (distance > epsilon) {
                                    isInside = false;
                                    break;
                                }
                            }
                            if (isInside) {
                                polys[i].push_back(intersectionPoint);
                                polys[j].push_back(intersectionPoint);
                                polys[k].push_back(intersectionPoint);
                            }
                        }
                    }
                }
            }

            // Process each polygon face
            for (int i = 0; i < numFaces; ++i) {
                RemoveDuplicatePoints(polys[i], (float)epsilon);

                if (polys[i].size() < 3) {
                    printf("Skipping polygon %d due to insufficient vertices (%zu).\n", i, polys[i].size());
                    continue;
                }

                SortPolygonVertices(polys[i], brush.faces[i].normal);

                Vector3 polyNormal = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);

                // If polygon normal is opposite to face normal, reverse
                if (Vector3DotProduct(polyNormal, brush.faces[i].normal) < 0.0f) {
                    std::reverse(polys[i].begin(), polys[i].end());
                    polyNormal = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
                }

                Vector3 v0 = polys[i][0];
                for (size_t t = 1; t + 1 < polys[i].size(); ++t) {
                    Vector3 v1 = polys[i][t];
                    Vector3 v2 = polys[i][t + 1];

                    std::string textureName = brush.faces[i].texture;
                    Texture2D texture = LoadTextureByName(textureManager, textureName);
                    if (texture.id == 0) {
                        auto defaultIt = textureManager.textures.find("default");
                        if (defaultIt != textureManager.textures.end()) {
                            texture = defaultIt->second;
                            textureName = "default";
                        } else {
                            printf("No valid texture found for face.\n");
                            continue;
                        }
                    }

                    if (textureMeshesMap.find(textureName) == textureMeshesMap.end()) {
                        textureMeshesMap[textureName] = TextureMeshData();
                    }

                    // All important texture handling based on face-line data happens here.
                   auto ComputeUV = [&](const Vector3& vertex) -> Vector2 {

                        float sx = Vector3DotProduct(vertex, brush.faces[i].textureAxes1);
                        float sy = Vector3DotProduct(vertex, brush.faces[i].textureAxes2);
                        
                        // TODO: Division is slow and expensive. Try implementing some sort of math wizardry in place of division.
                        sx /= brush.faces[i].scaleX;
                        sy /= brush.faces[i].scaleY;

                        // Align textures for rotated faces
                        float rad = brush.faces[i].rotation * DEG2RAD;
                        float cosr = cosf(rad);
                        float sinr = sinf(rad);

                        float sxRot = sx * cosr - sy * sinr;
                        float syRot = sx * sinr + sy * cosr;

                        sxRot += brush.faces[i].offsetX;
                        syRot += brush.faces[i].offsetY;

                        // NOTE: (IF TILING) Normalize to [0..1] for repeated tiling? || Use Raylibs built-in texture repeat? 
                        sxRot /= (float)texture.width;
                        syRot /= (float)texture.height;

                        // Flip X-axis for coordinate system conversion.
                        sxRot = 1.0f - sxRot; 

                        return Vector2{ sxRot, syRot };
                    };

                    size_t baseVertexIndex = textureMeshesMap[textureName].vertices.size();
                    textureMeshesMap[textureName].vertices.push_back(v0);
                    textureMeshesMap[textureName].normals.push_back(polyNormal);
                    textureMeshesMap[textureName].texcoords.push_back(ComputeUV(v0));

                    textureMeshesMap[textureName].vertices.push_back(v1);
                    textureMeshesMap[textureName].normals.push_back(polyNormal);
                    textureMeshesMap[textureName].texcoords.push_back(ComputeUV(v1));

                    textureMeshesMap[textureName].vertices.push_back(v2);
                    textureMeshesMap[textureName].normals.push_back(polyNormal);
                    textureMeshesMap[textureName].texcoords.push_back(ComputeUV(v2));

                    textureMeshesMap[textureName].indices.push_back((unsigned short)baseVertexIndex);
                    textureMeshesMap[textureName].indices.push_back((unsigned short)(baseVertexIndex + 1));
                    textureMeshesMap[textureName].indices.push_back((unsigned short)(baseVertexIndex + 2));
                }
            }
        }
    }

    std::vector<Mesh> meshes;
    std::vector<Texture2D> textures;

    for (auto& pair : textureMeshesMap) {
        const std::string& textureName = pair.first;
        const TextureMeshData& meshData = pair.second;

        if (meshData.indices.empty()) {
            continue;
        }

        Mesh mesh = {0};
        mesh.vertexCount = (int)meshData.vertices.size();
        mesh.triangleCount = (int)(meshData.indices.size() / 3);
        mesh.vertices = (float*)malloc(sizeof(float)*mesh.vertexCount*3);
        mesh.normals = (float*)malloc(sizeof(float)*mesh.vertexCount*3);
        mesh.texcoords = (float*)malloc(sizeof(float)*mesh.vertexCount*2);
        mesh.indices = (unsigned short*)malloc(sizeof(unsigned short)*meshData.indices.size());

        for (int i = 0; i < mesh.vertexCount; ++i) {
            mesh.vertices[i*3+0] = meshData.vertices[i].x;
            mesh.vertices[i*3+1] = meshData.vertices[i].y;
            mesh.vertices[i*3+2] = meshData.vertices[i].z;

            mesh.normals[i*3+0] = meshData.normals[i].x;
            mesh.normals[i*3+1] = meshData.normals[i].y;
            mesh.normals[i*3+2] = meshData.normals[i].z;

            mesh.texcoords[i*2+0] = meshData.texcoords[i].x;
            mesh.texcoords[i*2+1] = meshData.texcoords[i].y;
        }

        for (size_t i=0; i < meshData.indices.size(); ++i) {
            mesh.indices[i] = meshData.indices[i];
        }

        UploadMesh(&mesh, false);
        meshes.push_back(mesh);
        textures.push_back(textureManager.textures.at(textureName));
    }

    Model model = {0};
    model.transform = MatrixIdentity();
    model.meshCount = (int)meshes.size();
    model.materialCount = (int)textures.size();

    if (model.meshCount > 0 && model.materialCount > 0) {
        model.meshes = (Mesh*)malloc(sizeof(Mesh)*model.meshCount);
        model.materials = (Material*)malloc(sizeof(Material)*model.materialCount);
        model.meshMaterial = (int*)malloc(sizeof(int)*model.meshCount);

        for (int i = 0; i < model.meshCount; ++i) {
            model.meshes[i] = meshes[i];
            model.meshMaterial[i] = i;
        }

        for (int i = 0; i < model.materialCount; ++i) {
            model.materials[i] = LoadMaterialDefault();
            model.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture = textures[i];
        }

        printf("Model created with %d meshes and %d materials.\n", model.meshCount, model.materialCount);
    } else {
        printf("No meshes or textures available to create the model.\n");
    }

    return model; 
}
