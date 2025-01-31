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
------------------------------------------------------------
 HELPER FUNCTIONS
------------------------------------------------------------
*/

// Trim leading/trailing whitespace
static std::string Trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end   = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

// Split a string by whitespace
static std::vector<std::string> SplitBySpace(const std::string& s) {
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string token;
    while (iss >> token) tokens.push_back(token);
    return tokens;
}

// Calculate normal vector from 3 points
Vector3 CalculateNormal(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
    Vector3 edge1 = Vector3Subtract(v2, v1);
    Vector3 edge2 = Vector3Subtract(v3, v1);
    Vector3 n     = Vector3CrossProduct(edge1, edge2);
    return Vector3Normalize(n);
}

// Remove duplicates in-place
void RemoveDuplicatePoints(std::vector<Vector3>& points, float eps) {
    std::vector<Vector3> unique;
    unique.reserve(points.size());

    for (auto &p : points) {
        bool found = false;
        for (auto &u : unique) {
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

// Sort polygon vertices by angle around centroid
void SortPolygonVertices(std::vector<Vector3>& poly, const Vector3& normal) {
    if (poly.size() < 3) return;

    // Compute centroid
    Vector3 centroid = {0,0,0};
    for (auto &p : poly) centroid = Vector3Add(centroid, p);
    centroid = Vector3Scale(centroid, 1.0f / (float)poly.size());

    // Create local axes in plane
    Vector3 arbitrary = {1,0,0};
    if (fabsf(Vector3DotProduct(normal, arbitrary)) > 0.9f) {
        arbitrary = (Vector3){0,1,0};
    }

    Vector3 U = Vector3Normalize(Vector3CrossProduct(normal, arbitrary));
    Vector3 V = Vector3CrossProduct(normal, U);

    struct AnglePoint {
        float   angle;
        Vector3 point;
    };

    std::vector<AnglePoint> ap;
    ap.reserve(poly.size());
    for (auto &p : poly) {
        Vector3 dir = Vector3Subtract(p, centroid);
        float u     = Vector3DotProduct(dir, U);
        float v     = Vector3DotProduct(dir, V);
        float angle = atan2f(v, u);
        ap.push_back({angle, p});
    }

    std::sort(ap.begin(), ap.end(), [](const AnglePoint &a, const AnglePoint &b){
        return a.angle < b.angle;
    });

    for (size_t i=0; i<ap.size(); i++) {
        poly[i] = ap[i].point;
    }
}

// Simple intersection of three planes (in TB coords)
// **Credit to Stefan Hajnoczi**
bool GetIntersection(const Plane& p1, const Plane& p2, const Plane& p3, Vector3& out) {
    Vector3 cross23 = Vector3CrossProduct(p2.normal, p3.normal);
    float denom     = Vector3DotProduct(p1.normal, cross23);

    if (fabsf(denom) < (float)epsilon) {
        return false;
    }

    Vector3 term1 = Vector3Scale(cross23, -p1.d);
    Vector3 cross31 = Vector3CrossProduct(p3.normal, p1.normal);
    Vector3 term2 = Vector3Scale(cross31, -p2.d);
    Vector3 cross12 = Vector3CrossProduct(p1.normal, p2.normal);
    Vector3 term3 = Vector3Scale(cross12, -p3.d);

    Vector3 numerator = Vector3Add(Vector3Add(term1, term2), term3);
    out = Vector3Scale(numerator, 1.0f / denom);

    // Debug
    //printf("Intersection: [%.2f, %.2f, %.2f]\n", out.x, out.y, out.z);
    
    return true;
}

Vector3 ConvertTBtoRaylib(const Vector3& in) {
    //  TB: (x,  y,  z) = (right, forward, up)
    // Raylib: (x,  y,  z) = (right, up, forward)
    Vector3 out = { 
        -in.x,     // X
        -in.z,    // Y
        in.y     // Z
    };
    return out;
}

/*
  --------------------------------------------------------
    TEXTURE MANAGER
  --------------------------------------------------------
*/

void InitTextureManager(TextureManager& manager) {
    manager.textures.clear();
    printf("TextureManager initialized and cleared.\n");
}

Texture2D LoadTextureByName(TextureManager& manager, const std::string& textureName) { 
    auto it = manager.textures.find(textureName);
    if (it != manager.textures.end()) {
        // LOGGING
        // printf("Texture '%s' already loaded.\n", textureName.c_str());
        return it->second;
    } else {
        std::string filePath = "../../assets/textures/" + textureName + ".png"; // Assuming PNG format
        Texture2D tex = LoadTexture(filePath.c_str());
        if (tex.id != 0) {
            manager.textures[textureName] = tex;
            // LOGGING
            // printf("Successfully loaded texture: %s\n", filePath.c_str());
        } else {
            printf("Failed to load texture: %s\n", filePath.c_str());
            std::string defaultPath = "../../assets/textures/default.png";
            tex = LoadTexture(defaultPath.c_str());
            if (tex.id != 0) {
                manager.textures["default"] = tex;
                // LOGGING
                // printf("Loaded default texture: %s\n", defaultPath.c_str());
            } else {
                printf("Failed to load default texture: %s\n", defaultPath.c_str());
            }
        }
        return tex;
    }
}

void UnloadAllTextures(TextureManager& manager) {
    for (auto& pair : manager.textures) {
        UnloadTexture(pair.second);
        printf("Unloaded texture: %s\n", pair.first.c_str());
    }
    manager.textures.clear();
    printf("All textures unloaded and TextureManager cleared.\n");
}


/*
------------------------------------------------------------
 PARSER IMPLEMENTATION
------------------------------------------------------------
*/

// The main parse function: parse geometry in TB coords
Map ParseMapFile(const std::string &filename) {
    Map map;
    std::ifstream file(filename);
    if (!file.is_open()) {
        printf("Failed to open map: %s\n", filename.c_str());
        return map;
    }

    std::string line;
    Entity currentEntity;
    Brush currentBrush;
    bool inEntity = false;
    bool inBrush  = false;
    int entityCount=0, brushCount=0, faceCount=0;

    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0]=='/') {
            continue;
        }
        if (line == "{") {
            if (!inEntity) {
                currentEntity = Entity();
                inEntity = true;
                printf("Started new entity.\n");
            }
            else if (!inBrush) {
                currentBrush = Brush();
                inBrush = true;
                brushCount++;
                printf("Started new brush.\n");
            }
            continue;
        }
        if (line == "}") {
            if (inBrush) {
                currentEntity.brushes.push_back(currentBrush);
                inBrush = false;
                printf("Finished a brush. brushCount=%d\n", brushCount);
            }
            else if (inEntity) {
                map.entities.push_back(currentEntity);
                inEntity = false;
                entityCount++;
                printf("Finished an entity. entityCount=%d\n", entityCount);
            }
            continue;
        }

        // If parsing entity properties
        if (inEntity && !inBrush) {
            // ex: "classname" "worldspawn"
            size_t firstQ = line.find('"');
            if (firstQ != std::string::npos) {
                size_t secondQ = line.find('"', firstQ+1);
                if (secondQ != std::string::npos) {
                    std::string key = line.substr(firstQ+1, secondQ - firstQ-1);
                    size_t thirdQ = line.find('"', secondQ+1);
                    if (thirdQ != std::string::npos) {
                        size_t fourthQ = line.find('"', thirdQ+1);
                        if (fourthQ != std::string::npos) {
                            std::string val = line.substr(thirdQ+1, fourthQ - thirdQ -1);
                            currentEntity.properties[key] = val;
                            // Debug
                            printf("Property: %s = %s\n", key.c_str(), val.c_str());
                        }
                    }
                }
            }
            continue;
        }

        // If parsing brush faces
        if (inBrush) {
            if (line.find('(') != std::string::npos) {
                // accumulate
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

                // parse 3 vertices
                for (int i=0; i<3; ++i) {
                    iss >> token;
                    if (token.empty()) continue;
                    if (token.front()=='(') {
                        size_t cParen = token.find(')');
                        std::string vStr = (cParen!=std::string::npos)
                            ? token.substr(1, cParen-1)
                            : token.substr(1);

                        while (cParen==std::string::npos && iss >> token) {
                            size_t maybe = token.find(')');
                            if (maybe != std::string::npos) {
                                vStr += " " + token.substr(0, maybe);
                                break;
                            }
                            vStr += " " + token;
                        }
                        auto coords = SplitBySpace(vStr);
                        if (coords.size()==3) {
                            try {
                                Vector3 v = {
                                    std::stof(coords[0]),
                                    std::stof(coords[1]),
                                    std::stof(coords[2])
                                };
                                face.vertices.push_back(v);
                            } catch (...) {
                                face.vertices.clear();
                                break;
                            }
                        }
                    }
                }

                if (face.vertices.size()<3) {
                    printf("Skipped a face due to <3 vertices.\n");
                    continue;
                }

                // parse texture name
                iss >> token;
                face.texture = token;

                // parse texture axes
                for (int i=0; i<2; ++i) {
                    iss >> token;
                    if (token.empty()) continue;
                    if (token.front()=='[') {
                        std::string axesStr = token.substr(1);
                        if (token.find(']') == std::string::npos) {
                            while (iss >> token) {
                                axesStr += " " + token;
                                if (token.find(']')!=std::string::npos) {
                                    axesStr = axesStr.substr(0, axesStr.find(']'));
                                    break;
                                }
                            }
                        }
                        auto axisToks = SplitBySpace(axesStr);
                        if (axisToks.size()==4) {
                            try {
                                Vector3 axis = {
                                    std::stof(axisToks[0]),
                                    std::stof(axisToks[1]),
                                    std::stof(axisToks[2])
                                };
                                float offset = std::stof(axisToks[3]);
                                if (i==0) {
                                    face.textureAxes1 = axis;
                                    face.offsetX = offset;
                                } else {
                                    face.textureAxes2 = axis;
                                    face.offsetY = offset;
                                }
                            } catch(...) {}
                        }
                    }
                }

                iss >> token; 
                face.rotation = (!token.empty()) ? std::stof(token) : 0.f;
                iss >> token;
                face.scaleX   = (!token.empty()) ? std::stof(token) : 1.f;
                iss >> token;
                face.scaleY   = (!token.empty()) ? std::stof(token) : 1.f;
 
                face.normal = CalculateNormal(face.vertices[0],
                                              face.vertices[1],
                                              face.vertices[2]);
                currentBrush.faces.push_back(face);
                faceCount++;
            }
        }

    }

    file.close();
    printf("Parsed map: Entities=%d, Brushes=%d, Faces=%d\n", entityCount, brushCount, faceCount);
    return map;
}

/*
 --------------------------------------------------------
    ENTITY PARSING
 --------------------------------------------------------
*/
// For reading "info_player_start" in TB coords
std::vector<PlayerStart> GetPlayerStarts(const Map &map) {
    std::vector<PlayerStart> starts;
    for (auto &entity : map.entities) {
        auto it = entity.properties.find("classname");
        if (it!=entity.properties.end() && it->second=="info_player_start") {
            auto orgIt = entity.properties.find("origin");
            if (orgIt!=entity.properties.end()) {
                auto coords = SplitBySpace(orgIt->second);
                if (coords.size()==3) {
                    try {
                        Vector3 pos = {
                            std::stof(coords[0]),
                            std::stof(coords[2]),
                            std::stof(coords[1])
                        }; 
                        PlayerStart ps;
                        ps.position = pos; 
                        starts.push_back(ps);

                        printf("PlayerStart: TB(%.1f, %.1f, %.1f)\n",
                               pos.x, pos.y, pos.z);
                    } catch(...) {}
                }
            }
        }
    }
    return starts;
}

/*
------------------------------------------------------------
    BUILD MODEL
------------------------------------------------------------
*/

Model MapToMesh(const Map &map, TextureManager &textureManager) {
    // Store final triangles in the Raylib coordinate system.

    struct TextureMeshData {
        std::vector<Vector3> vertices; 
        std::vector<Vector3> normals;  
        std::vector<Vector2> texcoords;
        std::vector<unsigned short> indices;
    };
    std::unordered_map<std::string, TextureMeshData> textureMeshes;

    for (auto &entity : map.entities) {
        for (auto &brush : entity.brushes) {
            int numFaces = (int)brush.faces.size();
            if (numFaces < 3) {
                printf("Skipping brush (fewer than 3 faces).\n");
                continue;
            }

            // Create planes in TB coords
            std::vector<Plane> planes;
            planes.reserve(numFaces);
            for (auto &face : brush.faces) {
                Plane plane;
                plane.normal = face.normal; 
                plane.d      = Vector3DotProduct(face.normal, face.vertices[0]);
                planes.push_back(plane);
            }

            // Build polygons per face
            std::vector<std::vector<Vector3>> polys(numFaces);

            for (int i=0; i<numFaces-2; ++i) {
                for (int j=i+1; j<numFaces-1; ++j) {
                    for (int k=j+1; k<numFaces; ++k) {
                        Vector3 ip;
                        if (GetIntersection(planes[i], planes[j], planes[k], ip)) {
                            bool inside = true;
                            for (int m=0; m<numFaces; ++m) {
                                float dist = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                                if (dist > (float)epsilon) {
                                    inside = false;
                                    break;
                                }
                            }
                            if (inside) {
                                polys[i].push_back(ip);
                                polys[j].push_back(ip);
                                polys[k].push_back(ip);
                            }
                        }
                    }
                }
            }

            
            for (int i=0; i<numFaces; ++i) {
                RemoveDuplicatePoints(polys[i], (float)epsilon);
                if (polys[i].size()<3) {
                    printf("Skipping polygon %d, insufficient vertices.\n", i);
                    continue;
                }

                Vector3 faceNormalTB = brush.faces[i].normal;

                
                SortPolygonVertices(polys[i], faceNormalTB);

                for (auto &pt : polys[i]) {
                    pt = ConvertTBtoRaylib(pt);
                }
                Vector3 polyNormal = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);

                if (Vector3DotProduct(polyNormal, ConvertTBtoRaylib(faceNormalTB)) < 0.f) {
                    std::reverse(polys[i].begin(), polys[i].end());
                    polyNormal = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
                }

                Vector3 v0 = polys[i][0];
                for (size_t t=1; t+1<polys[i].size(); ++t) {
                    Vector3 v1 = polys[i][t];
                    Vector3 v2 = polys[i][t+1];
 
                    std::string texName = brush.faces[i].texture;
                    Texture2D texture   = LoadTextureByName(textureManager, texName);
                    if (texture.id == 0) {
                        auto def = textureManager.textures.find("default");
                        if (def!=textureManager.textures.end()) {
                            texture = def->second;
                            texName = "default";
                        } else {
                            printf("No valid texture found for face.\n");
                            continue;
                        }
                    }

                    if (textureMeshes.find(texName)==textureMeshes.end()) {
                        textureMeshes[texName] = TextureMeshData();
                    }

                    // For UV, we also want to convert face.textureAxes1/2 
                    Vector3 tAx1 = ConvertTBtoRaylib(brush.faces[i].textureAxes1);
                    Vector3 tAx2 = ConvertTBtoRaylib(brush.faces[i].textureAxes2);

                    float offsetX = brush.faces[i].offsetX;
                    float offsetY = brush.faces[i].offsetY;
                    float rotDeg  = brush.faces[i].rotation;
                    float scaleX  = brush.faces[i].scaleX;
                    float scaleY  = brush.faces[i].scaleY;

                    auto ComputeUV = [&](const Vector3 &vertRL) -> Vector2 {
                        // Dot in Raylib space
                        float sx = Vector3DotProduct(vertRL, tAx1);
                        float sy = Vector3DotProduct(vertRL, tAx2);

                        // Scale
                        sx /= scaleX;
                        sy /= scaleY;

                        // Rotation
                        float rad = rotDeg * DEG2RAD;
                        float cosr   = cosf(rad);
                        float sinr   = sinf(rad);

                        float sxRot = sx * cosr - sy * sinr;
                        float syRot = sx * sinr + sy * cosr;

                        // Offset
                        // NOTE: Negated for axis conversion.
                        sxRot += -(offsetX);
                        syRot += -(offsetY);

                        // Normalize to [0..1] for repeated tiling?
                        sxRot /= (float)texture.width;
                        syRot /= (float)texture.height;

                        // Orientation flip for axis conversion.
                        syRot = 1.0f - syRot;
                        sxRot = 1.0f - sxRot;

                        return (Vector2){sxRot, syRot};
                    };

                    // Add to buffer
                    size_t baseIdx = textureMeshes[texName].vertices.size();
                    textureMeshes[texName].vertices.push_back(v0);
                    textureMeshes[texName].normals.push_back(polyNormal);
                    textureMeshes[texName].texcoords.push_back( ComputeUV(v0) );

                    textureMeshes[texName].vertices.push_back(v1);
                    textureMeshes[texName].normals.push_back(polyNormal);
                    textureMeshes[texName].texcoords.push_back( ComputeUV(v1) );

                    textureMeshes[texName].vertices.push_back(v2);
                    textureMeshes[texName].normals.push_back(polyNormal);
                    textureMeshes[texName].texcoords.push_back( ComputeUV(v2) );

                    textureMeshes[texName].indices.push_back((unsigned short)baseIdx);
                    textureMeshes[texName].indices.push_back((unsigned short)(baseIdx+1));
                    textureMeshes[texName].indices.push_back((unsigned short)(baseIdx+2));
                } // triangulate
            } // each face
        } // each brush
    } // each entity

    // Build Raylib Model
    std::vector<Mesh>     meshes;
    std::vector<Texture2D> textures;

    for (auto &kv : textureMeshes) {
        const std::string &texName = kv.first;
        const auto &md            = kv.second;
        if (md.indices.empty()) continue;

        Mesh mesh = {0};
        mesh.vertexCount   = (int)md.vertices.size();
        mesh.triangleCount = (int)(md.indices.size()/3);

        mesh.vertices  = (float*)malloc(sizeof(float)*mesh.vertexCount*3);
        mesh.normals   = (float*)malloc(sizeof(float)*mesh.vertexCount*3);
        mesh.texcoords = (float*)malloc(sizeof(float)*mesh.vertexCount*2);
        mesh.indices   = (unsigned short*)malloc(sizeof(unsigned short)*md.indices.size());

        for (int i=0; i<mesh.vertexCount; ++i) {
            mesh.vertices[i*3+0]  = md.vertices[i].x;
            mesh.vertices[i*3+1]  = md.vertices[i].y;
            mesh.vertices[i*3+2]  = md.vertices[i].z;

            mesh.normals[i*3+0]   = md.normals[i].x;
            mesh.normals[i*3+1]   = md.normals[i].y;
            mesh.normals[i*3+2]   = md.normals[i].z;

            mesh.texcoords[i*2+0] = md.texcoords[i].x;
            mesh.texcoords[i*2+1] = md.texcoords[i].y;
        }
        for (size_t i=0; i<md.indices.size(); ++i) {
            mesh.indices[i] = md.indices[i];
        }

        // Upload to GPU
        UploadMesh(&mesh, false);
        meshes.push_back(mesh);

        // Grab the texture
        auto tIt = textureManager.textures.find(texName);
        if (tIt != textureManager.textures.end()) {
            textures.push_back(tIt->second);
        } else {
            // fallback to "default" if missing
            auto def = textureManager.textures.find("default");
            if (def!=textureManager.textures.end()) {
                textures.push_back(def->second);
            } else {
                // push a blank texture if absolutely none
                Texture2D blank = {0};
                textures.push_back(blank);
            }
        }
    }

    Model model = {0};
    model.transform = MatrixIdentity();
    model.meshCount = (int)meshes.size();
    model.materialCount = (int)textures.size();

    if (model.meshCount > 0 && model.materialCount > 0) {
        model.meshes        = (Mesh*)malloc(sizeof(Mesh)*model.meshCount);
        model.materials     = (Material*)malloc(sizeof(Material)*model.materialCount);
        model.meshMaterial  = (int*)malloc(sizeof(int)*model.meshCount);

        for (int i=0; i<model.meshCount; ++i) {
            model.meshes[i] = meshes[i];
            model.meshMaterial[i] = i;
        }
        for (int i=0; i<model.materialCount; ++i) {
            model.materials[i] = LoadMaterialDefault();
            model.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture = textures[i];
        }
        printf("Model created with %d meshes, %d materials.\n", model.meshCount, model.materialCount);
    }
    else {
        printf("No valid geometry found.\n");
    }

    return model;
}

