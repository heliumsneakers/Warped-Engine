// map_parser.cpp
#include "map_parser.h"
#include "parameters.h"
#include "../render/renderer.h"       // TextureManager / LoadTextureByName

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

// meters per TB unit conversion
static constexpr float TB_TO_WORLD = 1.0f / 32.0f;

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

    return true;
}

// Convert coordinates for brush defined entities, this conversion ONLY works for brushes defined by the plane intersection logic.
Vector3 ConvertTBtoRaylib(const Vector3& in) {
    //  TB: (x,  y,  z) = (right, forward, up)
    //  GL: (x,  y,  z) = (right, up, forward)
    Vector3 out = {
        -in.x,
        -in.z,
         in.y
    };
    return out;
}

// Convert coordinates for entities not defined by brushes or the plane intersection logic.
static Vector3 ConvertTBtoRaylibEntities(const Vector3& in) {
    Vector3 out = {
         in.x,
         in.z,
        -in.y
    };
    return out;
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

std::vector<PlayerStart> GetPlayerStarts(const Map &map) {
    std::vector<PlayerStart> starts;
    for (auto &entity : map.entities) {
        auto it = entity.properties.find("classname");
        if (it != entity.properties.end() && it->second == "info_player_start") {
            auto orgIt = entity.properties.find("origin");
            if (orgIt != entity.properties.end()) {
                auto coords = SplitBySpace(orgIt->second);
                if (coords.size() == 3) {
                    try {
                        Vector3 posTB = {
                            std::stof(coords[0]),
                            std::stof(coords[1]),
                            std::stof(coords[2])
                        };
                        Vector3 posRL = ConvertTBtoRaylibEntities(posTB);

                        PlayerStart ps;
                        ps.position = posRL;
                        starts.push_back(ps);

                        printf("PlayerStart TB:(%.1f, %.1f, %.1f)  RL:(%.1f, %.1f, %.1f)\n",
                               posTB.x, posTB.y, posTB.z, posRL.x, posRL.y, posRL.z);
                    } catch (...) {}
                }
            }
        }
    }
    return starts;
}

/*
------------------------------------------------------------
    BUILD RENDER GEOMETRY  (CPU side — GPU upload is in renderer.cpp)
------------------------------------------------------------
*/

std::vector<MapMeshBucket> BuildMapGeometry(const Map &map, TextureManager &textureManager)
{
    // texture-name → bucket-index
    std::unordered_map<std::string, size_t> bucketIdx;
    std::vector<MapMeshBucket> buckets;

    auto GetBucket = [&](const std::string& name) -> MapMeshBucket& {
        auto it = bucketIdx.find(name);
        if (it != bucketIdx.end()) return buckets[it->second];
        bucketIdx[name] = buckets.size();
        buckets.push_back(MapMeshBucket{});
        buckets.back().texture = name;
        return buckets.back();
    };

    for (auto &entity : map.entities) {
        // Logic for skipping trigger entity rendering if we are not in DEVMODE
        bool isTriggerBrush = false;
        auto it = entity.properties.find("classname");
        if (it != entity.properties.end()) {
            const std::string &classname = it->second;
            if (classname == "trigger_once" || classname == "trigger_multiple") {
                isTriggerBrush = true;
            }
        }

        for (auto &brush : entity.brushes) {

            // Skip trigger brushes rendering if !DEVMODE
            if (isTriggerBrush && !DEVMODE) {
                continue;
            }

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

                // Texture lookup (needed for width/height in UV divide)
                std::string texName = brush.faces[i].texture;
                const TextureEntry* tex = LoadTextureByName(textureManager, texName);
                if (!tex || tex->width == 0) {
                    tex = LoadTextureByName(textureManager, "default");
                    texName = "default";
                }

                float texW = (float)tex->width;
                float texH = (float)tex->height;

                Vector3 tAx1 = ConvertTBtoRaylib(brush.faces[i].textureAxes1);
                Vector3 tAx2 = ConvertTBtoRaylib(brush.faces[i].textureAxes2);

                float offsetX = brush.faces[i].offsetX;
                float offsetY = brush.faces[i].offsetY;
                float rotDeg  = brush.faces[i].rotation;
                float scaleX  = brush.faces[i].scaleX;
                float scaleY  = brush.faces[i].scaleY;

                auto ComputeUV = [&](const Vector3 &vertRL) -> Vector2 {
                    float sx = Vector3DotProduct(vertRL, tAx1);
                    float sy = Vector3DotProduct(vertRL, tAx2);

                    sx /= scaleX;
                    sy /= scaleY;

                    float rad  = rotDeg * DEG2RAD;
                    float cosr = cosf(rad);
                    float sinr = sinf(rad);

                    float sxRot = sx * cosr - sy * sinr;
                    float syRot = sx * sinr + sy * cosr;

                    // NOTE: Negated for axis conversion.
                    sxRot += -(offsetX);
                    syRot += -(offsetY);

                    sxRot /= texW;
                    syRot /= texH;

                    // Orientation flip for axis conversion.
                    syRot = 1.0f - syRot;
                    sxRot = 1.0f - sxRot;

                    return (Vector2){sxRot, syRot};
                };

                MapMeshBucket& bucket = GetBucket(texName);

                Vector3 v0 = polys[i][0];
                for (size_t t=1; t+1<polys[i].size(); ++t) {
                    Vector3 v1 = polys[i][t];
                    Vector3 v2 = polys[i][t+1];

                    uint32_t base = (uint32_t)bucket.vertices.size();

                    Vector2 uv0 = ComputeUV(v0);
                    Vector2 uv1 = ComputeUV(v1);
                    Vector2 uv2 = ComputeUV(v2);

                    bucket.vertices.push_back({v0.x,v0.y,v0.z, polyNormal.x,polyNormal.y,polyNormal.z, uv0.x,uv0.y});
                    bucket.vertices.push_back({v1.x,v1.y,v1.z, polyNormal.x,polyNormal.y,polyNormal.z, uv1.x,uv1.y});
                    bucket.vertices.push_back({v2.x,v2.y,v2.z, polyNormal.x,polyNormal.y,polyNormal.z, uv2.x,uv2.y});

                    bucket.indices.push_back(base+0);
                    bucket.indices.push_back(base+1);
                    bucket.indices.push_back(base+2);
                }
            } // each face
        } // each brush
    } // each entity

    printf("[MapParser] Built %zu texture buckets.\n", buckets.size());
    return buckets;
}
