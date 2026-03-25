// map_parser.cpp
#include "map_parser.h"
#include "parameters.h"
#ifndef WARPED_COMPILER_BUILD
#include "../render/renderer.h"       // TextureManager / LoadTextureByName
#endif

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

static bool ParseVec3Prop(const Entity& e, const char* key, Vector3& out) {
    auto it = e.properties.find(key);
    if (it == e.properties.end()) return false;
    auto t = SplitBySpace(it->second);
    if (t.size() != 3) return false;
    try { out = { std::stof(t[0]), std::stof(t[1]), std::stof(t[2]) }; }
    catch (...) { return false; }
    return true;
}

static bool ParseFloatProp(const Entity& e, const char* key, float& out) {
    auto it = e.properties.find(key);
    if (it == e.properties.end()) return false;
    try { out = std::stof(it->second); }
    catch (...) {
        printf("  [warn] entity '%s' has non-numeric %s='%s', using default\n",
               e.properties.count("classname") ? e.properties.at("classname").c_str() : "?",
               key, it->second.c_str());
        return false;
    }
    return true;
}

std::vector<PointLight> GetPointLights(const Map &map) {
    std::vector<PointLight> lights;
    for (auto &e : map.entities) {
        auto it = e.properties.find("classname");
        if (it == e.properties.end() || it->second != "light_point") continue;

        Vector3 originTB{0,0,0}, color255{255,255,255};
        ParseVec3Prop(e, "origin", originTB);
        ParseVec3Prop(e, "_color", color255);

        float intensity = 300.0f;
        ParseFloatProp(e, "intensity", intensity);

        PointLight pl;
        pl.position  = ConvertTBtoRaylibEntities(originTB);
        pl.color     = { color255.x/255.0f, color255.y/255.0f, color255.z/255.0f };
        pl.intensity = intensity;
        lights.push_back(pl);
        printf("PointLight at (%.1f,%.1f,%.1f) radius=%.1f\n",
               pl.position.x, pl.position.y, pl.position.z, pl.intensity);
    }
    return lights;
}

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
    UV PROJECTION (shared by legacy path and .bsp compiler)
------------------------------------------------------------
*/
Vector2 ComputeFaceUV(const Vector3& vert,
                      const Vector3& axisU, const Vector3& axisV,
                      float offU, float offV, float rot,
                      float scaleU, float scaleV,
                      float texW, float texH)
{
    float sx = Vector3DotProduct(vert, axisU) / scaleU;
    float sy = Vector3DotProduct(vert, axisV) / scaleV;

    float rad = rot * DEG2RAD;
    float cr = cosf(rad), sr = sinf(rad);
    float sxR = sx*cr - sy*sr;
    float syR = sx*sr + sy*cr;

    // Negated for axis conversion.
    sxR += -offU;
    syR += -offV;

    if (texW > 0.f) sxR /= texW;
    if (texH > 0.f) syR /= texH;

    // Orientation flip for axis conversion.
    return (Vector2){ 1.0f - sxR, 1.0f - syR };
}

/*
------------------------------------------------------------
    BUILD POLYGONS  — renderer-agnostic (no TextureManager)
    Does plane intersection → CCW-sorted world-space polys.
    Used by the .bsp compiler and by BuildMapGeometry below.
------------------------------------------------------------
*/
std::vector<MapPolygon> BuildMapPolygons(const Map &map, bool devMode)
{
    std::vector<MapPolygon> out;

    for (auto &entity : map.entities) {
        bool isTrigger = false;
        auto cit = entity.properties.find("classname");
        if (cit != entity.properties.end()) {
            const std::string &cn = cit->second;
            if (cn == "trigger_once" || cn == "trigger_multiple") isTrigger = true;
        }

        for (auto &brush : entity.brushes) {
            if (isTrigger && !devMode) continue;

            int nF = (int)brush.faces.size();
            if (nF < 3) continue;

            std::vector<Plane> planes(nF);
            for (int i=0;i<nF;++i) {
                planes[i].normal = brush.faces[i].normal;
                planes[i].d      = Vector3DotProduct(brush.faces[i].normal,
                                                     brush.faces[i].vertices[0]);
            }

            std::vector<std::vector<Vector3>> polys(nF);
            for (int i=0;i<nF-2;++i)
              for (int j=i+1;j<nF-1;++j)
                for (int k=j+1;k<nF;++k) {
                    Vector3 ip;
                    if (!GetIntersection(planes[i],planes[j],planes[k],ip)) continue;
                    bool inside = true;
                    for (int m=0;m<nF;++m) {
                        float d = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                        if (d > (float)epsilon) { inside=false; break; }
                    }
                    if (inside) { polys[i].push_back(ip); polys[j].push_back(ip); polys[k].push_back(ip); }
                }

            for (int i=0;i<nF;++i) {
                RemoveDuplicatePoints(polys[i], (float)epsilon);
                if (polys[i].size()<3) continue;

                Vector3 nTB = brush.faces[i].normal;
                SortPolygonVertices(polys[i], nTB);
                for (auto &p : polys[i]) p = ConvertTBtoRaylib(p);

                Vector3 nRL = CalculateNormal(polys[i][0],polys[i][1],polys[i][2]);
                if (Vector3DotProduct(nRL, ConvertTBtoRaylib(nTB)) < 0.f) {
                    std::reverse(polys[i].begin(), polys[i].end());
                    nRL = CalculateNormal(polys[i][0],polys[i][1],polys[i][2]);
                }

                MapPolygon mp;
                mp.verts    = std::move(polys[i]);
                mp.normal   = nRL;
                mp.texture  = brush.faces[i].texture;
                mp.texAxisU = ConvertTBtoRaylib(brush.faces[i].textureAxes1);
                mp.texAxisV = ConvertTBtoRaylib(brush.faces[i].textureAxes2);
                mp.offU     = brush.faces[i].offsetX;
                mp.offV     = brush.faces[i].offsetY;
                mp.rot      = brush.faces[i].rotation;
                mp.scaleU   = brush.faces[i].scaleX;
                mp.scaleV   = brush.faces[i].scaleY;
                out.push_back(std::move(mp));
            }
        }
    }
    printf("[MapParser] Built %zu polygons.\n", out.size());
    return out;
}

#ifndef WARPED_COMPILER_BUILD
/*
------------------------------------------------------------
    BUILD RENDER GEOMETRY  (legacy direct-from-.map path)
------------------------------------------------------------
*/
std::vector<MapMeshBucket> BuildMapGeometry(const Map &map, TextureManager &textureManager)
{
    std::vector<MapPolygon> polys = BuildMapPolygons(map, DEVMODE);

    std::unordered_map<std::string, size_t> bucketIdx;
    std::vector<MapMeshBucket> buckets;
    auto GetBucket = [&](const std::string& n) -> MapMeshBucket& {
        auto it = bucketIdx.find(n);
        if (it != bucketIdx.end()) return buckets[it->second];
        bucketIdx[n] = buckets.size();
        buckets.push_back(MapMeshBucket{});
        buckets.back().texture = n;
        return buckets.back();
    };

    for (auto &p : polys) {
        std::string texName = p.texture;
        const TextureEntry* tex = LoadTextureByName(textureManager, texName);
        if (!tex || tex->width == 0) {
            tex = LoadTextureByName(textureManager, "default");
            texName = "default";
        }
        float tW=(float)tex->width, tH=(float)tex->height;

        MapMeshBucket& b = GetBucket(texName);
        Vector3 v0 = p.verts[0];
        Vector2 uv0 = ComputeFaceUV(v0,p.texAxisU,p.texAxisV,p.offU,p.offV,p.rot,p.scaleU,p.scaleV,tW,tH);

        for (size_t t=1; t+1<p.verts.size(); ++t) {
            Vector3 v1=p.verts[t], v2=p.verts[t+1];
            Vector2 uv1 = ComputeFaceUV(v1,p.texAxisU,p.texAxisV,p.offU,p.offV,p.rot,p.scaleU,p.scaleV,tW,tH);
            Vector2 uv2 = ComputeFaceUV(v2,p.texAxisU,p.texAxisV,p.offU,p.offV,p.rot,p.scaleU,p.scaleV,tW,tH);
            uint32_t base=(uint32_t)b.vertices.size();
            b.vertices.push_back({v0.x,v0.y,v0.z, p.normal.x,p.normal.y,p.normal.z, uv0.x,uv0.y, 0,0});
            b.vertices.push_back({v1.x,v1.y,v1.z, p.normal.x,p.normal.y,p.normal.z, uv1.x,uv1.y, 0,0});
            b.vertices.push_back({v2.x,v2.y,v2.z, p.normal.x,p.normal.y,p.normal.z, uv2.x,uv2.y, 0,0});
            b.indices.push_back(base); b.indices.push_back(base+1); b.indices.push_back(base+2);
        }
    }
    printf("[MapParser] Built %zu texture buckets.\n", buckets.size());
    return buckets;
}
#endif // !WARPED_COMPILER_BUILD
