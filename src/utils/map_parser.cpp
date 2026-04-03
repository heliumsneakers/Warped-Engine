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
#include <cstring>
#include <unordered_map>
#include <vector>
#include <string>
#include <iterator>
#include <cmath>
#include <algorithm>

// meters per TB unit conversion
static constexpr float TB_TO_WORLD = 1.0f / 32.0f;

static Vector3 ConvertTBtoRaylibEntities(const Vector3& in);

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

struct BrushSolidPlane {
    Vector3 point;
    Vector3 normal;
};

struct BrushSolid {
    int sourceBrushId = -1;
    std::vector<BrushSolidPlane> planes;
};

struct PolygonPlaneSplit {
    std::vector<Vector3> front;
    std::vector<Vector3> back;
    bool hasStrictFront = false;
};

static void CleanupClippedPolygon(std::vector<Vector3>& poly, const Vector3& expectedNormal) {
    RemoveDuplicatePoints(poly, (float)epsilon);
    if (poly.size() < 3) {
        poly.clear();
        return;
    }

    std::vector<Vector3> cleaned;
    cleaned.reserve(poly.size());
    const float collinearEpsSq = (float)(epsilon * epsilon);
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& prev = poly[(i + poly.size() - 1) % poly.size()];
        const Vector3& curr = poly[i];
        const Vector3& next = poly[(i + 1) % poly.size()];
        const Vector3 e0 = Vector3Subtract(curr, prev);
        const Vector3 e1 = Vector3Subtract(next, curr);
        if (Vector3LengthSq(e0) <= collinearEpsSq || Vector3LengthSq(e1) <= collinearEpsSq) {
            continue;
        }
        const Vector3 cross = Vector3CrossProduct(e0, e1);
        if (Vector3LengthSq(cross) <= collinearEpsSq) {
            continue;
        }
        cleaned.push_back(curr);
    }

    RemoveDuplicatePoints(cleaned, (float)epsilon);
    if (cleaned.size() < 3) {
        poly.clear();
        return;
    }

    Vector3 clippedNormal = CalculateNormal(cleaned[0], cleaned[1], cleaned[2]);
    if (Vector3DotProduct(clippedNormal, expectedNormal) < 0.0f) {
        std::reverse(cleaned.begin(), cleaned.end());
    }
    poly.swap(cleaned);
}

static PolygonPlaneSplit SplitPolygonByPlane(const std::vector<Vector3>& poly,
                                             const Vector3& planePoint,
                                             const Vector3& planeNormal) {
    PolygonPlaneSplit split;
    if (poly.size() < 3) {
        return split;
    }

    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& a = poly[i];
        const Vector3& b = poly[(i + 1) % poly.size()];
        const float da = Vector3DotProduct(planeNormal, Vector3Subtract(a, planePoint));
        const float db = Vector3DotProduct(planeNormal, Vector3Subtract(b, planePoint));
        const bool aFront = da > (float)epsilon;
        const bool aBack = da < -(float)epsilon;

        if (aFront) {
            split.hasStrictFront = true;
        }
        if (!aBack) {
            split.front.push_back(a);
        }
        if (!aFront) {
            split.back.push_back(a);
        }

        if ((da > (float)epsilon && db < -(float)epsilon) ||
            (da < -(float)epsilon && db > (float)epsilon)) {
            const float t = da / (da - db);
            const Vector3 hit = Vector3Add(a, Vector3Scale(Vector3Subtract(b, a), t));
            split.front.push_back(hit);
            split.back.push_back(hit);
            split.hasStrictFront = true;
        }
    }

    return split;
}

static std::vector<BrushSolid> BuildBrushSolids(const std::vector<MapPolygon>& polys) {
    std::vector<BrushSolid> solids;
    std::unordered_map<int, size_t> solidIndexByBrushId;
    for (const MapPolygon& poly : polys) {
        if (poly.sourceBrushId < 0 || poly.verts.empty()) {
            continue;
        }
        auto [it, inserted] = solidIndexByBrushId.emplace(poly.sourceBrushId, solids.size());
        if (inserted) {
            BrushSolid solid;
            solid.sourceBrushId = poly.sourceBrushId;
            solids.push_back(std::move(solid));
        }
        solids[it->second].planes.push_back({ poly.verts[0], Vector3Normalize(poly.normal) });
    }
    return solids;
}

static std::vector<MapPolygon> SubtractSolidFromPolygon(const MapPolygon& poly, const BrushSolid& solid) {
    std::vector<MapPolygon> pending;
    std::vector<MapPolygon> kept;
    pending.push_back(poly);

    for (const BrushSolidPlane& plane : solid.planes) {
        std::vector<MapPolygon> nextPending;
        for (const MapPolygon& fragment : pending) {
            PolygonPlaneSplit split = SplitPolygonByPlane(fragment.verts, plane.point, plane.normal);
            CleanupClippedPolygon(split.front, fragment.normal);
            CleanupClippedPolygon(split.back, fragment.normal);

            if (split.hasStrictFront && split.front.size() >= 3) {
                MapPolygon outsideFragment = fragment;
                outsideFragment.verts = std::move(split.front);
                kept.push_back(std::move(outsideFragment));
            }
            if (split.back.size() >= 3) {
                MapPolygon insideFragment = fragment;
                insideFragment.verts = std::move(split.back);
                nextPending.push_back(std::move(insideFragment));
            }
        }
        pending.swap(nextPending);
        if (pending.empty()) {
            break;
        }
    }

    return kept;
}

static std::vector<MapPolygon> BuildExteriorPolygons(const std::vector<MapPolygon>& polys) {
    if (polys.empty()) {
        return {};
    }

    const std::vector<BrushSolid> solids = BuildBrushSolids(polys);
    std::vector<MapPolygon> exterior;
    for (const MapPolygon& poly : polys) {
        std::vector<MapPolygon> fragments;
        fragments.push_back(poly);

        for (const BrushSolid& solid : solids) {
            if (solid.sourceBrushId == poly.sourceBrushId) {
                continue;
            }

            std::vector<MapPolygon> nextFragments;
            for (const MapPolygon& fragment : fragments) {
                std::vector<MapPolygon> kept = SubtractSolidFromPolygon(fragment, solid);
                nextFragments.insert(nextFragments.end(),
                                     std::make_move_iterator(kept.begin()),
                                     std::make_move_iterator(kept.end()));
            }
            fragments.swap(nextFragments);
            if (fragments.empty()) {
                break;
            }
        }

        exterior.insert(exterior.end(),
                        std::make_move_iterator(fragments.begin()),
                        std::make_move_iterator(fragments.end()));
    }
    return exterior;
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

                Face face{};
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

                if (iss >> token) {
                    face.rotation = !token.empty() ? std::stof(token) : 0.f;
                } else {
                    face.rotation = 0.f;
                }
                if (iss >> token) {
                    face.scaleX = !token.empty() ? std::stof(token) : 1.f;
                } else {
                    face.scaleX = 1.f;
                }
                if (iss >> token) {
                    face.scaleY = !token.empty() ? std::stof(token) : 1.f;
                } else {
                    face.scaleY = 1.f;
                }

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

static bool ParseIntProp(const Entity& e, const char* key, int& out) {
    float parsed = 0.0f;
    if (!ParseFloatProp(e, key, parsed)) {
        return false;
    }
    out = (int)std::lround(parsed);
    return true;
}

static bool EntityHasClass(const Entity& e, const char* classname) {
    auto it = e.properties.find("classname");
    return (it != e.properties.end()) && (it->second == classname);
}

static bool EntityClassStartsWith(const Entity& e, const char* prefix) {
    auto it = e.properties.find("classname");
    if (it == e.properties.end()) {
        return false;
    }
    const std::string& name = it->second;
    const size_t len = strlen(prefix);
    return name.size() >= len && name.compare(0, len, prefix) == 0;
}

static bool ShouldRenderBrushEntity(const Entity& entity, bool devMode) {
    if ((EntityHasClass(entity, "trigger_once") || EntityHasClass(entity, "trigger_multiple")) && !devMode) {
        return false;
    }

    if (EntityHasClass(entity, "func_clip")) {
        return false;
    }

    return true;
}

static int ClampColor255Component(float value) {
    return std::max(0, std::min(255, (int)std::lround(value)));
}

static Vector3 ClampColor255(const Vector3& color255) {
    return {
        (float)ClampColor255Component(color255.x),
        (float)ClampColor255Component(color255.y),
        (float)ClampColor255Component(color255.z)
    };
}

static Vector3 Color255ToUnit(const Vector3& color255) {
    const Vector3 clamped = ClampColor255(color255);
    return {
        clamped.x / 255.0f,
        clamped.y / 255.0f,
        clamped.z / 255.0f
    };
}

static bool ParseColor255Prop(const Entity& e, const char* key, Vector3& out) {
    Vector3 parsed{};
    if (!ParseVec3Prop(e, key, parsed)) {
        return false;
    }
    out = ClampColor255(parsed);
    return true;
}

static bool ParseUnitOr255ColorProp(const Entity& e, const char* key, Vector3& outUnit) {
    Vector3 parsed{};
    if (!ParseVec3Prop(e, key, parsed)) {
        return false;
    }
    const float maxComponent = std::max(parsed.x, std::max(parsed.y, parsed.z));
    if (maxComponent <= 1.0f) {
        outUnit = {
            std::clamp(parsed.x, 0.0f, 1.0f),
            std::clamp(parsed.y, 0.0f, 1.0f),
            std::clamp(parsed.z, 0.0f, 1.0f)
        };
        return true;
    }
    outUnit = Color255ToUnit(parsed);
    return true;
}

static bool ParseLightColor255AndBrightness(const Entity& e,
                                            Vector3& outColor255,
                                            float* outBrightness)
{
    auto it = e.properties.find("_light");
    if (it == e.properties.end()) {
        return false;
    }

    const std::vector<std::string> toks = SplitBySpace(it->second);
    if (toks.size() != 3 && toks.size() != 4) {
        return false;
    }

    try {
        outColor255 = ClampColor255({
            std::stof(toks[0]),
            std::stof(toks[1]),
            std::stof(toks[2])
        });
        if (outBrightness && toks.size() == 4) {
            *outBrightness = std::stof(toks[3]);
        }
    } catch (...) {
        return false;
    }

    return true;
}

static PointLightAttenuationMode DelayToAttenuationMode(int delay) {
    switch (delay) {
        case 0: return POINT_LIGHT_ATTEN_LINEAR;
        case 1: return POINT_LIGHT_ATTEN_INVERSE;
        case 2: return POINT_LIGHT_ATTEN_INVERSE_SQUARE;
        case 3: return POINT_LIGHT_ATTEN_NONE;
        case 4: return POINT_LIGHT_ATTEN_LOCAL_MINLIGHT;
        case 5: return POINT_LIGHT_ATTEN_INVERSE_SQUARE_B;
        default: return POINT_LIGHT_ATTEN_LINEAR;
    }
}

static Vector3 MangleToTBDirection(const Vector3& mangle) {
    const float yaw = mangle.x * DEG2RAD;
    const float pitch = mangle.y * DEG2RAD;
    return Vector3Normalize({
        cosf(pitch) * cosf(yaw),
        cosf(pitch) * sinf(yaw),
        sinf(pitch)
    });
}

static Vector3 MangleToWorldLightDirection(const Vector3& mangle) {
    return Vector3Normalize(ConvertTBtoRaylibEntities(MangleToTBDirection(mangle)));
}

static bool FindEntityOriginByTargetname(const Map& map, const std::string& targetname, Vector3& outWorldOrigin) {
    if (targetname.empty()) {
        return false;
    }
    for (const Entity& entity : map.entities) {
        auto it = entity.properties.find("targetname");
        if (it == entity.properties.end() || it->second != targetname) {
            continue;
        }
        Vector3 originTB{};
        if (!ParseVec3Prop(entity, "origin", originTB)) {
            continue;
        }
        outWorldOrigin = ConvertTBtoRaylibEntities(originTB);
        return true;
    }
    return false;
}

static bool ParseLightDirection(const Map& map, const Entity& e, const Vector3& lightOrigin, Vector3& outDirection) {
    auto targetIt = e.properties.find("target");
    if (targetIt != e.properties.end()) {
        Vector3 targetOrigin{};
        if (FindEntityOriginByTargetname(map, targetIt->second, targetOrigin)) {
            const Vector3 toTarget = Vector3Subtract(targetOrigin, lightOrigin);
            if (Vector3LengthSq(toTarget) > 1e-6f) {
                outDirection = Vector3Normalize(toTarget);
                return true;
            }
        }
    }

    Vector3 mangle{};
    if (ParseVec3Prop(e, "mangle", mangle) || ParseVec3Prop(e, "angles", mangle)) {
        outDirection = MangleToWorldLightDirection(mangle);
        return true;
    }

    return false;
}

static float ParseAngleScaleProp(const Entity& e, float defaultValue) {
    float value = defaultValue;
    ParseFloatProp(e, "_anglescale", value) || ParseFloatProp(e, "_anglesense", value);
    return std::clamp(value, 0.0f, 1.0f);
}

static int ParseDirtOverrideProp(const Entity& e, const char* key, int defaultValue) {
    int value = defaultValue;
    ParseIntProp(e, key, value);
    return value;
}

static Vector3 ReadLightBrushColor255(const Entity& e) {
    Vector3 color255{255, 255, 255};
    ParseColor255Prop(e, "_color", color255);
    return ClampColor255(color255);
}

static float ReadLightBrushIntensity(const Entity& e) {
    float intensity = 300.0f;
    ParseFloatProp(e, "intensity", intensity);
    return intensity;
}

static std::string EncodeLightBrushTextureName(const Vector3& color255) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "__light_brush_%d_%d_%d",
             ClampColor255Component(color255.x),
             ClampColor255Component(color255.y),
             ClampColor255Component(color255.z));
    return std::string(buffer);
}

static Vector3 PolygonCentroid(const std::vector<Vector3>& verts) {
    Vector3 centroid{0, 0, 0};
    if (verts.empty()) {
        return centroid;
    }
    for (const Vector3& v : verts) {
        centroid = Vector3Add(centroid, v);
    }
    return Vector3Scale(centroid, 1.0f / (float)verts.size());
}

static void FaceBasis(const Vector3& n, Vector3& u, Vector3& v) {
    Vector3 ref = (fabsf(n.y) < 0.9f) ? (Vector3){0, 1, 0} : (Vector3){1, 0, 0};
    u = Vector3Normalize(Vector3CrossProduct(n, ref));
    v = Vector3CrossProduct(n, u);
}

static bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py) {
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const float ex = poly[j].x - poly[i].x;
        const float ey = poly[j].y - poly[i].y;
        const float cx = px - poly[i].x;
        const float cy = py - poly[i].y;
        if (ex * cy - ey * cx < -1e-3f) {
            return false;
        }
    }
    return true;
}

static std::vector<Vector3> GenerateFaceEmitterSamples(const MapPolygon& poly,
                                                       float sampleSpacing,
                                                       float offsetAlongNormal,
                                                       int maxSamplesPerAxis) {
    Vector3 axisU{};
    Vector3 axisV{};
    FaceBasis(poly.normal, axisU, axisV);

    float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
    std::vector<Vector2> poly2d;
    poly2d.reserve(poly.verts.size());
    const Vector3 planeAnchor = poly.verts[0];
    const float anchorU = Vector3DotProduct(planeAnchor, axisU);
    const float anchorV = Vector3DotProduct(planeAnchor, axisV);
    for (const Vector3& v : poly.verts) {
        const float u = Vector3DotProduct(v, axisU);
        const float vv = Vector3DotProduct(v, axisV);
        minU = std::min(minU, u);
        maxU = std::max(maxU, u);
        minV = std::min(minV, vv);
        maxV = std::max(maxV, vv);
        poly2d.push_back({u, vv});
    }

    const float extentU = std::max(0.0f, maxU - minU);
    const float extentV = std::max(0.0f, maxV - minV);
    const float safeSpacing = std::max(1.0f, sampleSpacing);
    const int safeMaxPerAxis = std::max(1, maxSamplesPerAxis);
    const int samplesU = std::max(1, std::min(safeMaxPerAxis, (int)std::ceil(extentU / safeSpacing)));
    const int samplesV = std::max(1, std::min(safeMaxPerAxis, (int)std::ceil(extentV / safeSpacing)));
    const float stepU = (samplesU > 0) ? (extentU / (float)samplesU) : 0.0f;
    const float stepV = (samplesV > 0) ? (extentV / (float)samplesV) : 0.0f;

    std::vector<Vector3> samples;
    samples.reserve((size_t)samplesU * (size_t)samplesV);
    for (int y = 0; y < samplesV; ++y) {
        for (int x = 0; x < samplesU; ++x) {
            const float u = minU + ((float)x + 0.5f) * stepU;
            const float v = minV + ((float)y + 0.5f) * stepV;
            if (!InsidePoly2D(poly2d, u, v)) {
                continue;
            }
            Vector3 p = planeAnchor;
            p = Vector3Add(p, Vector3Scale(axisU, u - anchorU));
            p = Vector3Add(p, Vector3Scale(axisV, v - anchorV));
            p = Vector3Add(p, Vector3Scale(poly.normal, offsetAlongNormal));
            samples.push_back(p);
        }
    }

    if (samples.empty()) {
        samples.push_back(Vector3Add(PolygonCentroid(poly.verts), Vector3Scale(poly.normal, offsetAlongNormal)));
    }
    return samples;
}

static std::vector<Vector3> GenerateFaceEmitterSamples(const MapPolygon& poly) {
    return GenerateFaceEmitterSamples(poly, 32.0f, 1.0f, 8);
}

static uint32_t HashLightSeed(const Vector3& position, int extra) {
    auto h = [](float v) -> uint32_t {
        return (uint32_t)std::lround(v * 1000.0f);
    };
    uint32_t seed = 2166136261u;
    seed = (seed ^ h(position.x)) * 16777619u;
    seed = (seed ^ h(position.y)) * 16777619u;
    seed = (seed ^ h(position.z)) * 16777619u;
    seed = (seed ^ (uint32_t)extra) * 16777619u;
    return seed ? seed : 1u;
}

static float RandomFloat01(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return (float)(state & 0x00FFFFFFu) / (float)0x01000000u;
}

static Vector3 RandomPointInUnitSphere(uint32_t& state) {
    for (int attempt = 0; attempt < 16; ++attempt) {
        const Vector3 p = {
            RandomFloat01(state) * 2.0f - 1.0f,
            RandomFloat01(state) * 2.0f - 1.0f,
            RandomFloat01(state) * 2.0f - 1.0f
        };
        if (Vector3LengthSq(p) <= 1.0f) {
            return p;
        }
    }
    return {0.0f, 0.0f, 0.0f};
}

static void AppendDeviatedLights(const PointLight& baseLight,
                                 float deviance,
                                 int samples,
                                 int seedSalt,
                                 std::vector<PointLight>& out)
{
    if (deviance <= 0.0f || samples <= 1) {
        out.push_back(baseLight);
        return;
    }

    const int safeSamples = std::max(1, samples);
    const Vector3 sampleColor = Vector3Scale(baseLight.color, 1.0f / (float)safeSamples);
    uint32_t seed = HashLightSeed(baseLight.position, seedSalt);
    for (int i = 0; i < safeSamples; ++i) {
        PointLight split = baseLight;
        split.position = Vector3Add(baseLight.position, Vector3Scale(RandomPointInUnitSphere(seed), deviance));
        split.color = sampleColor;
        out.push_back(split);
    }
}

LightBakeSettings GetLightBakeSettings(const Map &map) {
    LightBakeSettings settings;

    for (const Entity& entity : map.entities) {
        if (!EntityHasClass(entity, "worldspawn")) {
            continue;
        }

        Vector3 ambient255{};
        if (ParseColor255Prop(entity, "_ambient", ambient255)) {
            settings.ambientColor = Color255ToUnit(ambient255);
        }

        float luxelSize = settings.luxelSize;
        if (ParseFloatProp(entity, "_world_units_per_luxel", luxelSize) ||
            ParseFloatProp(entity, "_lmscale", luxelSize)) {
            settings.luxelSize = std::max(0.125f, luxelSize);
        }

        int bounceCount = settings.bounceCount;
        if (ParseIntProp(entity, "_bounce", bounceCount)) {
            settings.bounceCount = std::max(0, bounceCount);
        }

        float bounceScale = settings.bounceScale;
        if (ParseFloatProp(entity, "_bouncescale", bounceScale)) {
            settings.bounceScale = std::max(0.0f, bounceScale);
        }

        ParseFloatProp(entity, "_sunlight", settings.sunlightIntensity);
        ParseFloatProp(entity, "_sunlight2", settings.sunlight2Intensity);
        ParseFloatProp(entity, "_sunlight3", settings.sunlight3Intensity);
        ParseFloatProp(entity, "_sunlight_penumbra", settings.sunlightPenumbra);
        settings.sunlightAngleScale = ParseAngleScaleProp(entity, settings.sunlightAngleScale);
        settings.dirt = ParseDirtOverrideProp(entity, "_dirt", settings.dirt);
        if (settings.dirt == -1) {
            settings.dirt = ParseDirtOverrideProp(entity, "_dirty", settings.dirt);
        }
        settings.sunlightDirt = ParseDirtOverrideProp(entity, "_sunlight_dirt", settings.sunlightDirt);
        settings.sunlight2Dirt = ParseDirtOverrideProp(entity, "_sunlight2_dirt", settings.sunlight2Dirt);
        ParseIntProp(entity, "_dirtmode", settings.dirtMode);
        ParseFloatProp(entity, "_dirtdepth", settings.dirtDepth);
        ParseFloatProp(entity, "_dirtscale", settings.dirtScale);
        ParseFloatProp(entity, "_dirtgain", settings.dirtGain);
        ParseFloatProp(entity, "_dirtangle", settings.dirtAngle);

        Vector3 parsedColor{};
        if (ParseUnitOr255ColorProp(entity, "_sunlight_color", parsedColor) ||
            ParseUnitOr255ColorProp(entity, "_sun_color", parsedColor)) {
            settings.sunlightColor = parsedColor;
        }
        if (ParseUnitOr255ColorProp(entity, "_sunlight_color2", parsedColor) ||
            ParseUnitOr255ColorProp(entity, "_sunlight2_color", parsedColor)) {
            settings.sunlight2Color = parsedColor;
        }
        if (ParseUnitOr255ColorProp(entity, "_sunlight_color3", parsedColor) ||
            ParseUnitOr255ColorProp(entity, "_sunlight3_color", parsedColor)) {
            settings.sunlight3Color = parsedColor;
        }

        Vector3 sunMangle{};
        if (ParseVec3Prop(entity, "_sunlight_mangle", sunMangle) ||
            ParseVec3Prop(entity, "_sun_mangle", sunMangle) ||
            ParseVec3Prop(entity, "_sun_angle", sunMangle)) {
            settings.sunlightDirection = Vector3Scale(MangleToWorldLightDirection(sunMangle), -1.0f);
        }

        break;
    }

    printf("[LightSettings] ambient=(%.2f,%.2f,%.2f) luxel=%.3f bounces=%d bounceScale=%.2f sun=%.1f sun2=%.1f sun3=%.1f dirt=%d\n",
           settings.ambientColor.x, settings.ambientColor.y, settings.ambientColor.z,
           settings.luxelSize, settings.bounceCount, settings.bounceScale,
           settings.sunlightIntensity, settings.sunlight2Intensity, settings.sunlight3Intensity,
           settings.dirt);
    return settings;
}

std::vector<PointLight> GetPointLights(const Map &map) {
    std::vector<PointLight> lights;
    int nextLightSeedSalt = 1;
    for (const Entity& e : map.entities) {
        if (EntityHasClass(e, "light_point")) {
            Vector3 originTB{0, 0, 0};
            Vector3 color255{255, 255, 255};
            ParseVec3Prop(e, "origin", originTB);
            ParseColor255Prop(e, "_color", color255);

            float intensity = 300.0f;
            ParseFloatProp(e, "intensity", intensity);

            PointLight pl{};
            pl.position = ConvertTBtoRaylibEntities(originTB);
            pl.color = Color255ToUnit(color255);
            pl.intensity = std::max(1.0f, intensity);
            pl.emissionNormal = { 0.0f, 0.0f, 0.0f };
            pl.directional = 0;
            pl.ignoreOccluderGroup = -1;
            pl.attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
            pl.angleScale = ParseAngleScaleProp(e, 1.0f);
            pl.dirt = ParseDirtOverrideProp(e, "_dirt", pl.dirt);
            ParseFloatProp(e, "_dirtscale", pl.dirtScale);
            ParseFloatProp(e, "_dirtgain", pl.dirtGain);
            float deviance = 0.0f;
            ParseFloatProp(e, "_deviance", deviance);
            int samples = 16;
            ParseIntProp(e, "_samples", samples);
            AppendDeviatedLights(pl, deviance, samples, nextLightSeedSalt++, lights);
            printf("LightPoint at (%.1f,%.1f,%.1f) radius=%.1f\n",
                   pl.position.x, pl.position.y, pl.position.z, pl.intensity);
            continue;
        }

        if (!EntityClassStartsWith(e, "light") || EntityHasClass(e, "light_brush")) {
            continue;
        }
        if (e.properties.find("_surface") != e.properties.end()) {
            continue;
        }

        Vector3 originTB{0, 0, 0};
        ParseVec3Prop(e, "origin", originTB);
        const Vector3 origin = ConvertTBtoRaylibEntities(originTB);

        Vector3 color255{255, 255, 255};
        float brightness = 300.0f;
        bool hasLightColor = ParseLightColor255AndBrightness(e, color255, &brightness);
        if (!hasLightColor) {
            ParseColor255Prop(e, "_color", color255);
        }
        ParseFloatProp(e, "light", brightness);

        int delay = 0;
        ParseIntProp(e, "delay", delay);

        float wait = 1.0f;
        ParseFloatProp(e, "wait", wait);

        PointLight pl{};
        pl.position = origin;
        pl.color = Color255ToUnit(color255);
        pl.intensity = std::max(1.0f, brightness * std::max(0.01f, wait));
        pl.emissionNormal = { 0.0f, 0.0f, 0.0f };
        pl.directional = 0;
        pl.ignoreOccluderGroup = -1;
        pl.attenuationMode = DelayToAttenuationMode(delay);
        pl.angleScale = ParseAngleScaleProp(e, 0.5f);
        pl.dirt = ParseDirtOverrideProp(e, "_dirt", pl.dirt);
        ParseFloatProp(e, "_dirtscale", pl.dirtScale);
        ParseFloatProp(e, "_dirtgain", pl.dirtGain);

        Vector3 lightDirection{};
        if (ParseLightDirection(map, e, origin, lightDirection)) {
            float outerAngle = 40.0f;
            ParseFloatProp(e, "angle", outerAngle);
            float softAngle = 0.0f;
            ParseFloatProp(e, "_softangle", softAngle);
            const float outerHalf = std::clamp(outerAngle, 1.0f, 179.0f) * 0.5f * DEG2RAD;
            const float innerHalf = std::clamp((softAngle > 0.0f) ? softAngle : outerAngle, 1.0f, std::clamp(outerAngle, 1.0f, 179.0f)) * 0.5f * DEG2RAD;
            pl.spotDirection = lightDirection;
            pl.spotOuterCos = cosf(outerHalf);
            pl.spotInnerCos = cosf(innerHalf);
        }

        float deviance = 0.0f;
        ParseFloatProp(e, "_deviance", deviance);
        int samples = 16;
        ParseIntProp(e, "_samples", samples);
        AppendDeviatedLights(pl, deviance, samples, nextLightSeedSalt++, lights);
        printf("Light at (%.1f,%.1f,%.1f) radius=%.1f delay=%d wait=%.2f\n",
               pl.position.x, pl.position.y, pl.position.z, pl.intensity, delay, wait);
    }

    int nextLightBrushGroup = 0;
    for (const Entity& entity : map.entities) {
        if (!EntityHasClass(entity, "light_brush")) continue;

        const Vector3 color255 = ReadLightBrushColor255(entity);
        const float intensity = ReadLightBrushIntensity(entity);

        for (const Brush& brush : entity.brushes) {
            const int lightBrushGroup = nextLightBrushGroup++;
            std::vector<MapPolygon> polys;
            const int nF = (int)brush.faces.size();
            if (nF < 3) continue;

            std::vector<Plane> planes(nF);
            for (int i = 0; i < nF; ++i) {
                planes[i].normal = brush.faces[i].normal;
                planes[i].d = Vector3DotProduct(brush.faces[i].normal,
                                                brush.faces[i].vertices[0]);
            }

            std::vector<std::vector<Vector3>> facePolys(nF);
            for (int i = 0; i < nF - 2; ++i)
              for (int j = i + 1; j < nF - 1; ++j)
                for (int k = j + 1; k < nF; ++k) {
                    Vector3 ip;
                    if (!GetIntersection(planes[i], planes[j], planes[k], ip)) continue;
                    bool inside = true;
                    for (int m = 0; m < nF; ++m) {
                        float d = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                        if (d > (float)epsilon) { inside = false; break; }
                    }
                    if (inside) {
                        facePolys[i].push_back(ip);
                        facePolys[j].push_back(ip);
                        facePolys[k].push_back(ip);
                    }
                }

            for (int i = 0; i < nF; ++i) {
                RemoveDuplicatePoints(facePolys[i], (float)epsilon);
                if (facePolys[i].size() < 3) continue;

                Vector3 nTB = brush.faces[i].normal;
                SortPolygonVertices(facePolys[i], nTB);
                for (auto& p : facePolys[i]) p = ConvertTBtoRaylib(p);

                Vector3 nRL = CalculateNormal(facePolys[i][0], facePolys[i][1], facePolys[i][2]);
                if (Vector3DotProduct(nRL, ConvertTBtoRaylib(nTB)) < 0.f) {
                    std::reverse(facePolys[i].begin(), facePolys[i].end());
                    nRL = CalculateNormal(facePolys[i][0], facePolys[i][1], facePolys[i][2]);
                }

                MapPolygon mp;
                mp.verts = std::move(facePolys[i]);
                mp.normal = nRL;
                mp.occluderGroup = lightBrushGroup;
                polys.push_back(std::move(mp));
            }

            for (const MapPolygon& poly : polys) {
                const std::vector<Vector3> samples = GenerateFaceEmitterSamples(poly);
                const float invSampleCount = 1.0f / (float)samples.size();
                const Vector3 sampleColor = {
                    (color255.x / 255.0f) * invSampleCount,
                    (color255.y / 255.0f) * invSampleCount,
                    (color255.z / 255.0f) * invSampleCount
                };
                for (const Vector3& samplePos : samples) {
                    PointLight pl{};
                    pl.position = samplePos;
                    pl.color = sampleColor;
                    pl.intensity = intensity;
                    pl.emissionNormal = poly.normal;
                    pl.directional = 1;
                    pl.ignoreOccluderGroup = -1;
                    pl.attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
                    pl.angleScale = 1.0f;
                    lights.push_back(pl);
                }
                printf("LightBrush face emitters: %zu samples radius=%.1f normal=(%.2f,%.2f,%.2f)\n",
                       samples.size(), intensity, poly.normal.x, poly.normal.y, poly.normal.z);
            }
        }
    }
    return lights;
}

std::vector<SurfaceLightTemplate> GetSurfaceLightTemplates(const Map& map) {
    std::vector<SurfaceLightTemplate> templates;
    for (const Entity& e : map.entities) {
        if (!EntityClassStartsWith(e, "light")) {
            continue;
        }

        auto surfaceIt = e.properties.find("_surface");
        if (surfaceIt == e.properties.end() || surfaceIt->second.empty()) {
            continue;
        }

        Vector3 color255{255, 255, 255};
        float brightness = 300.0f;
        bool hasLightColor = ParseLightColor255AndBrightness(e, color255, &brightness);
        if (!hasLightColor) {
            ParseColor255Prop(e, "_color", color255);
        }
        ParseFloatProp(e, "light", brightness);

        int delay = 0;
        ParseIntProp(e, "delay", delay);

        float wait = 1.0f;
        ParseFloatProp(e, "wait", wait);

        SurfaceLightTemplate templ{};
        templ.texture = surfaceIt->second;
        ParseIntProp(e, "_surflight_group", templ.surfaceLightGroup);
        ParseFloatProp(e, "_surface_offset", templ.surfaceOffset);
        ParseIntProp(e, "_surface_spotlight", templ.surfaceSpotlight);
        ParseFloatProp(e, "_deviance", templ.deviance);
        ParseIntProp(e, "_samples", templ.devianceSamples);
        templ.light.color = Vector3Scale(Color255ToUnit(color255), std::max(0.0f, brightness) / 300.0f);
        templ.light.intensity = std::max(1.0f, brightness * std::max(0.01f, wait));
        templ.light.attenuationMode = DelayToAttenuationMode(delay);
        templ.light.angleScale = ParseAngleScaleProp(e, 0.5f);
        templ.light.dirt = ParseDirtOverrideProp(e, "_dirt", templ.light.dirt);
        ParseFloatProp(e, "_dirtscale", templ.light.dirtScale);
        ParseFloatProp(e, "_dirtgain", templ.light.dirtGain);

        Vector3 originTB{};
        ParseVec3Prop(e, "origin", originTB);
        const Vector3 origin = ConvertTBtoRaylibEntities(originTB);
        Vector3 lightDirection{};
        if (ParseLightDirection(map, e, origin, lightDirection)) {
            float outerAngle = 40.0f;
            ParseFloatProp(e, "angle", outerAngle);
            float softAngle = 0.0f;
            ParseFloatProp(e, "_softangle", softAngle);
            templ.light.spotDirection = lightDirection;
            templ.light.spotOuterCos = cosf(std::clamp(outerAngle, 1.0f, 179.0f) * 0.5f * DEG2RAD);
            templ.light.spotInnerCos = cosf(std::clamp((softAngle > 0.0f) ? softAngle : outerAngle, 1.0f, std::clamp(outerAngle, 1.0f, 179.0f)) * 0.5f * DEG2RAD);
        }

        templates.push_back(std::move(templ));
    }
    return templates;
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
static void AppendBrushEntityPolygons(const Entity& entity,
                                      int sourceEntityId,
                                      bool devMode,
                                      bool exteriorOnly,
                                      int* nextLightBrushGroup,
                                      int* nextSourceBrushId,
                                      size_t* sourceFaceCount,
                                      std::vector<MapPolygon>& out) {
    const bool isLightBrush = EntityHasClass(entity, "light_brush");
    const std::string lightBrushTexture = isLightBrush
        ? EncodeLightBrushTextureName(ReadLightBrushColor255(entity))
        : std::string();

    if (!ShouldRenderBrushEntity(entity, devMode)) {
        return;
    }

    int phongEnabled = 0;
    ParseIntProp(entity, "_phong", phongEnabled);
    float phongAngle = 89.0f;
    if (ParseFloatProp(entity, "_phong_angle", phongAngle)) {
        phongEnabled = 1;
    }
    float phongAngleConcave = 0.0f;
    ParseFloatProp(entity, "_phong_angle_concave", phongAngleConcave);
    int phongGroup = 0;
    ParseIntProp(entity, "_phong_group", phongGroup);
    int surfaceLightGroup = 0;
    ParseIntProp(entity, "_surflight_group", surfaceLightGroup);

    std::vector<MapPolygon> entityPolys;

    for (auto& brush : entity.brushes) {
        const int lightBrushGroup = (isLightBrush && nextLightBrushGroup) ? (*nextLightBrushGroup)++ : -1;
        const int sourceBrushId = nextSourceBrushId ? (*nextSourceBrushId)++ : -1;

        const int nF = (int)brush.faces.size();
        if (nF < 3) continue;

        std::vector<Plane> planes(nF);
        for (int i = 0; i < nF; ++i) {
            planes[i].normal = brush.faces[i].normal;
            planes[i].d = Vector3DotProduct(brush.faces[i].normal,
                                            brush.faces[i].vertices[0]);
        }

        std::vector<std::vector<Vector3>> polys(nF);
        for (int i = 0; i < nF - 2; ++i)
          for (int j = i + 1; j < nF - 1; ++j)
            for (int k = j + 1; k < nF; ++k) {
                Vector3 ip;
                if (!GetIntersection(planes[i], planes[j], planes[k], ip)) continue;
                bool inside = true;
                for (int m = 0; m < nF; ++m) {
                    float d = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                    if (d > (float)epsilon) { inside = false; break; }
                }
                if (inside) {
                    polys[i].push_back(ip);
                    polys[j].push_back(ip);
                    polys[k].push_back(ip);
                }
            }

        for (int i = 0; i < nF; ++i) {
            RemoveDuplicatePoints(polys[i], (float)epsilon);
            if (polys[i].size() < 3) continue;

            Vector3 nTB = brush.faces[i].normal;
            SortPolygonVertices(polys[i], nTB);
            for (auto& p : polys[i]) p = ConvertTBtoRaylib(p);
            const Vector3 expectedNormalRL = ConvertTBtoRaylib(nTB);
            SortPolygonVertices(polys[i], expectedNormalRL);
            RemoveDuplicatePoints(polys[i], 0.1f);
            CleanupClippedPolygon(polys[i], expectedNormalRL);
            if (polys[i].size() < 3) continue;

            Vector3 nRL = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
            if (Vector3DotProduct(nRL, expectedNormalRL) < 0.f) {
                std::reverse(polys[i].begin(), polys[i].end());
                nRL = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
            }

            MapPolygon mp;
            mp.verts    = std::move(polys[i]);
            mp.normal   = nRL;
            mp.texture  = isLightBrush ? lightBrushTexture : brush.faces[i].texture;
            mp.occluderGroup = lightBrushGroup;
            mp.sourceBrushId = sourceBrushId;
            mp.sourceEntityId = sourceEntityId;
            mp.sourceFaceIndex = i;
            mp.surfaceLightGroup = surfaceLightGroup;
            mp.phong = phongEnabled != 0;
            mp.phongAngle = std::max(1.0f, phongAngle);
            mp.phongAngleConcave = std::max(0.0f, phongAngleConcave);
            mp.phongGroup = phongGroup;
            mp.texAxisU = ConvertTBtoRaylib(brush.faces[i].textureAxes1);
            mp.texAxisV = ConvertTBtoRaylib(brush.faces[i].textureAxes2);
            mp.offU     = brush.faces[i].offsetX;
            mp.offV     = brush.faces[i].offsetY;
            mp.rot      = brush.faces[i].rotation;
            mp.scaleU   = brush.faces[i].scaleX;
            mp.scaleV   = brush.faces[i].scaleY;
            entityPolys.push_back(std::move(mp));
            if (sourceFaceCount) {
                ++(*sourceFaceCount);
            }
        }
    }

    if (exteriorOnly) {
        std::vector<MapPolygon> exteriorPolys = BuildExteriorPolygons(entityPolys);
        out.insert(out.end(),
                   std::make_move_iterator(exteriorPolys.begin()),
                   std::make_move_iterator(exteriorPolys.end()));
    } else {
        out.insert(out.end(),
                   std::make_move_iterator(entityPolys.begin()),
                   std::make_move_iterator(entityPolys.end()));
    }
}

std::vector<MapPolygon> BuildMapPolygons(const Map &map, bool devMode)
{
    std::vector<MapPolygon> out;
    int nextLightBrushGroup = 0;
    int nextSourceBrushId = 0;
    size_t sourceFaceCount = 0;
    for (size_t entityIndex = 0; entityIndex < map.entities.size(); ++entityIndex) {
        AppendBrushEntityPolygons(map.entities[entityIndex], (int)entityIndex, devMode, false, &nextLightBrushGroup, &nextSourceBrushId, &sourceFaceCount, out);
    }
    printf("[MapParser] Built %zu polygons from %zu brush faces.\n", out.size(), sourceFaceCount);
    return out;
}

std::vector<MapPolygon> BuildExteriorMapPolygons(const Map &map, bool devMode)
{
    std::vector<MapPolygon> out;
    int nextLightBrushGroup = 0;
    int nextSourceBrushId = 0;
    size_t sourceFaceCount = 0;
    for (size_t entityIndex = 0; entityIndex < map.entities.size(); ++entityIndex) {
        AppendBrushEntityPolygons(map.entities[entityIndex], (int)entityIndex, devMode, true, &nextLightBrushGroup, &nextSourceBrushId, &sourceFaceCount, out);
    }
    printf("[MapParser] Built %zu exterior occluder polygons from %zu brush faces.\n", out.size(), sourceFaceCount);
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
