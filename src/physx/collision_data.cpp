#include "raylib.h"
#include "raymath.h"
#include "collision_data.h"
#include "../utils/map_parser.h"
#include <vector>
#include <string>

static CollisionType GetEntityCollisionType(const Entity &ent) {
    CollisionType ct = CollisionType::UNKNOWN;

    auto it = ent.properties.find("classname");
    if (it != ent.properties.end()) {
        const std::string &classname = it->second;

        if (classname == "worldspawn") {
           ct = CollisionType::STATIC;
            printf("\n WORLDSPAWN ENTITY \n");
        }
        else if (classname.find("trigger_once") != std::string::npos) {
            ct = CollisionType::TRIGGER;
            printf("\n TRIGGER_ONCE ENTITY \n");

        }
        else if (classname.find("func_physics") != std::string::npos) {
            ct = CollisionType::DYNAMIC;
            printf("\n FUNC_PHYSICS ENTITY \n");
        }
        else if (classname.find("func_detail") != std::string::npos) {
            ct = CollisionType::NO_COLLIDE;
            printf("\n FUNC_DETAIL ENTITY \n");
        }
    }

    return ct;
}

static std::vector<Vector3> BuildBrushGeometry(const Brush &brush) {
    std::vector<Vector3> finalPoints; 

    // 1) Gather all planes in "TB" coords
    int numFaces = (int)brush.faces.size();
    if (numFaces < 3) {
        // If not enough planes, return empty
        return finalPoints;
    }

    std::vector<Plane> planes;
    planes.reserve(numFaces);
    for (auto &face : brush.faces) {
        Plane p;
        p.normal = face.normal;  //from .map parser (TrenchBroom coords)
        p.d      = Vector3DotProduct(face.normal, face.vertices[0]);
        planes.push_back(p);
    }

    // 2) Build polygons per face by intersecting planes:
    //    We'll store them in 'polys[face_index]'
    std::vector<std::vector<Vector3>> polys(numFaces);

    // Triple-plane intersection: For each combo (i,j,k)
    for (int i = 0; i < numFaces - 2; ++i) {
        for (int j = i + 1; j < numFaces - 1; ++j) {
            for (int k = j + 1; k < numFaces; ++k) {
                Vector3 ip;
                if (GetIntersection(planes[i], planes[j], planes[k], ip)) {
                    // Check if 'ip' is inside *all* planes => inside the brush
                    bool inside = true;
                    for (int m = 0; m < numFaces; ++m) {
                        float dist = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                        if (dist > (float)epsilon) {
                            inside = false;
                            break;
                        }
                    }
                    if (inside) {
                        // This intersection belongs to faces i, j, k
                        polys[i].push_back(ip);
                        polys[j].push_back(ip);
                        polys[k].push_back(ip);
                    }
                }
            }
        }
    }

    // 3) For each face, remove duplicates, optionally sort polygon, convert coords
    for (int i = 0; i < numFaces; ++i) {
        // remove duplicates
        RemoveDuplicatePoints(polys[i], (float)epsilon);

        if (polys[i].size() < 3) {
            // skip degenerate face
            continue;
        }

        // The original face normal in TB coords
        Vector3 faceNormalTB = brush.faces[i].normal;

        // Sort face's polygon by angle around centroid (like in MapToMesh)
        SortPolygonVertices(polys[i], faceNormalTB);

        // Convert each point from TB -> Raylib coords
        for (auto &pt : polys[i]) {
            pt = ConvertTBtoRaylib(pt);
        }

        // Now check if the polygon normal lines up with the original face normal
        // in Raylib coords. If it's flipped, reverse the polygon.
        Vector3 polyNormal = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
        Vector3 faceNormalRL = ConvertTBtoRaylib(faceNormalTB);

        if (Vector3DotProduct(polyNormal, faceNormalRL) < 0.f) {
            std::reverse(polys[i].begin(), polys[i].end());
            // recalc normal
            polyNormal = CalculateNormal(polys[i][0], polys[i][1], polys[i][2]);
        }

        // 4) Append these face vertices to 'finalPoints'
        //    (This means every face's polygon points end up in one big array.)
        //    That can be used to build a convex hull in Jolt
        for (auto &v : polys[i]) {
            finalPoints.push_back(v);
        }
    }

    // 5) Remove duplicates *again* in finalPoints to avoid overlap across faces
    RemoveDuplicatePoints(finalPoints, (float)epsilon);

    // Return the final point set
    return finalPoints;
}

std::vector<MeshCollisionData> ExtractCollisionData(const Map &map)
{
    std::vector<MeshCollisionData> result;

    // For each entity
    for (auto &ent : map.entities) {
        // 1) Determine collision type from entity
        CollisionType ct = GetEntityCollisionType(ent);

        // 2) For each brush in this entity, build geometry
        for (auto &brush : ent.brushes) {
            std::vector<Vector3> corners = BuildBrushGeometry(brush);

            if (corners.empty()) 
                continue; // skip invalid brush

            // 3) Store mesh collision data with type and points
            MeshCollisionData mcd;
            mcd.collisionType = ct;
            mcd.vertices      = std::move(corners);

            result.push_back(std::move(mcd));
        }
    }

    return result;
}
