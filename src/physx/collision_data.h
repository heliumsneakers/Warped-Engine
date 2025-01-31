#pragma once

#include <vector>
#include "raylib.h"
#include "../utils/map_parser.h"

enum class CollisionType {
    STATIC,
    DYNAMIC,
    TRIGGER,
    NO_COLLIDE,
    UNKNOWN,
};

struct MeshCollisionData {
    std::vector<Vector3> vertices;
    CollisionType collisionType;
};

static CollisionType GetEntityCollisionType(const Entity &ent);
static std::vector<Vector3> BuildBrushGeometry(const Brush &brush);
std::vector<MeshCollisionData> ExtractCollisionData(const Map &map);
