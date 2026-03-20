#pragma once

#include <vector>
#include "../math/wmath.h"
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

std::vector<MeshCollisionData> ExtractCollisionData(const Map &map);
