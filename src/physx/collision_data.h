#pragma once

#include <vector>
#include "raylib.h"

// Struct to store vertices for collision data

struct MeshCollisionData {
    std::vector<Vector3> vertices;
};

std::vector<MeshCollisionData> ExtractCollisionData(const Model &model);
