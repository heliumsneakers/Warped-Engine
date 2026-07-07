#pragma once

#include "collision_data.h"
#include "box3d/box3d.h"
#include <vector>

extern b3WorldId g_physicsWorld;
extern b3BodyId  debugSphereID;

// Collision category bits. Box3D filters with category/mask bit pairs instead
// of Jolt-style object layers.
namespace Layers
{
    constexpr uint64_t STATIC = 0x1; // non-moving world brushes
    constexpr uint64_t MOVING = 0x2; // dynamic bodies and the player queries
    constexpr uint64_t SENSOR = 0x4; // trigger volumes
}

void InitPhysicsSystem();

void ShutdownPhysicsSystem();

void UpdatePhysicsSystem(float delta_time);

void SpawnDebugPhysObj();

void BuildMapPhysics(const std::vector<MeshCollisionData> &meshCollisionData,
                     const std::vector<Entity> &entities);
