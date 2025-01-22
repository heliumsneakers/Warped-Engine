#pragma once

#include "Jolt/Core/Core.h"
#include "collision_data.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/Shape/ConvexShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Geometry/ConvexSupport.h"
#include "Jolt/Geometry/GJKClosestPoint.h"
#include "Jolt/Physics/Body/BodyInterface.h"
#include <vector>


extern JPH::BodyID debugSphereID;

namespace JPH {
    class PhysicsSystem;
    class BodyInterface;
}

namespace Layers
{
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING     = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

namespace BroadPhaseLayers
{
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr uint NUM_LAYERS(2);
}

void InitPhysicsSystem();

void ShutdownPhysicsSystem();

JPH::BodyInterface &GetBodyInterface();

void UpdatePhysicsSystem(float delta_time, JPH::BodyInterface *bodyInterface);

void SpawnDebugPhysObj(JPH::BodyInterface *bodyInterface);

void BuildMapPhysics(std::vector<MeshCollisionData> &meshCollisionData, JPH::BodyInterface *bodyInterface);

void SpawnMinimalTest(JPH::BodyInterface &bodyInterface);

