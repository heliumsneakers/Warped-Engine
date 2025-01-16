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

// Layers for checking collisions, 3 will not be computed but needs to be explicitly written.
// NOTE: Handle different layer assignments in entities.cpp
namespace World_Layers {
	static constexpr JPH::ObjectLayer WORLD_STATIC		= 0;
	static constexpr JPH::ObjectLayer WORLD_DYNAMIC		= 1;
	static constexpr JPH::ObjectLayer WORLD_TRIGGER		= 2;
	static constexpr JPH::ObjectLayer WORLD_NO_COLLIDE	= 3;
};
// Broad phase layers for checking when we actually need to compute collisions.
namespace BP_Layers {
	static constexpr JPH::BroadPhaseLayer BP_STATIC		(0);
	static constexpr JPH::BroadPhaseLayer BP_DYNAMIC	(1);
	static constexpr JPH::BroadPhaseLayer BP_TRIGGER	(2);
};

void BuildMapPhysics(MeshCollisionData &meshCollisionData, JPH::BodyInterface &bodyInterface);
