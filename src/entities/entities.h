#pragma once

#include "../math/wmath.h"
#include "../utils/map_parser.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyID.h"

namespace JPH {
class PhysicsSystem;
class Shape;
}

namespace GameplayEntities {

struct PlayerEffectResult {
    bool appliedBoost = false;
    bool launchOffGround = false;
};

struct TriggerTeleportResult {
    bool teleportPlayer = false;
    Vector3 position = Vector3Zero();
    float yaw = 0.0f;
    float pitch = 0.0f;
};

void Reset();

void RegisterPointEntities(const std::vector<Entity>& entities);

void RegisterBrushEntity(const Entity& entity, int entityIndex, JPH::BodyID bodyID);

PlayerEffectResult ApplyPlayerEffects(JPH::PhysicsSystem* physicsSystem,
                                      const JPH::Shape* playerShape,
                                      Vector3 playerCenter,
                                      Vector3 groundNormal,
                                      bool isGrounded,
                                      float deltaTime,
                                      Vector3& inOutVelocity);

TriggerTeleportResult QueryPlayerTeleportTrigger(JPH::PhysicsSystem* physicsSystem,
                                                 const JPH::Shape* playerShape,
                                                 Vector3 playerCenter,
                                                 float deltaTime);

} // namespace GameplayEntities
