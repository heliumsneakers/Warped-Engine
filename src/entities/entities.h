#pragma once

#include "../math/wmath.h"
#include "../utils/map_types.h"
#include "box3d/box3d.h"

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

void RegisterBrushEntity(const Entity& entity, int entityIndex, b3BodyId bodyId);

PlayerEffectResult ApplyPlayerEffects(const b3ShapeProxy* playerProxy,
                                      Vector3 playerCenter,
                                      Vector3 groundNormal,
                                      bool isGrounded,
                                      float deltaTime,
                                      Vector3& inOutVelocity);

TriggerTeleportResult QueryPlayerTeleportTrigger(const b3ShapeProxy* playerProxy,
                                                 Vector3 playerCenter,
                                                 float deltaTime);

} // namespace GameplayEntities
