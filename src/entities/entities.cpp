#include "entities.h"

#include "../physx/physics.h"

#include "Jolt/Jolt.h"
#include "Jolt/Math/Vec3.h"
#include "Jolt/Physics/Collision/BackFaceMode.h"
#include "Jolt/Physics/Collision/CollideShape.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"
#include "Jolt/Physics/PhysicsSystem.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace GameplayEntities {
namespace {

static constexpr int kBoostAmountMin = 1;
static constexpr int kBoostAmountMax = 1000;
static constexpr int kAccelerationMin = 100;
static constexpr int kAccelerationMax = 1000;
static constexpr int kDefaultBoostAmount = 100;
static constexpr int kDefaultAcceleration = 100;
static constexpr Vector3 kDefaultDirectionTB = { 0.0f, 0.0f, 1.0f };
static constexpr float kBoostTouchDistance = 0.1f;
static constexpr float kTriggerTouchDistance = 0.1f;

struct BoostVolume {
    int entityIndex = -1;
    JPH::BodyID bodyID;
    Vector3 direction = { 0.0f, 1.0f, 0.0f };
    float boostAmount = (float)kDefaultBoostAmount;
    float acceleration = (float)kDefaultAcceleration;
};

struct ActiveBoostState {
    float targetAlongDirection = 0.0f;
};

struct CheckPoint {
    int entityIndex = -1;
    std::string targetname;
    Vector3 position = Vector3Zero();
    float yaw = 0.0f;
    float pitch = 0.0f;
};

struct TeleportTrigger {
    int entityIndex = -1;
    JPH::BodyID bodyID;
    std::string target;
    bool triggerOnce = false;
    bool enabled = true;
    bool consumed = false;
    float wait = 0.0f;
    double nextFireTime = 0.0;
};

std::vector<BoostVolume> sBoostVolumes;
std::unordered_map<JPH::BodyID, size_t> sBoostVolumesByBody;
std::unordered_map<JPH::BodyID, ActiveBoostState> sActivePlayerBoosts;
std::vector<CheckPoint> sCheckPoints;
std::unordered_map<std::string, size_t> sCheckPointsByTargetname;
std::vector<TeleportTrigger> sTeleportTriggers;
std::unordered_map<JPH::BodyID, size_t> sTeleportTriggersByBody;
std::unordered_set<JPH::BodyID> sActivePlayerTriggerContacts;
double sGameplayTime = 0.0;

class BoostTouchLayerFilter final : public JPH::ObjectLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer) const override
    {
        return inLayer == Layers::NON_MOVING;
    }
};

BoostTouchLayerFilter sBoostTouchLayerFilter;

class SensorTouchLayerFilter final : public JPH::ObjectLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer) const override
    {
        return inLayer == Layers::SENSOR;
    }
};

SensorTouchLayerFilter sSensorTouchLayerFilter;

static inline JPH::RVec3 ToJoltRVec3(Vector3 v)
{
    return JPH::RVec3(v.x, v.y, v.z);
}

static bool ParseVec3Property(const Entity& entity, const char* key, Vector3& out)
{
    auto it = entity.properties.find(key);
    if (it == entity.properties.end()) {
        return false;
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (std::sscanf(it->second.c_str(), "%f %f %f", &x, &y, &z) != 3) {
        return false;
    }

    out = { x, y, z };
    return true;
}

static bool ParseStringProperty(const Entity& entity, const char* key, std::string& out)
{
    auto it = entity.properties.find(key);
    if (it == entity.properties.end()) {
        return false;
    }
    out = it->second;
    return true;
}

static int ParseClampedIntProperty(const Entity& entity, const char* key, int defaultValue, int minValue, int maxValue)
{
    auto it = entity.properties.find(key);
    if (it == entity.properties.end()) {
        return defaultValue;
    }

    try {
        const int parsed = (int)std::lround(std::stof(it->second));
        return std::clamp(parsed, minValue, maxValue);
    } catch (...) {
        auto classnameIt = entity.properties.find("classname");
        const char* classname = classnameIt != entity.properties.end() ? classnameIt->second.c_str() : "?";
        printf("[entities] %s has invalid %s='%s', using default %d\n",
               classname,
               key,
               it->second.c_str(),
               defaultValue);
        return defaultValue;
    }
}

static float ParseFloatProperty(const Entity& entity, const char* key, float defaultValue, float minValue)
{
    auto it = entity.properties.find(key);
    if (it == entity.properties.end()) {
        return defaultValue;
    }

    try {
        return std::max(minValue, std::stof(it->second));
    } catch (...) {
        auto classnameIt = entity.properties.find("classname");
        const char* classname = classnameIt != entity.properties.end() ? classnameIt->second.c_str() : "?";
        printf("[entities] %s has invalid %s='%s', using default %.2f\n",
               classname,
               key,
               it->second.c_str(),
               defaultValue);
        return defaultValue;
    }
}

static Vector3 ParseBoostDirection(const Entity& entity)
{
    Vector3 directionTB = kDefaultDirectionTB;
    Vector3 parsedTB{};
    if (ParseVec3Property(entity, "direction", parsedTB)) {
        directionTB = parsedTB;
    }

    Vector3 directionWorld = ConvertTBPointEntityToWorld(directionTB);
    if (Vector3LengthSq(directionWorld) <= 1.0e-6f) {
        auto classnameIt = entity.properties.find("classname");
        const char* classname = classnameIt != entity.properties.end() ? classnameIt->second.c_str() : "?";
        printf("[entities] %s has zero direction, using default 0 0 1\n", classname);
        directionWorld = ConvertTBPointEntityToWorld(kDefaultDirectionTB);
    }

    return Vector3Normalize(directionWorld);
}

static bool ApplyBoostVolume(const BoostVolume& boostVolume,
                             const ActiveBoostState& activeBoostState,
                             float deltaTime,
                             Vector3& inOutVelocity)
{
    const float currentAlongDirection = Vector3DotProduct(inOutVelocity, boostVolume.direction);
    if (currentAlongDirection >= activeBoostState.targetAlongDirection) {
        return false;
    }

    const float velocityStep = boostVolume.acceleration * boostVolume.boostAmount * deltaTime;
    const float addSpeed = std::min(velocityStep, activeBoostState.targetAlongDirection - currentAlongDirection);
    inOutVelocity = Vector3Add(inOutVelocity, Vector3Scale(boostVolume.direction, addSpeed));
    return addSpeed > 0.0f;
}

} // namespace

void Reset()
{
    sBoostVolumes.clear();
    sBoostVolumesByBody.clear();
    sActivePlayerBoosts.clear();
    sCheckPoints.clear();
    sCheckPointsByTargetname.clear();
    sTeleportTriggers.clear();
    sTeleportTriggersByBody.clear();
    sActivePlayerTriggerContacts.clear();
    sGameplayTime = 0.0;
}

void RegisterPointEntities(const std::vector<Entity>& entities)
{
    for (size_t entityIndex = 0; entityIndex < entities.size(); ++entityIndex) {
        const Entity& entity = entities[entityIndex];

        auto classnameIt = entity.properties.find("classname");
        if (classnameIt == entity.properties.end() || classnameIt->second != "check_point") {
            continue;
        }

        std::string targetname;
        if (!ParseStringProperty(entity, "targetname", targetname) || targetname.empty()) {
            printf("[entities] check_point entity=%zu is missing targetname and cannot be targeted\n", entityIndex);
            continue;
        }

        Vector3 originTB{};
        if (!ParseVec3Property(entity, "origin", originTB)) {
            printf("[entities] check_point '%s' is missing origin\n", targetname.c_str());
            continue;
        }

        CheckPoint checkPoint;
        checkPoint.entityIndex = (int)entityIndex;
        checkPoint.targetname = targetname;
        checkPoint.position = ConvertTBPointEntityToWorld(originTB);
        ParsePointEntityFacing(entity, checkPoint.yaw, checkPoint.pitch);

        sCheckPointsByTargetname[targetname] = sCheckPoints.size();
        sCheckPoints.push_back(checkPoint);

        printf("[entities] registered check_point '%s' entity=%zu pos=(%.1f %.1f %.1f) yaw=%.1f pitch=%.1f\n",
               targetname.c_str(),
               entityIndex,
               checkPoint.position.x,
               checkPoint.position.y,
               checkPoint.position.z,
               checkPoint.yaw,
               checkPoint.pitch);
    }
}

void RegisterBrushEntity(const Entity& entity, int entityIndex, JPH::BodyID bodyID)
{
    auto classnameIt = entity.properties.find("classname");
    if (classnameIt == entity.properties.end()) {
        return;
    }

    const std::string& classname = classnameIt->second;

    if (classname == "func_boost") {
        BoostVolume boostVolume;
        boostVolume.entityIndex = entityIndex;
        boostVolume.bodyID = bodyID;
        boostVolume.direction = ParseBoostDirection(entity);
        boostVolume.boostAmount = (float)ParseClampedIntProperty(entity, "boost", kDefaultBoostAmount, kBoostAmountMin, kBoostAmountMax);
        boostVolume.acceleration = (float)ParseClampedIntProperty(entity, "acceleration", kDefaultAcceleration, kAccelerationMin, kAccelerationMax);

        sBoostVolumesByBody[bodyID] = sBoostVolumes.size();
        sBoostVolumes.push_back(boostVolume);

        printf("[entities] registered func_boost entity=%d body=%u boost=%.1f accel=%.1f dir=(%.3f %.3f %.3f)\n",
               entityIndex,
               bodyID.GetIndexAndSequenceNumber(),
               boostVolume.boostAmount,
               boostVolume.acceleration,
               boostVolume.direction.x,
               boostVolume.direction.y,
               boostVolume.direction.z);
        return;
    }

    if (classname == "trigger_once" || classname == "trigger_multiple") {
        std::string target;
        if (!ParseStringProperty(entity, "target", target) || target.empty()) {
            printf("[entities] %s entity=%d is missing target and will not teleport the player\n",
                   classname.c_str(),
                   entityIndex);
            return;
        }

        const int spawnflags = ParseClampedIntProperty(entity, "spawnflags", 0, 0, 0x7fffffff);

        TeleportTrigger trigger;
        trigger.entityIndex = entityIndex;
        trigger.bodyID = bodyID;
        trigger.target = target;
        trigger.triggerOnce = classname == "trigger_once";
        trigger.enabled = (spawnflags & 1) == 0;
        trigger.wait = trigger.triggerOnce ? 0.0f : ParseFloatProperty(entity, "wait", 1.0f, 0.0f);

        sTeleportTriggersByBody[bodyID] = sTeleportTriggers.size();
        sTeleportTriggers.push_back(trigger);

        printf("[entities] registered %s entity=%d body=%u target='%s' enabled=%d wait=%.2f\n",
               classname.c_str(),
               entityIndex,
               bodyID.GetIndexAndSequenceNumber(),
               target.c_str(),
               trigger.enabled ? 1 : 0,
               trigger.wait);
    }
}

PlayerEffectResult ApplyPlayerEffects(JPH::PhysicsSystem* physicsSystem,
                                      const JPH::Shape* playerShape,
                                      Vector3 playerCenter,
                                      Vector3 groundNormal,
                                      bool isGrounded,
                                      float deltaTime,
                                      Vector3& inOutVelocity)
{
    PlayerEffectResult result;

    if (physicsSystem == nullptr || playerShape == nullptr || sBoostVolumes.empty()) {
        return result;
    }

    JPH::CollideShapeSettings settings;
    settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mMaxSeparationDistance = kBoostTouchDistance;

    JPH::ClosestHitPerBodyCollisionCollector<JPH::CollideShapeCollector> collector;
    physicsSystem->GetNarrowPhaseQuery().CollideShape(
        playerShape,
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(ToJoltRVec3(playerCenter)),
        settings,
        JPH::RVec3::sZero(),
        collector,
        physicsSystem->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        sBoostTouchLayerFilter
    );

    if (!collector.HadHit()) {
        sActivePlayerBoosts.clear();
        return result;
    }

    collector.Sort();
    std::unordered_set<int> appliedEntities;
    std::unordered_set<JPH::BodyID> touchedBoostBodies;
    const Vector3 entryVelocity = inOutVelocity;

    for (const JPH::CollideShapeResult& hit : collector.mHits) {
        auto boostIt = sBoostVolumesByBody.find(hit.mBodyID2);
        if (boostIt == sBoostVolumesByBody.end()) {
            continue;
        }

        const BoostVolume& boostVolume = sBoostVolumes[boostIt->second];
        touchedBoostBodies.insert(hit.mBodyID2);

        auto activeIt = sActivePlayerBoosts.find(hit.mBodyID2);
        if (activeIt == sActivePlayerBoosts.end()) {
            ActiveBoostState activeBoostState;
            activeBoostState.targetAlongDirection =
                Vector3DotProduct(entryVelocity, boostVolume.direction) + boostVolume.boostAmount;
            activeIt = sActivePlayerBoosts.emplace(hit.mBodyID2, activeBoostState).first;
        }

        if (!appliedEntities.insert(boostVolume.entityIndex).second) {
            continue;
        }

        if (!ApplyBoostVolume(boostVolume, activeIt->second, deltaTime, inOutVelocity)) {
            continue;
        }

        result.appliedBoost = true;
        if (isGrounded && Vector3DotProduct(boostVolume.direction, groundNormal) > 0.001f) {
            result.launchOffGround = true;
        }
    }

    for (auto it = sActivePlayerBoosts.begin(); it != sActivePlayerBoosts.end(); ) {
        if (touchedBoostBodies.find(it->first) == touchedBoostBodies.end()) {
            it = sActivePlayerBoosts.erase(it);
        } else {
            ++it;
        }
    }

    return result;
}

TriggerTeleportResult QueryPlayerTeleportTrigger(JPH::PhysicsSystem* physicsSystem,
                                                 const JPH::Shape* playerShape,
                                                 Vector3 playerCenter,
                                                 float deltaTime)
{
    TriggerTeleportResult result;
    sGameplayTime += deltaTime;

    if (physicsSystem == nullptr || playerShape == nullptr || sTeleportTriggers.empty()) {
        sActivePlayerTriggerContacts.clear();
        return result;
    }

    JPH::CollideShapeSettings settings;
    settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mMaxSeparationDistance = kTriggerTouchDistance;

    JPH::ClosestHitPerBodyCollisionCollector<JPH::CollideShapeCollector> collector;
    physicsSystem->GetNarrowPhaseQuery().CollideShape(
        playerShape,
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(ToJoltRVec3(playerCenter)),
        settings,
        JPH::RVec3::sZero(),
        collector,
        physicsSystem->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        sSensorTouchLayerFilter
    );

    if (!collector.HadHit()) {
        sActivePlayerTriggerContacts.clear();
        return result;
    }

    collector.Sort();
    std::unordered_set<JPH::BodyID> touchedTriggerBodies;

    for (const JPH::CollideShapeResult& hit : collector.mHits) {
        auto triggerIt = sTeleportTriggersByBody.find(hit.mBodyID2);
        if (triggerIt == sTeleportTriggersByBody.end()) {
            continue;
        }

        touchedTriggerBodies.insert(hit.mBodyID2);
        TeleportTrigger& trigger = sTeleportTriggers[triggerIt->second];

        if (result.teleportPlayer ||
            !trigger.enabled ||
            trigger.consumed ||
            sActivePlayerTriggerContacts.find(hit.mBodyID2) != sActivePlayerTriggerContacts.end() ||
            sGameplayTime < trigger.nextFireTime) {
            continue;
        }

        auto checkpointIt = sCheckPointsByTargetname.find(trigger.target);
        if (checkpointIt == sCheckPointsByTargetname.end()) {
            printf("[entities] trigger entity=%d target='%s' does not match any check_point targetname\n",
                   trigger.entityIndex,
                   trigger.target.c_str());
            continue;
        }

        const CheckPoint& checkPoint = sCheckPoints[checkpointIt->second];
        result.teleportPlayer = true;
        result.position = checkPoint.position;
        result.yaw = checkPoint.yaw;
        result.pitch = checkPoint.pitch;

        if (trigger.triggerOnce) {
            trigger.consumed = true;
        } else {
            trigger.nextFireTime = sGameplayTime + trigger.wait;
        }
    }

    sActivePlayerTriggerContacts = std::move(touchedTriggerBodies);
    return result;
}

} // namespace GameplayEntities
