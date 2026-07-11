#include "entities.h"
#include "entity_defs.h"

#include "../compiler/map_geometry.h"
#include "../compiler/map_parser.h"
#include "../physx/physics.h"

#include "box3d/box3d.h"

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

        // Bodies are tracked by their packed 64bit id so they can key hash containers.
        using BodyKey = uint64_t;

        struct BoostVolume {
            int entityIndex = -1;
            BodyKey bodyKey = 0;
            Vector3 direction = { 0.0f, 1.0f, 0.0f };
            float boostAmount = (float)kDefaultBoostAmount;
            float acceleration = (float)kDefaultAcceleration;
        };

        struct ActiveBoostState {
            float targetAlongDirection = 0.0f;
        };

        struct BoostBodyPair {
            BodyKey boostBody = 0;
            BodyKey visitorBody = 0;

            bool operator==(const BoostBodyPair& other) const
            {
                return boostBody == other.boostBody && visitorBody == other.visitorBody;
            }
        };

        struct BoostBodyPairHash {
            size_t operator()(const BoostBodyPair& pair) const
            {
                uint64_t x = pair.boostBody;
                uint64_t y = pair.visitorBody;
                x ^= y + 0x9e3779b97f4a7c15ull + (x << 6u) + (x >> 2u);
                return (size_t)x;
            }
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
            BodyKey bodyKey = 0;
            std::string target;
            bool triggerOnce = false;
            bool enabled = true;
            bool consumed = false;
            float wait = 0.0f;
            double nextFireTime = 0.0;
        };

        std::vector<BoostVolume> sBoostVolumes;
        std::unordered_map<BodyKey, size_t> sBoostVolumesByBody;
        std::unordered_map<BodyKey, ActiveBoostState> sActivePlayerBoosts;
        std::unordered_map<BoostBodyPair, ActiveBoostState, BoostBodyPairHash> sActiveBodyBoosts;
        std::vector<CheckPoint> sCheckPoints;
        std::unordered_map<std::string, size_t> sCheckPointsByTargetname;
        std::vector<TeleportTrigger> sTeleportTriggers;
        std::unordered_map<BodyKey, size_t> sTeleportTriggersByBody;
        std::unordered_set<BodyKey> sActivePlayerTriggerContacts;
        double sGameplayTime = 0.0;

        struct SensorOverlapContext {
            std::vector<BodyKey> touchedBodies;
        };

        static bool CollectSensorOverlaps(b3ShapeId shapeId, void* context)
        {
            auto* overlapContext = (SensorOverlapContext*)context;
            overlapContext->touchedBodies.push_back(b3StoreBodyId(b3Shape_GetBody(shapeId)));
            return true;
        }

        // Overlap the player shape (inflated by touchDistance via the proxy radius)
        // against sensor shapes and return the packed body ids of every touched sensor.
        static std::vector<BodyKey> QueryTouchedSensors(const b3ShapeProxy* playerProxy,
                                                        Vector3 playerCenter,
                                                        float touchDistance)
        {
            b3ShapeProxy proxy = { playerProxy->points, playerProxy->count, playerProxy->radius + touchDistance };

            b3QueryFilter filter = b3DefaultQueryFilter();
            filter.categoryBits = Layers::MOVING;
            filter.maskBits = Layers::SENSOR;

            SensorOverlapContext context;
            b3World_OverlapShape(g_physicsWorld,
                                 b3Pos{ playerCenter.x, playerCenter.y, playerCenter.z },
                                 &proxy,
                                 filter,
                                 CollectSensorOverlaps,
                                 &context);
            return context.touchedBodies;
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
        sActiveBodyBoosts.clear();
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

            const EntityDefs::EntityDef* def = EntityDefs::Find(entity);
            if (def == nullptr || def->behavior != EntityDefs::RuntimeBehavior::CheckPoint) {
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

    void RegisterBrushEntity(const Entity& entity, int entityIndex, b3BodyId bodyId)
    {
        const EntityDefs::EntityDef* def = EntityDefs::Find(entity);
        if (def == nullptr) {
            return;
        }

        const char* classname = EntityDefs::Classname(entity);
        const BodyKey bodyKey = b3StoreBodyId(bodyId);

        if (def->behavior == EntityDefs::RuntimeBehavior::BoostVolume) {
            BoostVolume boostVolume;
            boostVolume.entityIndex = entityIndex;
            boostVolume.bodyKey = bodyKey;
            boostVolume.direction = ParseBoostDirection(entity);
            boostVolume.boostAmount = (float)ParseClampedIntProperty(entity, "boost", kDefaultBoostAmount, kBoostAmountMin, kBoostAmountMax);
            boostVolume.acceleration = (float)ParseClampedIntProperty(entity, "acceleration", kDefaultAcceleration, kAccelerationMin, kAccelerationMax);

            sBoostVolumesByBody[bodyKey] = sBoostVolumes.size();
            sBoostVolumes.push_back(boostVolume);

            printf("[entities] registered %s entity=%d body=%llu boost=%.1f accel=%.1f dir=(%.3f %.3f %.3f)\n",
                   classname,
                   entityIndex,
                   (unsigned long long)bodyKey,
                   boostVolume.boostAmount,
                   boostVolume.acceleration,
                   boostVolume.direction.x,
                   boostVolume.direction.y,
                   boostVolume.direction.z);
            return;
        }

        if (def->behavior == EntityDefs::RuntimeBehavior::TeleportTrigger) {
            std::string target;
            if (!ParseStringProperty(entity, "target", target) || target.empty()) {
                printf("[entities] %s entity=%d is missing target and will not teleport the player\n",
                       classname,
                       entityIndex);
                return;
            }

            const int spawnflags = ParseClampedIntProperty(entity, "spawnflags", 0, 0, 0x7fffffff);

            TeleportTrigger trigger;
            trigger.entityIndex = entityIndex;
            trigger.bodyKey = bodyKey;
            trigger.target = target;
            trigger.triggerOnce = std::string(classname) == "trigger_once";
            trigger.enabled = (spawnflags & 1) == 0;
            trigger.wait = trigger.triggerOnce ? 0.0f : ParseFloatProperty(entity, "wait", 1.0f, 0.0f);

            sTeleportTriggersByBody[bodyKey] = sTeleportTriggers.size();
            sTeleportTriggers.push_back(trigger);

            printf("[entities] registered %s entity=%d body=%llu target='%s' enabled=%d wait=%.2f\n",
                   classname,
                   entityIndex,
                   (unsigned long long)bodyKey,
                   target.c_str(),
                   trigger.enabled ? 1 : 0,
                   trigger.wait);
        }
    }

    PlayerEffectResult ApplyPlayerEffects(const b3ShapeProxy* playerProxy,
                                          Vector3 playerCenter,
                                          Vector3 groundNormal,
                                          bool isGrounded,
                                          float deltaTime,
                                          Vector3& inOutVelocity)
    {
        PlayerEffectResult result;

        if (!b3World_IsValid(g_physicsWorld) || playerProxy == nullptr || sBoostVolumes.empty()) {
            return result;
        }

        const std::vector<BodyKey> touched = QueryTouchedSensors(playerProxy, playerCenter, kBoostTouchDistance);

        if (touched.empty()) {
            sActivePlayerBoosts.clear();
            return result;
        }

        std::unordered_set<int> appliedEntities;
        std::unordered_set<BodyKey> touchedBoostBodies;
        const Vector3 entryVelocity = inOutVelocity;

        for (BodyKey bodyKey : touched) {
            auto boostIt = sBoostVolumesByBody.find(bodyKey);
            if (boostIt == sBoostVolumesByBody.end()) {
                continue;
            }

            const BoostVolume& boostVolume = sBoostVolumes[boostIt->second];
            touchedBoostBodies.insert(bodyKey);

            auto activeIt = sActivePlayerBoosts.find(bodyKey);
            if (activeIt == sActivePlayerBoosts.end()) {
                ActiveBoostState activeBoostState;
                activeBoostState.targetAlongDirection =
                    Vector3DotProduct(entryVelocity, boostVolume.direction) + boostVolume.boostAmount;
                activeIt = sActivePlayerBoosts.emplace(bodyKey, activeBoostState).first;
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

    void UpdateDynamicBodyEffects(float deltaTime)
    {
        if (!b3World_IsValid(g_physicsWorld) || sBoostVolumes.empty()) {
            sActiveBodyBoosts.clear();
            return;
        }

        std::unordered_set<BoostBodyPair, BoostBodyPairHash> touchedPairs;

        for (const BoostVolume& boostVolume : sBoostVolumes) {
            b3BodyId boostBody = b3LoadBodyId(boostVolume.bodyKey);
            if (!b3Body_IsValid(boostBody)) {
                continue;
            }

            const int shapeCount = b3Body_GetShapeCount(boostBody);
            if (shapeCount <= 0) {
                continue;
            }

            std::vector<b3ShapeId> sensorShapes((size_t)shapeCount);
            const int storedShapeCount = b3Body_GetShapes(boostBody, sensorShapes.data(), shapeCount);
            for (int shapeIndex = 0; shapeIndex < storedShapeCount; ++shapeIndex) {
                const b3ShapeId sensorShape = sensorShapes[(size_t)shapeIndex];
                if (!b3Shape_IsValid(sensorShape)) {
                    continue;
                }

                const int visitorCapacity = b3Shape_GetSensorCapacity(sensorShape);
                if (visitorCapacity <= 0) {
                    continue;
                }

                std::vector<b3ShapeId> visitorShapes((size_t)visitorCapacity);
                const int visitorCount = b3Shape_GetSensorData(sensorShape, visitorShapes.data(), visitorCapacity);
                for (int visitorIndex = 0; visitorIndex < visitorCount; ++visitorIndex) {
                    const b3ShapeId visitorShape = visitorShapes[(size_t)visitorIndex];
                    if (!b3Shape_IsValid(visitorShape)) {
                        continue;
                    }

                    const b3BodyId visitorBody = b3Shape_GetBody(visitorShape);
                    if (!b3Body_IsValid(visitorBody) || b3Body_GetType(visitorBody) != b3_dynamicBody) {
                        continue;
                    }

                    const BodyKey visitorBodyKey = b3StoreBodyId(visitorBody);
                    if (visitorBodyKey == boostVolume.bodyKey) {
                        continue;
                    }

                    const BoostBodyPair pair{ boostVolume.bodyKey, visitorBodyKey };
                    touchedPairs.insert(pair);

                    b3Vec3 linearVelocityB3 = b3Body_GetLinearVelocity(visitorBody);
                    Vector3 linearVelocity = { linearVelocityB3.x, linearVelocityB3.y, linearVelocityB3.z };

                    auto activeIt = sActiveBodyBoosts.find(pair);
                    if (activeIt == sActiveBodyBoosts.end()) {
                        ActiveBoostState activeBoostState;
                        activeBoostState.targetAlongDirection =
                            Vector3DotProduct(linearVelocity, boostVolume.direction) + boostVolume.boostAmount;
                        activeIt = sActiveBodyBoosts.emplace(pair, activeBoostState).first;
                    }

                    if (!ApplyBoostVolume(boostVolume, activeIt->second, deltaTime, linearVelocity)) {
                        continue;
                    }

                    b3Body_SetLinearVelocity(visitorBody, b3Vec3{ linearVelocity.x, linearVelocity.y, linearVelocity.z });
                }
            }
        }

        for (auto it = sActiveBodyBoosts.begin(); it != sActiveBodyBoosts.end(); ) {
            if (touchedPairs.find(it->first) == touchedPairs.end()) {
                it = sActiveBodyBoosts.erase(it);
            } else {
                ++it;
            }
        }
    }

    TriggerTeleportResult QueryPlayerTeleportTrigger(const b3ShapeProxy* playerProxy,
                                                     Vector3 playerCenter,
                                                     float deltaTime)
    {
        TriggerTeleportResult result;
        sGameplayTime += deltaTime;

        if (!b3World_IsValid(g_physicsWorld) || playerProxy == nullptr || sTeleportTriggers.empty()) {
            sActivePlayerTriggerContacts.clear();
            return result;
        }

        const std::vector<BodyKey> touched = QueryTouchedSensors(playerProxy, playerCenter, kTriggerTouchDistance);

        if (touched.empty()) {
            sActivePlayerTriggerContacts.clear();
            return result;
        }

        std::unordered_set<BodyKey> touchedTriggerBodies;

        for (BodyKey bodyKey : touched) {
            auto triggerIt = sTeleportTriggersByBody.find(bodyKey);
            if (triggerIt == sTeleportTriggersByBody.end()) {
                continue;
            }

            touchedTriggerBodies.insert(bodyKey);
            TeleportTrigger& trigger = sTeleportTriggers[triggerIt->second];

            if (result.teleportPlayer ||
                !trigger.enabled ||
                trigger.consumed ||
                sActivePlayerTriggerContacts.find(bodyKey) != sActivePlayerTriggerContacts.end() ||
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
