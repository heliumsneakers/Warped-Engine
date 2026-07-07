#include "physics.h"
#include "collision_data.h"
#include "../entities/entities.h"

#include "box3d/box3d.h"

#include <algorithm>
#include <cstdio>
#include <thread>
#include <vector>

/* Here we will setup all Box3D physics functions for collisions with the world.
 *  Theoretical structure:
 *      - The worldspawn layer in TB will contain all of our brushes, brushes
 *        inside the worldspawn layer that arent entities (besides clip brushes and triggers), will be
 *        calculated as convex hulls for applying the minkowski difference and GJK algorithm
 *        for collision detection. This is heavily inspired by Quake's collision system.
 *
 *        NOTE: See "Real-Time Collision Detection by Christer Ericson" page 400, Ch. 9.5.
 *              This chapter reviews many convex collision detection algorithms, the section
 *              specified defines the Gilbert-Johnson-Keerthi algorithm (GJK).
 *
 *      - Box3D is a C API built around convex hulls, so the parsed brush point clouds feed
 *        directly into b3CreateHull. Triggers are static sensor shapes, the player never has
 *        a body and instead queries the world with casts and overlaps similar to Quake and GoldSrc.
 * */

//--------------------------------------//
// Global or static variables
//--------------------------------------//
b3WorldId g_physicsWorld = B3_NULL_ID;
b3BodyId  debugSphereID  = B3_NULL_ID;

//--------------------------------------//
// Implementation
//--------------------------------------//

void InitPhysicsSystem()
{
    if (b3World_IsValid(g_physicsWorld))
        return;

    // Length units are left at the default (1.0). The player controller keeps
    // POSITION_EPSILON (0.02) of separation from surfaces, which must stay
    // above Box3D's linear slope (0.005 * lengthUnits) or casts that start near
    // a surface report an initial overlap with a zero normal.
    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.gravity = b3Vec3{ 0.0f, -98.1f, 0.0f };

    const unsigned hw = std::thread::hardware_concurrency();
    worldDef.workerCount = std::clamp(hw > 1 ? hw - 1 : 1u, 1u, (unsigned)B3_MAX_WORKERS);

    g_physicsWorld = b3CreateWorld(&worldDef);

    printf("[InitPhysicsSystem] Box3D setup complete\n");
}

void ShutdownPhysicsSystem()
{
    if (!b3World_IsValid(g_physicsWorld))
        return;

    b3DestroyWorld(g_physicsWorld);
    g_physicsWorld = B3_NULL_ID;
    debugSphereID  = B3_NULL_ID;

    printf("[ShutdownPhysicsSystem] Freed Box3D resources.\n");
}

void UpdatePhysicsSystem(float delta_time)
{
    if (!b3World_IsValid(g_physicsWorld))
        return;

    const int subStepCount = 4;
    b3World_Step(g_physicsWorld, delta_time, subStepCount);

    // Trigger volumes are sensor shapes, dynamic bodies entering them show up here.
    b3SensorEvents sensorEvents = b3World_GetSensorEvents(g_physicsWorld);
    for (int i = 0; i < sensorEvents.beginCount; ++i) {
        printf("\n Trigger Activated\n");
    }

    b3BodyEvents bodyEvents = b3World_GetBodyEvents(g_physicsWorld);
    for (int i = 0; i < bodyEvents.moveCount; ++i) {
        if (bodyEvents.moveEvents[i].fellAsleep) {
            printf("A body went to sleep\n");
        }
    }
}

void SpawnDebugPhysObj()
{
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{ 0.0f, 1500.0f, -180.0f };

    debugSphereID = b3CreateBody(g_physicsWorld, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.filter.categoryBits = Layers::MOVING;
    shapeDef.filter.maskBits = Layers::STATIC | Layers::MOVING | Layers::SENSOR;
    shapeDef.enableSensorEvents = true;

    b3Sphere sphere = { b3Vec3_zero, 10.0f };
    b3CreateSphereShape(debugSphereID, &shapeDef, &sphere);

    printf("\n --TEST OBJECT SPAWNED-- \n");
}

void BuildMapPhysics(const std::vector<MeshCollisionData> &meshCollisionData,
                     const std::vector<Entity> &entities)
{
    int count = 0;
    GameplayEntities::Reset();
    GameplayEntities::RegisterPointEntities(entities);

    for (auto &mcd : meshCollisionData) {
        // If NO_COLLIDE or something similar, we skip
        if (mcd.collisionType == CollisionType::NO_COLLIDE) {
            continue;
        }

        // Convert engine vectors to Box3D points
        std::vector<b3Vec3> points;
        points.reserve(mcd.vertices.size());
        for (auto &v : mcd.vertices) {
            points.push_back(b3Vec3{ v.x, v.y, v.z });
        }

        if (points.size() < 4) {
            printf("Skipping brush with too few points for a hull (%zu)\n", points.size());
            continue;
        }

        // Build a convex hull from these points
        b3HullData *hull = b3CreateHull(points.data(), (int)points.size(), (int)points.size());
        if (hull == nullptr) {
            printf("Error building hull shape from %zu points\n", points.size());
            continue;
        }

        // Decide body type and filter bits from collisionType
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3ShapeDef shapeDef = b3DefaultShapeDef();

        switch (mcd.collisionType) {
            case CollisionType::STATIC: {
                bodyDef.type = b3_staticBody;
                shapeDef.filter.categoryBits = Layers::STATIC;
                shapeDef.filter.maskBits = Layers::MOVING;
                break;
            }
            case CollisionType::TRIGGER: {
                // Static sensor volume: generates overlap events but no collision response.
                bodyDef.type = b3_staticBody;
                shapeDef.filter.categoryBits = Layers::SENSOR;
                shapeDef.filter.maskBits = Layers::MOVING;
                shapeDef.isSensor = true;
                shapeDef.enableSensorEvents = true;
                printf("\n SETTING IS SENSOR TO TRUE \n");
                break;
            }
            case CollisionType::DYNAMIC: {
                bodyDef.type = b3_dynamicBody;
                shapeDef.filter.categoryBits = Layers::MOVING;
                shapeDef.filter.maskBits = Layers::STATIC | Layers::MOVING | Layers::SENSOR;
                shapeDef.enableSensorEvents = true;
                break;
            }
            // NO_COLLIDE or UNKNOWN => skip
            default: {
                b3DestroyHull(hull);
                continue;
            }
        }

        // Brush geometry is already in world space, so the body sits at the origin.
        b3BodyId body = b3CreateBody(g_physicsWorld, &bodyDef);
        if (B3_IS_NULL(body)) {
            printf("Failed to create body for a brush\n");
            b3DestroyHull(hull);
            continue;
        }

        b3ShapeId shape = b3CreateHullShape(body, &shapeDef, hull);
        b3DestroyHull(hull); // the world interns hull data in its hull database

        if (B3_IS_NULL(shape)) {
            printf("Failed to create hull shape for a brush\n");
            b3DestroyBody(body);
            continue;
        }

        if (mcd.entityIndex >= 0 && (size_t)mcd.entityIndex < entities.size()) {
            GameplayEntities::RegisterBrushEntity(entities[(size_t)mcd.entityIndex], mcd.entityIndex, body);
        }
        ++count;
    }

    printf("\n\n %d MAP COLLISIONS SUCCESSFULLY CREATED \n\n", count);
}
