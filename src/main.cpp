// main.cpp
#include "raylib.h"
#include "rlgl.h"
#include <stdio.h>
#include <vector>
#include "physx/collision_data.h"
#include "physx/physics.h"
#include "utils/map_parser.h"
#include "utils/parameters.h"
#include "player/player.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyInterface.h"



int main() {

    // Initialize window
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Warped Engine");
    SetTargetFPS(FPS_MAX);

    printf("WINDOW INIT\n");

    // Initialize TextureManager
    TextureManager textureManager;
    InitTextureManager(textureManager);

    // Culling Specification
    rlSetClipPlanes(0.01, 5000.0);
    printf("Near Cull dist: %f\n", rlGetCullDistanceNear());
    printf("Far Cull dist: %f \n", rlGetCullDistanceFar());

    Map map = ParseMapFile("../../assets/maps/test.map");

    std::vector<PlayerStart> starts = GetPlayerStarts(map);

    Vector3 spawnRL = starts.empty() ? (Vector3){0,0,0} : starts[0].position;

    // Initialize player with center = spawn
    Player player;
    InitPlayer(&player, spawnRL, (Vector3){0,0,0}, (Vector3){0,1,0}, 90.0f, CAMERA_PERSPECTIVE);

    printf("\n RL Spawn point:: x: %f y: %f z: %f \n\n", player.center.x, player.center.y, player.center.z);

    // Convert map to mesh and create model
    Model mapModel = MapToMesh(map, textureManager);
    printf("Map Model Loaded: %d materials\n", mapModel.materialCount);
    printf("Map mesh count: %d\n", mapModel.meshCount);


    // Initialize physics here and generate collision hulls.
    InitPhysicsSystem();

    JPH::BodyInterface *bodyInterface = &GetBodyInterface();

    printf("\n\n EXTRACTING COLLISION DATA");
    std::vector<MeshCollisionData> collisionData = ExtractCollisionData(map);
    printf("\n\n COLLISION DATA EXTRACTED SUCCESFULLY");

    //printf("\n\n BUILDING MAP PHYSICS");
    BuildMapPhysics(collisionData, bodyInterface);

    // TODO: Assign the players physics and add functionality to UpdatePlayer() to update with the physics tick.

    SpawnDebugPhysObj(bodyInterface);

    InitJoltCharacter(&player, s_physics_system);

    printf("\n\n RL Spawn point at Jolt INIT:: x: %f y: %f z: %f \n\n", player.center.x, player.center.y, player.center.z);

    // Main game loop
    while (!WindowShouldClose()) {

        UpdatePhysicsSystem(deltaTime, bodyInterface);

        if (DEVMODE) {
            UpdatePlayer(&player, s_physics_system, deltaTime);
        } else UpdatePlayerMove(&player, s_physics_system, deltaTime);

        UpdateCameraTarget(&player);

        // Begin drawing
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(player.camera);

        DrawModel(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);

        if (bodyInterface->IsActive(debugSphereID)) {
            JPH::RVec3 pos = bodyInterface->GetCenterOfMassPosition(debugSphereID);
            Vector3 spherePos = {(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()};
            DrawSphere(spherePos, 10.0f, RED);
        }

        DebugDrawPlayerAABB(&player);
        DebugDir(&player); 
        //DrawModelWires(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, GREEN);

        EndMode3D();

        DebugDrawPlayerPos(&player, 10, 30);
        DebugDrawPlayerVel(640, 600);
        DrawFPS(10, 10);
        EndDrawing();
    }

    // Unload resources
    ShutdownPhysicsSystem();
    UnloadModel(mapModel);
    UnloadAllTextures(textureManager);
    CloseWindow();
    return 0;
}
