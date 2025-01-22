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
   
    // Initialize player 
    Player player;
    InitPlayer(&player, (Vector3){20.0f, 20.0f, 20.0f}, (Vector3){0.0f, 1.0f, 0.0f}, (Vector3){0.0f, 1.0f, 0.0f}, 90.0f, CAMERA_PERSPECTIVE);

    // Parse the map
    Map map = ParseMapFile("../../assets/maps/test.map");
    printf("Parsed Map: %zu entities\n", map.entities.size());

    // Extract player start positions
    std::vector<PlayerStart> playerStarts = GetPlayerStarts(map);

    Vector3 playerPosition = {0.0f, 0.0f, 0.0f};
    if (!playerStarts.empty()) {
        playerPosition = playerStarts[0].position;
        printf("Player Start Position: (%f, %f, %f)\n", playerPosition.x, playerPosition.y, playerPosition.z);
    } else {
        printf("No player start positions found.\n");
    }

    if (!playerStarts.empty()) {
        player.camera.position = (Vector3){
            playerPosition.x,
            playerPosition.y + eyeOffset,
            playerPosition.z
        };
        printf("\n\nplayer cam pos y = %f\n\n", player.camera.position.y);
        player.camera.target = (Vector3){0.0f, 0.0f, 0.0f};
        player.camera.up = (Vector3){0.0f, 1.0f, 0.0f};
        printf("Player camera position set to player start position and target set to model.\n");
    }

    // Convert map to mesh and create model
    Model mapModel = MapToMesh(map, textureManager);
    printf("Map Model Loaded: %d materials\n", mapModel.materialCount);
    printf("Map mesh count: %d\n", mapModel.meshCount);


    // Initialize physics here and generate collision hulls.

    InitPhysicsSystem();

    JPH::BodyInterface *bodyInterface = &GetBodyInterface();

    printf("\n\n EXTRACTING COLLISION DATA");
    std::vector<MeshCollisionData> collisionData = ExtractCollisionData(mapModel);
    printf("\n\n COLLISION DATA EXTRACTED SUCCESFULLY");

    //printf("\n\n BUILDING MAP PHYSICS");
    BuildMapPhysics(collisionData, bodyInterface);

    // TODO: Assign the players physics and add functionality to UpdatePlayer() to update with the physics tick.

    float deltaTime = 1.0f/60.0f;

    SpawnDebugPhysObj(bodyInterface);
    
    //SpawnMinimalTest(*bodyInterface);

    // Main game loop
    while (!WindowShouldClose()) {
         
        UpdatePhysicsSystem(deltaTime, bodyInterface);

        UpdatePlayer(&player, deltaTime);

        UpdateCameraTarget(&player);

        // Begin drawing
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(player.camera);

                DrawGrid(100, 5.0f);

                DrawModel(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);

                if (bodyInterface->IsActive(debugSphereID)) {
                    JPH::RVec3 pos = bodyInterface->GetCenterOfMassPosition(debugSphereID);
                        Vector3 spherePos = {(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()};
                    DrawSphere(spherePos, 10.0f, RED);
                }

                DebugDrawPlayerAABB(&player);

                //DrawModelWires(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, GREEN);

            EndMode3D();

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
