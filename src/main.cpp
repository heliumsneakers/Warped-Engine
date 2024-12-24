// main.cpp
#include "raylib.h"
#include "rlgl.h"
#include "map_parser.h"
#include "parameters.h"
#include "player.h"
#include <stdio.h>

int main() {
    // Initialize window
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Warped Engine");
    SetTargetFPS(144);

    printf("WINDOW INIT\n");

    // Initialize TextureManager
    TextureManager textureManager;
    InitTextureManager(textureManager);
   
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
        player.camera.position = playerPosition;
        player.camera.target = (Vector3){0.0f, 0.0f, 0.0f};
        player.camera.up = (Vector3){0.0f, 1.0f, 0.0f};
        printf("Player camera position set to player start position and target set to model.\n");
    }

    // Convert map to mesh and create model
    Model mapModel = MapToMesh(map, textureManager);
    printf("Map Model Loaded: %d materials\n", mapModel.materialCount);
    printf("Map mesh count: %d\n", mapModel.meshCount);

    // Debug against obj
    Model obj = LoadModel("../../assets/maps/test.obj");
    printf("OBJ MESH COUNT: %d\n", obj.meshCount);
 
    // Main game loop
    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();

        UpdatePlayer(&player, deltaTime);

        UpdateCameraTarget(&player);

        // Begin drawing
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(player.camera);

                DrawGrid(100, 5.0f);

                DrawModel(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);

                DrawModelWires(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, RED);

            EndMode3D();

            DrawFPS(10, 10);
        EndDrawing();
    }

    // Unload resources
    UnloadModel(mapModel);
    UnloadModel(obj);
    UnloadAllTextures(textureManager);
    CloseWindow();
    return 0;
}
