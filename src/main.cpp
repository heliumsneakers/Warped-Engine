#include "raylib.h"
#include "raymath.h"
#include "player.h"
#include "parameters.h"
#include <string>
#include "map_parser.h"

int main() {
    // Initialize window
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Warped Engine");
    printf("WINDOW INIT\n");

    // Load model (including materials)
    const char* objPath = "../../assets/maps/test.obj";
    Model obj = LoadModel(objPath);
    printf("MODEL LOADED\n");


    // Set model position
    Vector3 modelPosition = { 0.0f, 0.0f, 0.0f };

    // Initialize player
    Player player;
    InitPlayer(&player, (Vector3){ 20.0f, 20.0f, 20.0f }, (Vector3){ 0.0f, 1.0f, 0.0f }, (Vector3){ 0.0f, 1.0f, 0.0f }, 90.0f, CAMERA_PERSPECTIVE);

    SetTargetFPS(60);

    TextureManager textureManager;
    InitTextureManager(textureManager);
    printf("Texture Manager Initialized\n");

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

    // Convert map to mesh
    Mesh mapMesh = MapToMesh(map, textureManager);
    printf("Map Mesh: %d vertices, %d triangles\n", mapMesh.vertexCount, mapMesh.triangleCount);

    if (mapMesh.vertexCount == 0) {
        printf("Map Mesh is empty. Exiting.\n");
        UnloadAllTextures(textureManager);
        CloseWindow();
        return 1;
    }

    // Load the mesh into a Raylib Model
    Model mapModel = LoadModelFromMesh(mapMesh);
    printf("Map Model Loaded: %d materials\n", mapModel.materialCount);

    if (mapModel.materialCount == 0) {
        printf("Map Model has no materials. Exiting.\n");
        UnloadMesh(mapMesh);
        UnloadAllTextures(textureManager);
        CloseWindow();
        return 1;
    }

    // Assign textures to the model's materials
    // Since we have multiple textures, ensure each submesh has the correct material
    // For simplicity, assuming a single material here
    // If multiple materials are needed, consider creating multiple models
    if (!textureManager.textures.empty()) {
        // Assign the first loaded texture to the model's diffuse map
        Texture2D firstTexture = textureManager.textures.begin()->second;
        mapModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = firstTexture;
        printf("Assigned texture to Map Model.\n");
    }

    // Main game loop
    while (!WindowShouldClose()) {
        // Calculate delta time
        float deltaTime = GetFrameTime();

        // Update player
        UpdatePlayer(&player, deltaTime);

        // Draw
        BeginDrawing();
            ClearBackground(DARKGRAY);

            BeginMode3D(player.camera);

                DrawGrid(100, 5.0f);

                /*
                DrawModel(obj, modelPosition, 0.2f, WHITE); 
                DrawModelWires(obj, modelPosition, 0.2f, GREEN);
                */

                // Draw the model at the origin
                DrawModel(mapModel, (Vector3){0.0f, 0.0f, 0.0f}, 0.2f, WHITE);
                DrawModelWires(mapModel, (Vector3){0.0f,0.0f,0.0f}, 1.0f, RED);

            EndMode3D();

            DrawFPS(10, 10);
        EndDrawing();
    }

    // Unload model and deinitialize
    UnloadModel(obj);
    UnloadModel(mapModel);
    CloseWindow();

    return 0;
}
