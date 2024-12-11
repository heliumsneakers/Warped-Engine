#include "raylib.h"
#include "raymath.h"
#include "player.h"
#include "parameters.h"

#include <string>

int main() {
    // Initialize window
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Warped Engine");

    // Load model (including materials)
    const char* objPath = "../../assets/maps/test.obj";
    Model obj = LoadModel(objPath);

    // Set model position
    Vector3 modelPosition = { 0.0f, 0.0f, 0.0f };

    // Initialize player
    Player player;
    InitPlayer(&player, (Vector3){ 20.0f, 20.0f, 20.0f }, (Vector3){ 0.0f, 1.0f, 0.0f }, (Vector3){ 0.0f, 1.0f, 0.0f }, 90.0f, CAMERA_PERSPECTIVE);

    SetTargetFPS(60);

    // Main game loop
    while (!WindowShouldClose()) {
        // Calculate delta time
        float deltaTime = GetFrameTime();

        // Update player
        UpdatePlayer(&player, deltaTime);

        // Draw
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(player.camera);
                DrawGrid(100, 5.0f);
                DrawModel(obj, modelPosition, 0.2f, WHITE); // Draw the loaded model
            EndMode3D();

            DrawFPS(10, 10);
        EndDrawing();
    }

    // Unload model and deinitialize
    UnloadModel(obj);
    CloseWindow();

    return 0;
}
