// src/main.cpp

#include "raylib.h"
#include "raymath.h"
#include "player.h"
#include "parameters.h"

int main() { 
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Raylib Game Engine with TrenchBroom Integration");

    // Initialize player
    Player player;
    InitPlayer(&player, (Vector3){20.0f, 20.0f, 20.0f}, (Vector3){0.0f, 1.0f, 0.0f}, (Vector3){0.0f, 1.0f, 0.0f}, 90.0f, CAMERA_PERSPECTIVE);

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
                // Add your drawing code here
            EndMode3D();

            DrawFPS(10, 10);
        EndDrawing();
    }

    // De-Initialization
    CloseWindow();

    return 0;
}
