// src/player.cpp

#include "player.h"
#include "raylib.h"
#include "raymath.h"
#include "parameters.h"

// Sensitivity for mouse movement
#define MOUSE_SENSITIVITY 0.5f

// Initializes the player with default settings
void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy, int projection) {
    player->camera.position = position;
    player->camera.target = target;
    player->camera.up = up;
    player->camera.fovy = fovy;
    player->camera.projection = projection;

    player->speed = 200.0f;             // Units per second
    player->rotationSpeed = 90.0f;     // Degrees per second

    // Calculate initial yaw and pitch based on initial camera direction
    Vector3 direction = Vector3Normalize(Vector3Subtract(target, position));
    player->yaw = atan2f(direction.z, direction.x) * RAD2DEG;
    player->pitch = asinf(direction.y) * RAD2DEG;

    // Initialize mouse position to screen center
    SetMousePosition(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    // Hide the mouse cursor
    DisableCursor();
}

// Updates the player's camera based on input
void UpdatePlayer(Player *player, float deltaTime) {
    // ********** Mouse Look Implementation **********

    // Get the mouse movement deltas
    Vector2 mouseDelta = GetMouseDelta();

    // Update yaw and pitch based on mouse movement
    player->yaw += mouseDelta.x * MOUSE_SENSITIVITY;
    player->pitch -= mouseDelta.y * MOUSE_SENSITIVITY; // Inverted Y for natural feel

    // Clamp the pitch to prevent flipping
    if (player->pitch > 89.0f) player->pitch = 89.0f;
    if (player->pitch < -89.0f) player->pitch = -89.0f;

    // Update the camera's target based on the new yaw and pitch
    UpdateCameraTarget(player);

    // ********** Keyboard Movement Implementation **********

    Vector3 direction = {0.0f, 0.0f, 0.0f};
    Vector3 forward = Vector3Normalize(Vector3Subtract(player->camera.target, player->camera.position));
    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, player->camera.up));

    // Movement controls
    if (IsKeyDown(KEY_W)) {
        direction = Vector3Add(direction, Vector3Scale(forward, player->speed * deltaTime));
    }
    if (IsKeyDown(KEY_S)) {
        direction = Vector3Subtract(direction, Vector3Scale(forward, player->speed * deltaTime));
    }
    if (IsKeyDown(KEY_A)) {
        direction = Vector3Subtract(direction, Vector3Scale(right, player->speed * deltaTime));
    }
    if (IsKeyDown(KEY_D)) {
        direction = Vector3Add(direction, Vector3Scale(right, player->speed * deltaTime));
    }
    if (IsKeyDown(KEY_SPACE)) {
        direction = Vector3Add(direction, Vector3Scale(player->camera.up, player->speed * deltaTime));
    }
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        direction = Vector3Subtract(direction, Vector3Scale(player->camera.up, player->speed * deltaTime));
    }

    // Apply movement
    player->camera.position = Vector3Add(player->camera.position, direction);
    player->camera.target = Vector3Add(player->camera.target, direction);
}

// Updates the camera's target based on the current yaw and pitch
void UpdateCameraTarget(Player *player) {
    // Convert yaw and pitch from degrees to radians
    float yawRad = player->yaw * DEG2RAD;
    float pitchRad = player->pitch * DEG2RAD;

    // Calculate the new direction vector
    Vector3 direction;
    direction.x = cosf(pitchRad) * cosf(yawRad);
    direction.y = sinf(pitchRad);
    direction.z = cosf(pitchRad) * sinf(yawRad);

    direction = Vector3Normalize(direction);

    // Update the camera's target
    player->camera.target = Vector3Add(player->camera.position, direction);
}
