// src/player.h

#ifndef PLAYER_H
#define PLAYER_H

#include "raylib.h"

// Structure to represent the player (camera)
typedef struct Player {
    Camera3D camera;          // Raylib Camera3D structure
    float speed;              // Movement speed
    float rotationSpeed;      // Rotation speed (degrees per second)
    float yaw;                // Rotation around the Y-axis (left/right)
    float pitch;              // Rotation around the X-axis (up/down)
} Player;

// Initializes the player with default settings
void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy, int projection);

// Updates the player's camera based on input
void UpdatePlayer(Player *player, float deltaTime);

// Resets the camera orientation based on yaw and pitch
void UpdateCameraTarget(Player *player);

#endif // PLAYER_H
