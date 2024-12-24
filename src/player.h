// src/player.h

#ifndef PLAYER_H
#define PLAYER_H

#include "raylib.h"

typedef struct Player {
    Camera3D camera;          
    float speed;              
    float rotationSpeed;      
    float yaw;                
    float pitch;              
} Player;

void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy, int projection);

void UpdatePlayer(Player *player, float deltaTime);

void UpdateCameraTarget(Player *player);

#endif // PLAYER_H
