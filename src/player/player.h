#pragma once

#include "../math/wmath.h"

extern float eyeOffset;

struct Camera {
    Vector3 position;
    Vector3 target;
    Vector3 up;
    float   fovy;
};

typedef struct Player {
    Camera camera;
    float  speed;
    float  rotationSpeed;
    float  yaw;
    float  pitch;

    // Bounding box for collisions
    Vector3 center;
    Vector3 halfExt;
} Player;


void InitPlayerPhysics(Player *player);

void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy);

void RespawnPlayer(Player *player, Vector3 position, float yaw, float pitch);

void UpdatePlayerMove(Player *player, float deltaTime);

void UpdatePlayer(Player *player, float deltaTime);

void UpdateCameraTarget(Player *player);

void DebugDrawPlayerAABB(Player *player);
void DebugDrawGroundProbe(void);
void DebugDir(Player *player);
void DebugDrawPlayerPos(const Player *player, int col, int row);
void DebugDrawPlayerVel(int col, int row);
