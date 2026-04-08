#pragma once

#include "../math/wmath.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/PhysicsSystem.h"

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


void InitJoltCharacter(Player *player, JPH::PhysicsSystem *physicsSystem);

void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy);

void RespawnPlayer(Player *player, JPH::PhysicsSystem *physicsSystem, Vector3 position, float yaw, float pitch);

void UpdatePlayerMove(Player *player, JPH::PhysicsSystem *s_physics_system, float deltaTime);

void UpdatePlayer(Player *player, JPH::PhysicsSystem *s_physics_system, float deltaTime);

void UpdateCameraTarget(Player *player);

void DebugDrawPlayerAABB(Player *player);
void DebugDrawGroundProbe(void);
void DebugDir(Player *player);
void DebugDrawPlayerPos(const Player *player, int col, int row);
void DebugDrawPlayerVel(int col, int row);
