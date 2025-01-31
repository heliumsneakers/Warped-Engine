#pragma once

#include "raylib.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/PhysicsSystem.h"

extern float eyeOffset;

typedef struct Player {
    Camera3D camera;          
    float speed;              
    float rotationSpeed;      
    float yaw;                
    float pitch;
        
    //Bounding box for collisions: We get the center of the bbox and the half extents.
    Vector3 center;
    Vector3 halfExt;

} Player;


void InitJoltCharacter(Player *player, JPH::PhysicsSystem *physicsSystem);

void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy, int projection);

void PM_Friction();

void PM_Accelerate(Vector3 wishDir, float wishSpeed, float accel);

void PM_AirAccelerate(Vector3 wishDir, float wishSpeed, float accel);

void PM_AirMove();

void UpdatePlayer(Player *player,JPH::PhysicsSystem *s_physics_system ,float deltaTime);

void UpdateCameraTarget(Player *player);

void DebugDrawPlayerAABB(Player *player);


