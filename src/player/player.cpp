// src/player.cpp

#include "player.h"
#include "raylib.h"
#include "raymath.h"
#include "utils/parameters.h"

// Jolt Specific
#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Geometry/ConvexSupport.h"
#include "Jolt/Physics/Collision/Shape/ConvexShape.h"

// NOTE: Thinking about writing the player collision here directly, could be too messy though
//       considering that all of the quake style movement code will go here as well. 
//       See to adding the collision code back into physics.cpp and just calling it here.

#define MOUSE_SENSITIVITY 0.5f
bool cursorEnabled;

float eyeOffset = 16.0f;

void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy, int projection) {
    player->camera.position = (Vector3){
        position.x,
        position.y + eyeOffset,
        position.z
    };
    player->camera.target = target;
    player->camera.up = up;
    player->camera.fovy = fovy;
    player->camera.projection = projection;

    player->speed = 200.0f;             // Units per second
    player->rotationSpeed = 90.0f;     // Degrees per second

    Vector3 direction = Vector3Normalize(Vector3Subtract(target, position));
    player->yaw = atan2f(direction.z, direction.x) * RAD2DEG;
    player->pitch = asinf(direction.y) * RAD2DEG;

    player->center = position;
    player->halfExt = (Vector3){16,28,16}; 

    SetMousePosition(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    DisableCursor();
    cursorEnabled = false;
}

void UpdatePlayer(Player *player, float deltaTime) {

    // ------ Mouse ------

    Vector2 mouseDelta = GetMouseDelta();

    player->yaw += mouseDelta.x * MOUSE_SENSITIVITY;
    player->pitch -= mouseDelta.y * MOUSE_SENSITIVITY;

    // Clamp the pitch to prevent flipping
    if (player->pitch > 89.0f) player->pitch = 89.0f;
    if (player->pitch < -89.0f) player->pitch = -89.0f;

    UpdateCameraTarget(player);  

    // ------ Keyboard ------
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
    if (IsKeyPressed(KEY_M) && cursorEnabled == false) {
        EnableCursor();
        cursorEnabled = true;
    } else if (IsKeyPressed(KEY_M) && cursorEnabled == true){
        DisableCursor();
        cursorEnabled = false;
    }
    
    // Apply movement
    player->camera.position = Vector3Add(player->camera.position, direction);
    player->camera.target = Vector3Add(player->camera.target, direction);
    player->center = (Vector3) {
        player->camera.position.x,
        player->camera.position.y - eyeOffset,
        player->camera.position.z
    };
}

void UpdateCameraTarget(Player *player) {
    float yawRad = player->yaw * DEG2RAD;
    float pitchRad = player->pitch * DEG2RAD;

    Vector3 direction;
    direction.x = cosf(pitchRad) * cosf(yawRad);
    direction.y = sinf(pitchRad);
    direction.z = cosf(pitchRad) * sinf(yawRad);

    direction = Vector3Normalize(direction);

    player->camera.target = Vector3Add(player->camera.position, direction);
}

void DebugDrawPlayerAABB(Player *player) {
     Vector3 minPt = {
        player->center.x - player->halfExt.x,
        player->center.y - player->halfExt.y,
        player->center.z - player->halfExt.z
    };
    Vector3 maxPt = {
        player->center.x + player->halfExt.x,
        player->center.y + player->halfExt.y,
        player->center.z + player->halfExt.z
    };

    DrawCubeWires(
        Vector3Scale(Vector3Add(minPt, maxPt), 0.5f), 
        (maxPt.x - minPt.x),
        (maxPt.y - minPt.y),
        (maxPt.z - minPt.z),
        RED
    );
} 
