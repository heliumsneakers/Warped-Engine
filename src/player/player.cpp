// src/player.cpp

#include "player.h"
#include "Jolt/Math/Math.h"
#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "raylib.h"
#include "raymath.h"
#include "../utils/parameters.h"
#include "../physx/physics.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Math/MathTypes.h"
#include "Jolt/Math/Real.h"
#include "Jolt/Math/Vec3.h"
#include "Jolt/Physics/Body/BodyID.h"
#include "Jolt/Physics/Body/BodyInterface.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#define JPH_ENABLE_ASSERTS

/*  NOTE: Thinking about writing the player collision here directly, could be too messy though
*         considering that all of the quake style movement code will go here as well. 
*         See to adding the collision code back into physics.cpp and just calling it here.
*
*   NOTE: Upon further reading into the Jolt library I believe the best way forward will be to do an implementation of the Jolt 
*         CharacterVirtual here directly, and adapt my original Quake inspired movement code. Input handling will be done by raylib,
*         maybe all of the actual movement calculations can be done using raylibs Vector3 and at the end where we finally update the 
*         player position we do a conversion into Jolt's Vec3? not sure yet, will use this as the first attempt.
*/

#define MOUSE_SENSITIVITY 0.5f
bool cursorEnabled;

float eyeOffset = 16.0f;

// Movement constants for Quake PM impl.
const float MAX_SPEED           = 640.0f;   // Maximum speed in m/s (adjusted from 250.0f)
const float ACCELERATION        = 320.0f;   // Acceleration in m/s² (adjusted from 100.0f)
const float FRICTION            = 4.0f;     // Friction coefficient
const float STOP_SPEED          = 1.0f;     // Speed below which the player stops
const float JUMP_FORCE          = 100.0f;    // Jump force
const float AIR_ACCELERATION    = 1000.0f;   // Air acceleration in m/s^2
const float MAX_AIR_SPEED       = 1000.0f;   // Maximum speed while in air
const float GRAVITY             = -98.1f;   

Vector3 velocity = Vector3Zero();
float playerYaw = 0.0f;
float playerPitch = 0.0f;
bool isGrounded = false;

Vector3 wishDir     = Vector3Zero();
float   wishSpeed   = 0.0f;

// --------- BEGIN Jolt Character Collision Impl. ---------

static JPH::CharacterVirtual *gCharacter = nullptr;

static JPH::Vec3 gQuakeVelocity = JPH::Vec3::sZero();
static JPH::Vec3 gGroundNormal = JPH::Vec3::sZero();
JPH::CharacterVirtual::EGroundState gGroundState;


class MyCharacterLayerFilter : public JPH::ObjectLayerFilter
{
public:
    // Here we decide which layers the character "sees" during its shape cast
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override
    {
        // We want collisions with NON_MOVING, MOVING, and also SENSOR
        if (inLayer == Layers::NON_MOVING ||
            inLayer == Layers::MOVING     ||
            inLayer == Layers::SENSOR)    
        {
            return true;
        }
        return false;
    }
};

class MyCharacterListener : public JPH::CharacterContactListener
{
public:
    virtual void OnContactAdded(const JPH::CharacterVirtual *inCharacter,
                                const JPH::BodyID &inBodyID2,
                                const JPH::SubShapeID &inSubShapeID2,
                                JPH::RVec3Arg inContactPosition,
                                JPH::Vec3Arg inContactNormal,
                                JPH::CharacterContactSettings &ioSettings) override
    {
        HandleContact(inCharacter, inBodyID2, inSubShapeID2, inContactPosition, inContactNormal, ioSettings, true);
    }

    virtual void OnContactPersisted(const JPH::CharacterVirtual *inCharacter,
                                    const JPH::BodyID &inBodyID2,
                                    const JPH::SubShapeID &inSubShapeID2,
                                    JPH::RVec3Arg inContactPosition,
                                    JPH::Vec3Arg inContactNormal,
                                    JPH::CharacterContactSettings &ioSettings) override
    {
        HandleContact(inCharacter, inBodyID2, inSubShapeID2, inContactPosition, inContactNormal, ioSettings, false);
    }

    virtual void OnContactRemoved(const JPH::CharacterVirtual *inCharacter,
                                  const JPH::BodyID &inBodyID2,
                                  const JPH::SubShapeID &inSubShapeID2) override
    {
        JPH::BodyLockRead lock(s_physics_system->GetBodyLockInterfaceNoLock(), inBodyID2);
        if (!lock.Succeeded())
            return;
        
        const JPH::Body &body = lock.GetBody();

        CollisionType ct = (CollisionType) body.GetUserData();
        if (body.GetObjectLayer() == Layers::NON_MOVING && ct == CollisionType::STATIC) {
            isGrounded = false; 
        }
    }

private:
    void HandleContact(const JPH::CharacterVirtual *inCharacter,
                       const JPH::BodyID &inBodyID2,
                       const JPH::SubShapeID &inSubShapeID2,
                       JPH::RVec3Arg inContactPosition,
                       JPH::Vec3Arg inContactNormal,
                       JPH::CharacterContactSettings &ioSettings,
                       bool inIsNewContact)
    {
        // Attempt to read the body
        JPH::BodyLockRead lock(s_physics_system->GetBodyLockInterfaceNoLock(), inBodyID2);
        if (!lock.Succeeded())
            return;
        
        const JPH::Body &body = lock.GetBody();

        CollisionType ct = (CollisionType) body.GetUserData(); // Potentially we store type in user data? use a int pointer?
        if (body.GetObjectLayer() == Layers::SENSOR || ct == CollisionType::TRIGGER) {
            // We do NOT want the trigger to block the character
            // So set these flags:
            ioSettings.mCanPushCharacter = false;      // The trigger won't push us
            ioSettings.mCanReceiveImpulses = false;    // We won't push it

            // If we only want to do trigger logic on first contact:
            if (inIsNewContact) {
                printf("[CharacterListener] Overlapped TRIGGER, contact added!\n");
                //handle once-only activation
            }
            else {
                // Repeated each frame
                // printf("[CharacterListener] Trigger still active.\n");
            }
        }

        if (body.GetObjectLayer() == Layers::MOVING) {
            ioSettings.mCanPushCharacter = false;
        }
        
        else {
            // This is normal geometry
            // Possibly do normal character stuff
            if (body.GetObjectLayer() == Layers::NON_MOVING) {
                float norm = gCharacter->GetGroundNormal().GetY();
                if (norm < 0.7f) {
                    isGrounded = false;
                    gGroundNormal = inContactNormal;
                } else {
                    isGrounded = true;
                    gGroundNormal = inContactNormal;
                }

            } 
        }
    }
};

static MyCharacterListener s_char_listener;
static MyCharacterLayerFilter s_char_layer_filter;

// --------- END Jolt Character Collision Impl. ---------

void InitJoltCharacter(Player *player, JPH::PhysicsSystem *physicsSystem) {

    // Try this to debug slope bugs: JPH::RefConst<JPH::Shape> playerShape = new JPH::CapsuleShape(16.0f, 28.0f, 16.0f);

    JPH::RefConst<JPH::Shape> playerShape = new JPH::BoxShape(JPH::Vec3(16.0f, 28.0f, 16.0f));

    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
    settings->mShape = playerShape;
    settings->mMaxSlopeAngle = JPH::DegreesToRadians(45.0f);
    settings->mMaxStrength   = 150.0f;
    settings->mCharacterPadding = 0.02f;
    settings->mPenetrationRecoverySpeed = 1.0f;
    settings->mPredictiveContactDistance = 0.1f;
    settings->mInnerBodyLayer = Layers::MOVING;

    gCharacter = new JPH::CharacterVirtual(
        settings,
        JPH::RVec3(player->camera.position.x, 
                   player->camera.position.y + eyeOffset,
                   player->camera.position.z),
        JPH::Quat::sIdentity(),
        /* inUserData = */ 0,
        physicsSystem
    );
    
    gCharacter->SetListener(&s_char_listener);
    gQuakeVelocity = JPH::Vec3::sZero();

    printf("\n\n JOLT CHARACTER VIRTUAL INITIALIZED\n\n");
} 

void InitPlayer(Player *player, Vector3 position, Vector3 target, Vector3 up, float fovy, int projection) {
    player->camera.position = (Vector3){
        position.x,
        position.y - eyeOffset,
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

// --------- BEGIN Quake Movement Impl. ---------

// Apply friction when grounded
void PM_Friction(float deltaTime) {
    if (isGrounded) {
        Vector3 vel = velocity;
        vel.y = 0; // Ignore vertical component

        float speed = Vector3Length(vel);
        if (speed < 0.1f) {
            velocity.x = 0.0f;
            velocity.z = 0.0f;
            return;
        }

        float control = (speed < STOP_SPEED) ? STOP_SPEED : speed;
        float drop = control * FRICTION * deltaTime;

        float newSpeed = speed - drop;
        if (newSpeed < 0)
            newSpeed = 0;

        newSpeed /= speed;

        // Scale the velocity
        velocity.x *= newSpeed;
        velocity.z *= newSpeed;
    }
}

// Modified PM_Accelerate: Projects wishDir onto the slope plane if needed.
static void PM_Accelerate(Vector3 wishDir, float wishSpeed, float accel, float dt)
{
    // If we're not fully grounded and we have a valid ground normal,
    // project the wishDir onto the plane defined by gGroundNormal.
    if (!isGrounded && gGroundNormal.Length() > 0.001f)
    {
        Vector3 n = { gGroundNormal.GetX(), gGroundNormal.GetY(), gGroundNormal.GetZ() };
        float dot = Vector3DotProduct(wishDir, n);
        Vector3 projected = Vector3Subtract(wishDir, Vector3Scale(n, dot));
        if (Vector3Length(projected) > 0.001f)
            wishDir = Vector3Normalize(projected);
    }

    float currentSpeed = Vector3DotProduct(velocity, wishDir);
    float addSpeed = wishSpeed - currentSpeed;
    if (addSpeed <= 0)
        return;

    float accelSpeed = accel * dt;
    if (accelSpeed > addSpeed)
        accelSpeed = addSpeed;

    // Add only along wishDir, preserving any velocity components that already exist.
    velocity = Vector3Add(velocity, Vector3Scale(wishDir, accelSpeed));
}

// Modified PM_AirAccelerate: Same projection logic.
static void PM_AirAccelerate(Vector3 wishDir, float wishSpeed, float accel, float dt)
{
    if (wishSpeed > MAX_AIR_SPEED)
        wishSpeed = MAX_AIR_SPEED;

    if (!isGrounded && gGroundNormal.Length() > 0.001f)
    {
        Vector3 n = { gGroundNormal.GetX(), gGroundNormal.GetY(), gGroundNormal.GetZ() };
        float dot = Vector3DotProduct(wishDir, n);
        Vector3 projected = Vector3Subtract(wishDir, Vector3Scale(n, dot));
        if (Vector3Length(projected) > 0.001f)
            wishDir = Vector3Normalize(projected);
    }

    float currentSpeed = Vector3DotProduct(velocity, wishDir);
    float addSpeed = wishSpeed - currentSpeed;
    if (addSpeed <= 0)
        return;

    float accelSpeed = accel * dt;
    if (accelSpeed > addSpeed)
        accelSpeed = addSpeed;

    velocity = Vector3Add(velocity, Vector3Scale(wishDir, accelSpeed));
}

// PM_AirMove: Computes the wish direction from input.
static void PM_AirMove(float dt)
{
    float fmove = 0.0f;
    float smove = 0.0f;
    if (IsKeyDown(KEY_W)) fmove += 1.0f;
    if (IsKeyDown(KEY_S)) fmove -= 1.0f;
    if (IsKeyDown(KEY_D)) smove += 1.0f;
    if (IsKeyDown(KEY_A)) smove -= 1.0f;

    // Classic Quake uses yaw only (movement is in the horizontal plane)
    float yawRad = DEG2RAD * playerYaw;
    Vector3 forward = { cosf(yawRad), 0.0f, sinf(yawRad) };
    Vector3 right   = { cosf(yawRad + PI * 0.5f), 0.0f, sinf(yawRad + PI * 0.5f) };

    forward = Vector3Normalize(forward);
    right = Vector3Normalize(right);

    Vector3 inputVel = Vector3Add(Vector3Scale(forward, fmove), Vector3Scale(right, smove));
    wishSpeed = Vector3Length(inputVel);
    if (wishSpeed > 0.0001f)
        wishDir = Vector3Scale(inputVel, 1.0f / wishSpeed);
    else
        wishDir = (Vector3){0, 0, 0};

    // Clamp wishSpeed if necessary.
    if (wishSpeed > MAX_SPEED)
    {
        inputVel = Vector3Scale(inputVel, MAX_SPEED / wishSpeed);
        wishSpeed = MAX_SPEED;
    }

    // Use ground or air acceleration as appropriate.
    if (isGrounded)
        PM_Accelerate(wishDir, wishSpeed * 50, ACCELERATION, dt);
    else
        PM_AirAccelerate(wishDir, wishSpeed * 5, AIR_ACCELERATION, dt);
}

static void PM_ApplyGravity(float deltaTime) {
    if (!isGrounded) {
        velocity.y += GRAVITY *deltaTime;
    }
}

// --------- END Quake Movement Impl. ---------

void UpdatePlayerMove(Player *player, JPH::PhysicsSystem *mPhysicsSystem, float deltaTime) {
    // 1) Read current velocity from Jolt.
    {
        JPH::Vec3 curVel = gCharacter->GetLinearVelocity();
        velocity.x = curVel.GetX();
        velocity.y = curVel.GetY();
        velocity.z = curVel.GetZ();
    }

    // 2) Run ExtendedUpdate first to let the world (collisions, slopes, etc.) update.
    JPH::CharacterVirtual::ExtendedUpdateSettings updateSettings;
    gCharacter->ExtendedUpdate(
        deltaTime,
        JPH::Vec3::sZero(),
        updateSettings,
        mPhysicsSystem->JPH::PhysicsSystem::GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        s_char_layer_filter,
        {},
        {},
        *s_temp_allocator
    );

    // 3) Read back the world-updated velocity.
    {
        JPH::Vec3 worldVel = gCharacter->GetLinearVelocity();
        velocity.x = worldVel.GetX();
        velocity.y = worldVel.GetY();
        velocity.z = worldVel.GetZ();
    }

    // 4) Mouse look.
    Vector2 mouseDelta = GetMouseDelta();
    playerYaw   += mouseDelta.x * MOUSE_SENSITIVITY;
    playerPitch -= mouseDelta.y * MOUSE_SENSITIVITY;
    if (playerPitch > 89.0f) playerPitch = 89.0f;
    if (playerPitch < -89.0f) playerPitch = -89.0f;

    // 5) Handle jump.
    if (IsKeyDown(KEY_SPACE) && isGrounded) {
        velocity.y = JUMP_FORCE;
        isGrounded = false;
        gGroundNormal = JPH::Vec3::sZero();
    }
    if(isGrounded && gGroundState == JPH::CharacterVirtual::EGroundState::OnGround || gGroundState == JPH::CharacterVirtual::EGroundState::OnSteepGround) {
        velocity.y = 0.0f;
    }
 
    // 6) Apply friction, input-based acceleration, gravity.
    PM_Friction(deltaTime);
    PM_AirMove(deltaTime);
    PM_ApplyGravity(deltaTime);
    
    // 7) Update player angles.
    player->yaw = playerYaw;
    player->pitch = playerPitch;

    // 8) Write updated velocity back to Jolt.
    gCharacter->SetLinearVelocity(JPH::Vec3(velocity.x, velocity.y, velocity.z));

    // 9) Update player's position from Jolt.
    JPH::RVec3 finalPos = gCharacter->GetPosition();
    player->center.x = (float)finalPos.GetX();
    player->center.y = (float)finalPos.GetY();
    player->center.z = (float)finalPos.GetZ();

    // 10) Update camera.
    player->camera.position = (Vector3){ player->center.x, player->center.y + eyeOffset, player->center.z };
    float yawRad = DEG2RAD * playerYaw;
    float pitchRad = DEG2RAD * playerPitch;
    Vector3 camForward = { cosf(pitchRad)*sinf(yawRad), sinf(pitchRad), cosf(pitchRad)*cosf(yawRad) };
    player->camera.target = Vector3Add(player->camera.position, camForward);

    // 11) Toggle mouse cursor if needed.
    if (IsKeyPressed(KEY_M)) {
        cursorEnabled = !cursorEnabled;
        if (cursorEnabled)
            EnableCursor();
        else
            DisableCursor();
    }
}

void UpdatePlayer(Player *player,JPH::PhysicsSystem *mPhysicsSystem ,float deltaTime) {

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

    Vector3 desiredVel = Vector3Scale(Vector3Normalize(direction), player->speed);

    JPH::Vec3 charVelocity(desiredVel.x, desiredVel.y, desiredVel.z);
    gCharacter->SetLinearVelocity(charVelocity);

    JPH::CharacterVirtual::ExtendedUpdateSettings updateSettings;
    // Examples for potential usage
    // e.g. updateSettings.mStickToFloorStepDown = JPH::Vec3(0, -0.2f, 0);
    // e.g. updateSettings.mWalkStairsStepUp     = JPH::Vec3(0,  0.2f, 0);

    gCharacter->ExtendedUpdate(
        deltaTime,
        JPH::Vec3(0, 0, 0), // gravity
        updateSettings,
        mPhysicsSystem->JPH::PhysicsSystem::GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        s_char_layer_filter,
        {}, // body filter
        {}, // shape filter
        *s_temp_allocator 
    ); 
     
    JPH::RVec3 finalPos = gCharacter->GetPosition();
     // Apply movement
    player->center = (Vector3) {
        gCharacter->GetPosition().GetX(),
        gCharacter->GetPosition().GetY(),
        gCharacter->GetPosition().GetZ()
    };
    player->camera.position = (Vector3) {player->center.x,
                                         player->center.y + eyeOffset,
                                         player->center.z};
    player->camera.target = Vector3Add(player->camera.target, direction);


    /*
    printf("JOLT POS: x = %f | y = %f | z = %f\n", finalPos.GetX(), finalPos.GetY(), finalPos.GetZ());
    printf("RAYLIB POS: x = %f | y = %f | z = %f\n", player->center.x, player->center.y, player->center.z);
    */
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

void DebugDir(Player *player) {
    // Choose a length for the debug arrows (adjust as needed)
    const float arrowLength = 50.0f;
    
    // Starting point is the player's center.
    Vector3 start = player->center;
    
    // --- Draw Wish Direction ---
    if (Vector3Length(wishDir) < 0.001f) {
        // If no input, draw a small blue sphere at the player's center.
        DrawSphere(start, 3.0f, YELLOW);
    } else {
        // Compute end point for the wish direction arrow.
        Vector3 wishEnd = Vector3Add(start, Vector3Scale(wishDir, arrowLength));
        // Draw a line representing the wish direction (green line).
        DrawLine3D(start, wishEnd, GREEN);
        // Draw a small red sphere at the tip.
        DrawSphere(wishEnd, 2.0f, RED);
    }
    
    // --- Draw Horizontal Velocity ---
    // Ignore the Y component of the velocity vector.
    Vector3 horizVelocity = velocity;
    horizVelocity.y = 0;
    float horizMagnitude = Vector3Length(horizVelocity);
    
    if (horizMagnitude < 0.001f) {
        // If horizontal velocity is nearly zero, draw a small sphere at the start.
        DrawSphere(start, 3.0f, PURPLE);
    } else {
        // Normalize the horizontal velocity.
        Vector3 velDir = Vector3Scale(horizVelocity, 1.0f / horizMagnitude);
        // Compute end point for the horizontal velocity arrow.
        Vector3 velEnd = Vector3Add(start, Vector3Scale(velDir, arrowLength));
        // Draw a blue line representing horizontal velocity.
        DrawLine3D(start, velEnd, BLUE);
        // Draw a dark blue sphere at the tip.
        DrawSphere(velEnd, 2.0f, DARKBLUE);
    }
}
