// src/player.cpp

#include "player.h"
#include "raylib.h"
#include "raymath.h"
#include "../utils/parameters.h"
#include "../physx/physics.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Core/Core.h"
#include "Jolt/Core/JobSystem.h"
#include "Jolt/Geometry/Sphere.h"
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
#include "Jolt/Math/Float3.h"
#include "Jolt/Physics/Body/MotionType.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/Shape/ConvexShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Geometry/ConvexSupport.h"
#include "Jolt/Geometry/GJKClosestPoint.h"
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
const float MAX_SPEED = 320.0f;            // Maximum speed in m/s (adjusted from 250.0f)
const float ACCELERATION = 100.0f;         // Acceleration in m/s² (adjusted from 100.0f)
const float FRICTION = 8.0f;              // Friction coefficient
const float STOP_SPEED = 1.0f;            // Speed below which the player stops
const float JUMP_FORCE = 10.0f;            // Jump force
const float AIR_ACCELERATION = 250.0f;     // Air acceleration in m/s²
const float MAX_AIR_SPEED = 100.0f;        // Maximum speed while in air
bool isGrounded = false;

static JPH::CharacterVirtual *gCharacter = nullptr;

static JPH::Vec3 gQuakeVelocity = JPH::Vec3::sZero();

// --------- Character Collision Impl. ---------
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
        printf("Character left body ID=%d\n", inBodyID2.GetIndexAndSequenceNumber());
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
                printf("[CharacterListener] Trigger still active.\n");
            }
        }
        else {
            // This is normal geometry
            // Possibly do normal character stuff
        }
    }
};

static MyCharacterListener s_char_listener;
static MyCharacterLayerFilter s_char_layer_filter;

void InitJoltCharacter(Player *player, JPH::PhysicsSystem *physicsSystem) {

    JPH::RefConst<JPH::Shape> playerShape = new JPH::BoxShape(JPH::Vec3(16.0f, 28.0f, 16.0f));

    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
    settings->mShape = playerShape;
    settings->mMaxSlopeAngle = JPH::DegreesToRadians(50.0f);
    settings->mMaxStrength   = 150.0f;
    settings->mCharacterPadding = 0.02f;
    settings->mPenetrationRecoverySpeed = 1.0f;
    settings->mPredictiveContactDistance = 0.1f;
    settings->mInnerBodyLayer = Layers::MOVING;

    gCharacter = new JPH::CharacterVirtual(
        settings,
        JPH::RVec3(player->camera.position.x, 
                   player->camera.position.y - eyeOffset,
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
        gCharacter->GetPosition().GetY() - eyeOffset,
        gCharacter->GetPosition().GetZ()
    };
    player->camera.position = player->center;
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
