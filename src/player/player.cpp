// src/player.cpp

#include <cstdio>

#include "player.h"
#include "Jolt/Math/Math.h"
#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"
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
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>        
#include <Jolt/Physics/Collision/CastResult.h>     
#define JPH_ENABLE_ASSERTS

/*  NOTE: Thinking about writing the player collision here directly, could be too messy though
*         considering that all of the quake style movement code will go here as well. 
*         See to adding the collision code back into physics.cpp and just calling it here.
*
*   NOTE: Upon further reading into the Jolt library I believe the best way forward will be to do an implementation of the Jolt 
*         CharacterVirtual here directly, and adapt my original Quake inspired movement code. Input handling will be done by raylib,
*         maybe all of the actual movement calculations can be done using raylibs Vector3 and at the end where we finally update the 
*         player position we do a conversion into Jolt's Vec3? Not sure yet, will use this as the first attempt.
*/

#define MOUSE_SENSITIVITY 0.5f
bool cursorEnabled;

float eyeOffset = 16.0f;

// Movement constants for Quake PM impl.
const float     MAX_SPEED           = 100.0f;        // Maximum speed in m/s
const float     ACCELERATION        = 5.0f;          // Acceleration in m/s^2 
static float    FRICTION            = 4.0f;          // Friction coefficient
const float     STOP_SPEED          = 10.0f;         // Speed below which the player stops
const float     JUMP_FORCE          = 80.0f;         // Jump force
const float     AIR_WISH_SPEED_CAP  = 12.0f;         // Capping wishSpeed magnitude while in air. higher values = more speed gain
const float     AIR_ACCELERATION    = 100.0f;        // Air acceleration in m/s^2 higher values = more air control
const float     GRAVITY             = -98.1f;        // Gravity constant

Vector3 velocity = Vector3Zero();
float playerYaw = 0.0f;
float playerPitch = 0.0f;
bool isGrounded = false;

static bool s_allow_sliding     = true;             // Fix for sliding on slopes when idle. True while player is actively providing input.
static bool s_has_move_input    = false;            // Check if the player is currently providing movement input.
static bool s_skip_slope_cancel_this_frame = false; // Skip slope movement cancel this frame for better control feel.

Vector3 wishDir     = Vector3Zero();
Vector3 wishVel     = Vector3Zero();
float   wishSpeed   = 0.0f;

// =========
// DEBUG
// ========
struct VelDebug {
    float horiz = 0.0f;     // sqrt(vx^2 + vz^2)
    float vert  = 0.0f;     // vy
    float total = 0.0f;     // |v|
    float peakH = 0.0f;     // peak horizontal speed this run
    float lastH = 0.0f;     // last-frame horiz (for delta color)
};
static VelDebug g_velDbg;

static inline float Clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// Call this after reading back post-solve velocity each frame
void DebugUpdateVelMetrics() {
    g_velDbg.horiz = sqrtf(velocity.x*velocity.x + velocity.z*velocity.z);
    g_velDbg.vert  = velocity.y;
    g_velDbg.total = Vector3Length(velocity);
    if (g_velDbg.horiz > g_velDbg.peakH) g_velDbg.peakH = g_velDbg.horiz;
}

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

    virtual void OnContactSolve(const JPH::CharacterVirtual *inCharacter,
                                const JPH::BodyID &inBodyID2,
                                const JPH::SubShapeID &inSubShapeID2,
                                JPH::RVec3Arg inContactPosition,
                                JPH::Vec3Arg inContactNormal,
                                JPH::Vec3Arg inContactVelocity,
                                const JPH::PhysicsMaterial *inContactMaterial,
                                JPH::Vec3Arg inCharacterVelocity,
                                JPH::Vec3 &ioNewCharacterVelocity) override
    {
        // Cancel only the into-plane component when idle on walkable slopes
        if (inCharacter != gCharacter) return;
        if (inCharacter->IsSlopeTooSteep(inContactNormal)) return;
        if (s_has_move_input) return;
        if (s_skip_slope_cancel_this_frame) return;

        // If the surface isn't moving, kill only the normal component so gravity can't
        // inject along-slope drift, but keep whatever tangential speed we already have.
        if (inContactVelocity.IsNearZero())
        {
            JPH::Vec3 n = inContactNormal.Normalized();
            float vn = ioNewCharacterVelocity.Dot(n);
            if (vn < 0.0f) {
                ioNewCharacterVelocity -= n * vn; // remove into-plane
            }
        }
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
    const float radius = 16.0f;
    const float height = 56.0f;
    const float cyl_h  = height - 2.0f * radius;

    JPH::RefConst<JPH::CapsuleShape> cap = new JPH::CapsuleShape(0.5f * cyl_h, radius);

    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
    settings->mShape                       = cap;
    settings->mMaxSlopeAngle               = JPH::DegreesToRadians(45.0f);
    settings->mMaxStrength                 = 150.0f;
    settings->mCharacterPadding            = 0.5f;
    settings->mPenetrationRecoverySpeed    = 3.0f;
    settings->mPredictiveContactDistance   = 2.0f;
    settings->mBackFaceMode                = JPH::EBackFaceMode::CollideWithBackFaces;
    settings->mEnhancedInternalEdgeRemoval = true;
    settings->mSupportingVolume            = JPH::Plane(JPH::Vec3::sAxisY(), -radius);
    settings->mInnerBodyLayer              = Layers::MOVING;

    gCharacter = new JPH::CharacterVirtual(
        settings,
        JPH::RVec3(player->center.x, player->center.y, player->center.z),
        JPH::Quat::sIdentity(),
        0,
        physicsSystem
    );

    gCharacter->SetListener(&s_char_listener);
    gCharacter->SetUp(JPH::Vec3::sAxisY());
}

void InitPlayer(Player *player, Vector3 center, Vector3 target, Vector3 up, float fovy, int projection) {
    player->center = center;

    player->camera.position   = (Vector3){ center.x, center.y + eyeOffset, center.z };
    player->camera.target     = target;
    player->camera.up         = up;
    player->camera.fovy       = fovy;
    player->camera.projection = projection;

    player->yaw = 0.0f;
    player->pitch = 0.0f;
    UpdateCameraTarget(player);

    player->speed = 200.0f;
    player->rotationSpeed = 90.0f;
    player->halfExt = (Vector3){16,28,16};
    SetMousePosition(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    DisableCursor();
    cursorEnabled = false;
}

// --------- BEGIN Quake Movement Impl. ---------


// Projection of the player's movement vector along slopes so we dont fight the force of gravity up and down slopes.
static inline Vector3 ProjectOntoPlane(Vector3 v, Vector3 n) {
    // n must be normalized
    return Vector3Subtract(v, Vector3Scale(n, Vector3DotProduct(v, n)));
}

static inline bool OnWalkableGround() {
    return isGrounded && !gCharacter->IsSlopeTooSteep(gGroundNormal);
}

static inline void PM_ClipVelocity(const Vector3 &in, const Vector3 &normal, Vector3 &out, float overbounce)
{
    float backoff = Vector3DotProduct(in, normal) * overbounce;

    out.x = in.x - normal.x * backoff;
    out.y = in.y - normal.y * backoff;
    out.z = in.z - normal.z * backoff;

    // stop tiny jitter
    const float STOP_EPSILON = 0.1f;
    if (out.x > -STOP_EPSILON && out.x < STOP_EPSILON) out.x = 0.0f;
    if (out.y > -STOP_EPSILON && out.y < STOP_EPSILON) out.y = 0.0f;
    if (out.z > -STOP_EPSILON && out.z < STOP_EPSILON) out.z = 0.0f;
}

// Apply friction when grounded
void PM_Friction(float dt)
{
    // Must be on contact; Quake only frictions when grounded.
    if (!isGrounded) return;

    // Use the best normal we have; fall back to +Y.
    Vector3 n = {0.0f, 1.0f, 0.0f};
    if (gGroundNormal.LengthSq() > 1e-6f) {
        n = (Vector3){ gGroundNormal.GetX(), gGroundNormal.GetY(), gGroundNormal.GetZ() };
        float ln = Vector3Length(n);
        if (ln > 0.0f) n = Vector3Scale(n, 1.0f / ln);
        else           n = (Vector3){0.0f, 1.0f, 0.0f};
    }

    // Decompose velocity into normal + tangent (Quake did vel[2]=0 before friction.
    // here we generalize by using the plane tangent).
    float   vn    = Vector3DotProduct(velocity, n);
    Vector3 vtan  = Vector3Subtract(velocity, Vector3Scale(n, vn));
    float   speed = Vector3Length(vtan);

    // Quake snap: if really slow, stop tangential motion (keeps normal component).
    if (speed < 1.0f) { // matches Quake’s threshold
        velocity = Vector3Scale(n, vn);
        return;
    }

    // (Optional) “ledge” check from Quake would double friction near drop-offs.
    // Not implemented here because it needs a short trace, keep FRICTION as-is.
    float friction = FRICTION;

    // Quake math: control = max(STOP_SPEED, speed), drop = control*friction*dt
    float control = (speed < STOP_SPEED) ? STOP_SPEED : speed;
    float drop    = control * friction * dt;

    float newSpeed = speed - drop;
    if (newSpeed < 0.0f) newSpeed = 0.0f;

    float scale = (speed > 0.0f) ? (newSpeed / speed) : 0.0f;
    vtan = Vector3Scale(vtan, scale);

    // Recompose: keep normal component (don’t fight contact normal), reduce tangent.
    velocity = Vector3Add(Vector3Scale(n, vn), vtan);
}

static inline void PM_BuildWish()
{
    float fmove = 0.0f, smove = 0.0f;
    if (IsKeyDown(KEY_W)) fmove += 1.0f;
    if (IsKeyDown(KEY_S)) fmove -= 1.0f;
    if (IsKeyDown(KEY_D)) smove += 1.0f;
    if (IsKeyDown(KEY_A)) smove -= 1.0f;

    const float yawRad = DEG2RAD * playerYaw;
    const Vector3 forward = { cosf(yawRad), 0.0f, sinf(yawRad) };
    const Vector3 right   = { -sinf(yawRad), 0.0f, cosf(yawRad) };

    // raw (unnormalized) horizontal wish velocity like Quake's wishveloc
    wishVel = Vector3Add(Vector3Scale(forward, fmove), Vector3Scale(right, smove));

    const float inputMag = Vector3Length(wishVel);    // 0..sqrt(2)
    s_has_move_input = (inputMag > 0.0001f);

    // wishDir: unit vector (if any input)
    wishDir = (inputMag > 0.0f) ? Vector3Scale(wishVel, 1.0f / inputMag) : Vector3Zero();

    // Quake: wishspeed = min(length(wishvel), sv_maxspeed)
    const float inputScale = fminf(1.0f, inputMag);
    wishSpeed = MAX_SPEED * inputScale;
}

// Modified PM_Accelerate: Projects wishDir onto the slope plane if needed.
static void PM_Accelerate(Vector3 wishDir, float wishSpeed, float accel, float dt)
{
    // NOTE: Removed slope projection logic from accelerate 
    float currentSpeed = Vector3DotProduct(velocity, wishDir);
    float addSpeed     = wishSpeed - currentSpeed;
    if (addSpeed <= 0.0f) return;

    // Quake: include wishSpeed
    float accelSpeed = accel * dt * wishSpeed;
    if (accelSpeed > addSpeed) accelSpeed = addSpeed;

    velocity = Vector3Add(velocity, Vector3Scale(wishDir, accelSpeed));
}

// Air acceleration is where all the fun stuff happens.
static void PM_AirAccelerate(Vector3 wishDir, float wishSpeed, float accel, float dt)
{
   float wishspd = wishSpeed;

    // Slope handling for surfing on ramps
    if (gGroundNormal.Length() > 0.001f) {
        Vector3 n = { gGroundNormal.GetX(), gGroundNormal.GetY(), gGroundNormal.GetZ() };
        float ln = Vector3Length(n); if (ln > 0.0f) n = Vector3Scale(n, 1.0f / ln);

        if (Vector3DotProduct(velocity, n) < 0.0f) {
            Vector3 clipped; PM_ClipVelocity(velocity, n, clipped, 1.0f); velocity = clipped;
        }
        if (Vector3DotProduct(wishDir, n) < 0.0f) {
            Vector3 steerv = wishDir; PM_ClipVelocity(steerv, n, steerv, 1.0f);
            float sl = Vector3Length(steerv);
            if (sl > 0.0001f) wishDir = Vector3Scale(steerv, 1.0f / sl);
        }
    }

    if (wishspd > AIR_WISH_SPEED_CAP) wishspd = AIR_WISH_SPEED_CAP;

    float currentspeed = Vector3DotProduct(velocity, wishVel);
    float addspeed     = wishspd - currentspeed;
    if (addspeed <= 0.0f) return;

    // Quake uses global wishSpeed (sv_maxspeed-clamped) here
    float accelspeed = accel * wishSpeed * dt;
    if (accelspeed > addspeed) accelspeed = addspeed;

    velocity = Vector3Add(velocity, Vector3Scale(wishVel, accelspeed));
}

// PM_AirMove: Applies the acceleration functions to impulse the player in the desired movement directions.
static void PM_AirMove(float dt, bool grounded)
{ 
    bool walkable = OnWalkableGround();
    if (walkable && grounded) {
        Vector3 n = { gGroundNormal.GetX(), gGroundNormal.GetY(), gGroundNormal.GetZ() };
        float ln = Vector3Length(n); if (ln > 0.0f) n = Vector3Scale(n, 1.0f / ln);

        Vector3 p = ProjectOntoPlane(wishDir, n);
        float pl = Vector3Length(p);
        if (pl > 0.0f) wishDir = Vector3Scale(p, 1.0f / pl);

        PM_Friction(dt);
        PM_Accelerate(wishDir, wishSpeed, ACCELERATION, dt);
    } else {
        PM_AirAccelerate(wishDir, wishSpeed, AIR_ACCELERATION, dt);
    }
}

static void PM_ApplyGravity(float deltaTime) {
    if (!isGrounded) {
        velocity.y += GRAVITY *deltaTime;
    }
}

// -------- END Quake Movement Impl. ---------

void UpdatePlayerMove(Player *player, JPH::PhysicsSystem *ps, float dt) {

    bool was_grounded_this_frame_start = isGrounded; // last-frame value

    // 1) Mouse look -> yaw/pitch/camera target
    Vector2 md = GetMouseDelta();
    playerYaw   += md.x * MOUSE_SENSITIVITY;
    playerPitch -= md.y * MOUSE_SENSITIVITY;
    playerPitch  = fmaxf(fminf(playerPitch, 89.0f), -89.0f);
    player->yaw   = playerYaw;
    player->pitch = playerPitch;

    // 2) Ground/platform velocity refresh
    gCharacter->UpdateGroundVelocity();

    // 3) Start from current velocity
    {
        JPH::Vec3 v = gCharacter->GetLinearVelocity();
        velocity = { v.GetX(), v.GetY(), v.GetZ() };

         // movement debug
        DebugUpdateVelMetrics();
    }
     
    // 4) Input → wish dir/speed  
    PM_BuildWish();

    // 5) Jump
    if ((IsKeyDown(KEY_SPACE) && isGrounded)) {
        velocity.y   = JUMP_FORCE;
        isGrounded   = false;
        gGroundNormal = JPH::Vec3::sZero();
    }

    // 6) Gravity
    PM_ApplyGravity(dt);

    // 7) Write velocity to character (pre-solve)
    gCharacter->SetLinearVelocity(JPH::Vec3(velocity.x, velocity.y, velocity.z));

    s_allow_sliding = s_has_move_input;
 
    // 8) Configure ExtendedUpdate (stick-to-floor / stair step)
    JPH::CharacterVirtual::ExtendedUpdateSettings us;
    const float step_up = 10.0f; // ~8 units in TB
    const float step_dn = step_up;
    us.mWalkStairsStepUp     =  gCharacter->GetUp() * step_up;
    us.mStickToFloorStepDown = -gCharacter->GetUp() * step_dn;

    // 9) Gravity for CharacterVirtual 
    const float g = 98.1f; 
    const JPH::Vec3 gravity = -gCharacter->GetUp() * g;

    // 10) Do the move
    gCharacter->ExtendedUpdate(
        dt,
        gravity,
        us,
        ps->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        s_char_layer_filter,   // object layer filter
        {}, {},                // body/shape filters
        *s_temp_allocator
    );

    // 11) Query ground state (truth source)
    JPH::CharacterVirtual::EGroundState new_state = gCharacter->GetGroundState();
    bool nowGrounded = (new_state == JPH::CharacterVirtual::EGroundState::OnGround);
    JPH::Vec3 gn = gCharacter->GetGroundNormal();
    gGroundNormal = gn;
    bool walkable = nowGrounded && !gCharacter->IsSlopeTooSteep(gn);
    isGrounded = nowGrounded; // update the global for the rest of the frame

    // 12) Read back velocity (post-solve) and position
    {
        JPH::Vec3 nv = gCharacter->GetLinearVelocity();
        velocity = { nv.GetX(), nv.GetY(), nv.GetZ() };

        JPH::RVec3 p = gCharacter->GetPosition();
        player->center = { (float)p.GetX(), (float)p.GetY(), (float)p.GetZ() };
    }

    // 13) Landing handling + friction (post-solve)
    bool landedThisFrame = (!was_grounded_this_frame_start && walkable);
    s_skip_slope_cancel_this_frame = landedThisFrame; // OnContactSolve idle-slope logic
    
    // Quake: apply friction every grounded tick (including the landing tick)
    PM_AirMove(dt, isGrounded);

    gCharacter->SetLinearVelocity(JPH::Vec3(velocity.x, velocity.y, velocity.z));

    // 14) Set camera from center + orientation
    player->camera.position = { player->center.x, player->center.y + eyeOffset, player->center.z };
    float yawRad = DEG2RAD * playerYaw;
    float pitchRad = DEG2RAD * playerPitch;
    Vector3 fwd = { cosf(pitchRad) * sinf(yawRad), sinf(pitchRad), cosf(pitchRad) * cosf(yawRad) };
    player->camera.target = Vector3Add(player->camera.position, fwd);
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
    Vector3 start = player->center;

    // --- WishDir (unit vector, length always 1 if input present) ---
    float wishDirLen = Vector3Length(wishDir);
    if (wishDirLen > 0.0001f) {
        Vector3 wishEnd = Vector3Add(start, wishDir * 50);
        DrawLine3D(start, wishEnd, GREEN);
        DrawSphere(wishEnd, 2.0f, LIME);
    } else {
        DrawSphere(start, 3.0f, YELLOW); // no input
    }

    // --- WishVel (raw vector before normalization) ---
    float wishVelLen = Vector3Length(wishVel);
    if (wishVelLen > 0.0001f) {
        Vector3 wishVelEnd = Vector3Add(start, wishVel);
        DrawLine3D(start, wishVelEnd, ORANGE);
        DrawSphere(wishVelEnd, 2.0f, GOLD);
    }

    // --- Horizontal Velocity (actual, no scaling) ---
    Vector3 horizVelocity = velocity;
    horizVelocity.y = 0;
    float horizMag = Vector3Length(horizVelocity);

    if (horizMag > 0.0001f) {
        Vector3 velEnd = Vector3Add(start, horizVelocity);
        DrawLine3D(start, velEnd, BLUE);
        DrawSphere(velEnd, 2.0f, DARKBLUE);
    } else {
        DrawSphere(start, 3.0f, PURPLE); // no horizontal velocity
    }
}

void DebugDrawPlayerPos(const Player *player, int x, int y) { 
    // Format position
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "Player Pos: X=%.2f  Y=%.2f  Z=%.2f",
             player->center.x, player->center.y, player->center.z);

    DrawText(buffer, x, y, 20, RED);
}

void DebugDrawPlayerVel(int x, int y) {
    // Color: green if speeding up, red if slowing, gray if ~same
    float dh = g_velDbg.horiz - g_velDbg.lastH;
    Color c = (dh > 0.1f) ? GREEN : (dh < -0.1f) ? RED : LIGHTGRAY;

    char line[192];
    snprintf(line, sizeof(line),
             "H: %.2f   V: %.2f   |v|: %.2f   PeakH: %.2f   %s",
             g_velDbg.horiz, g_velDbg.vert, g_velDbg.total, g_velDbg.peakH,
             isGrounded ? "GROUND" : "AIR");

    DrawText(line, x, y, 20, c);

    // Horizontal speed bar 
    const float maxShow = 600.0f;     // display ceiling
    const float w = 240.0f, h = 10.0f;
    DrawRectangleLines(x, y + 26, (int)w, (int)h, GRAY);
    float fill = w * Clamp01(g_velDbg.horiz / maxShow);
    DrawRectangle(x + 1, y + 27, (int)(fill - 2 > 0 ? fill - 2 : 0), (int)h - 2, BLUE);

    g_velDbg.lastH = g_velDbg.horiz;
}
