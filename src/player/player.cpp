// src/player/player.cpp
//
// GoldSrc-style movement using manual Jolt box sweeps.

#include <cmath>

#include "player.h"
#include "../math/wmath.h"
#include "../input/input.h"
#include "../render/debug_draw.h"
#include "../utils/parameters.h"
#include "../physx/physics.h"

#include "sokol_gfx.h"
#include "sokol_debugtext.h"

#include "Jolt/Jolt.h"
#include "Jolt/Math/Math.h"
#include "Jolt/Math/Real.h"
#include "Jolt/Math/Vec3.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Body/BodyLock.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/Collision/NarrowPhaseQuery.h"
#include "Jolt/Physics/Collision/RayCast.h"
#include "Jolt/Physics/Collision/CastResult.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/ShapeCast.h"
#include "Jolt/Physics/Collision/CollideShape.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"

#define JPH_ENABLE_ASSERTS

#define MOUSE_SENSITIVITY 0.5f

bool cursorEnabled = false;
float eyeOffset = 16.0f;

static constexpr float MAX_SPEED = 250.0f;
static constexpr float ACCELERATION = 10.0f;
static constexpr float AIR_ACCELERATION = 100.0f;
static constexpr float FRICTION = 6.0f;
static constexpr float STOP_SPEED = 100.0f;
static constexpr float AIR_WISH_SPEED_CAP = 30.0f;
static constexpr float GRAVITY = 800.0f;
static constexpr float JUMP_HEIGHT = 45.0f;
static constexpr float STEP_HEIGHT = 18.0f;
static constexpr float GROUND_NORMAL_MIN = 0.70710677f; // cos(45 deg)
static constexpr float SLOPE_STOP_SPEED_EPSILON = 1.0f;
static constexpr float GROUND_PROBE_HALF_THICKNESS = 0.1f;
static constexpr float OVERCLIP = 1.001f;
static constexpr float STOP_EPSILON = 0.1f;
static constexpr float CLIP_PLANE_EPSILON = 0.1f;
static constexpr float DUPLICATE_PLANE_DOT = 0.99f;
static constexpr float GROUND_PROBE_DISTANCE = 2.0f;
static constexpr float GROUND_CONTACT_DISTANCE = 0.25f;
static constexpr float POSITION_EPSILON = 0.02f;
static constexpr int MAX_CLIP_PLANES = 5;
static constexpr int MAX_BUMPS = 4;
static constexpr int MAX_PENETRATION_ITERS = 8;
static constexpr float PLAYER_RADIUS = 16.0f;
static constexpr float PLAYER_HEIGHT = 56.0f;
static constexpr float PLAYER_HALF_HEIGHT = PLAYER_HEIGHT * 0.5f;
static constexpr float JUMP_FORCE = 268.3281573f; // sqrt(2 * 800 * 45)

Vector3 velocity = Vector3Zero();
float playerYaw = 0.0f;
float playerPitch = 0.0f;
bool isGrounded = false;

Vector3 wishDir = Vector3Zero();
Vector3 wishVel = Vector3Zero();
float wishSpeed = 0.0f;

static JPH::RefConst<JPH::Shape> gPlayerShape;
static JPH::RefConst<JPH::Shape> gGroundProbeShape;
static JPH::Vec3 gGroundNormal = JPH::Vec3::sAxisY();

// Debug colors
static const Color C_RED    = WCOLOR(230, 41, 55, 255);
static const Color C_GREEN  = WCOLOR(  0,228, 48, 255);
static const Color C_LIME   = WCOLOR(  0,158, 47, 255);
static const Color C_YELLOW = WCOLOR(253,249,  0, 255);
static const Color C_ORANGE = WCOLOR(255,161,  0, 255);
static const Color C_GOLD   = WCOLOR(255,203,  0, 255);
static const Color C_BLUE   = WCOLOR(  0,121,241, 255);
static const Color C_DBLUE  = WCOLOR(  0, 82,172, 255);
static const Color C_PURPLE = WCOLOR(200,122,255, 255);

struct VelDebug {
    float horiz = 0.0f;
    float vert  = 0.0f;
    float total = 0.0f;
    float peakH = 0.0f;
    float lastH = 0.0f;
};
static VelDebug g_velDbg;

struct MoveTrace {
    bool hit = false;
    bool startSolid = false;
    float fraction = 1.0f;
    Vector3 endPos = Vector3Zero();
    Vector3 normal = { 0.0f, 1.0f, 0.0f };
    JPH::BodyID bodyID;
};

struct GroundSupport {
    bool found = false;
    float centerY = 0.0f;
    Vector3 normal = { 0.0f, 1.0f, 0.0f };
};

struct GroundProbeDebug {
    bool valid = false;
    bool hadHit = false;
    Vector3 startCenter = Vector3Zero();
    Vector3 endCenter = Vector3Zero();
    Vector3 hitCenter = Vector3Zero();
    Vector3 hitNormal = { 0.0f, 1.0f, 0.0f };
};
static GroundProbeDebug gGroundProbeDebug;

static GroundSupport FindGroundSupport(JPH::PhysicsSystem *ps, Vector3 position, float maxDistance);

class PlayerQueryLayerFilter : public JPH::ObjectLayerFilter
{
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override
    {
        return inLayer == Layers::NON_MOVING || inLayer == Layers::MOVING;
    }
};

static PlayerQueryLayerFilter s_player_query_layer_filter;

static inline float Clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
static inline float HorizontalLength(Vector3 v) { return sqrtf(v.x * v.x + v.z * v.z); }
static inline float HorizontalLengthSq(Vector3 v) { return v.x * v.x + v.z * v.z; }
static inline bool IsWalkableNormal(Vector3 n) { return n.y >= GROUND_NORMAL_MIN; }
static inline bool OnWalkableGround() { return isGrounded && gGroundNormal.GetY() >= GROUND_NORMAL_MIN; }

static inline JPH::Vec3 ToJoltVec3(Vector3 v) { return JPH::Vec3(v.x, v.y, v.z); }
static inline JPH::RVec3 ToJoltRVec3(Vector3 v) { return JPH::RVec3(v.x, v.y, v.z); }
static inline Vector3 FromJoltVec3(JPH::Vec3Arg v) { return { v.GetX(), v.GetY(), v.GetZ() }; }
static inline Vector3 FromJoltRVec3(JPH::RVec3Arg v) { return { (float)v.GetX(), (float)v.GetY(), (float)v.GetZ() }; }
static inline Vector3 GroundNormalVector() { return FromJoltVec3(gGroundNormal); }

static inline Vector3 ProjectVectorOntoPlane(Vector3 v, Vector3 normal)
{
    return Vector3Subtract(v, Vector3Scale(normal, Vector3DotProduct(v, normal)));
}

static Vector3 ReprojectVelocityPreserveSpeed(Vector3 velocity, Vector3 normal, float targetSpeed)
{
    Vector3 projected = ProjectVectorOntoPlane(velocity, normal);
    float lenSq = Vector3LengthSq(projected);
    if (lenSq <= 1e-8f || targetSpeed <= 0.0f)
        return Vector3Zero();
    return Vector3Scale(projected, targetSpeed / sqrtf(lenSq));
}

static Vector3 ProjectDirectionOntoPlane(Vector3 dir, Vector3 normal)
{
    Vector3 projected = ProjectVectorOntoPlane(dir, normal);
    float lenSq = Vector3LengthSq(projected);
    if (lenSq <= 1e-8f)
        return Vector3Zero();
    return Vector3Scale(projected, 1.0f / sqrtf(lenSq));
}

static void DebugUpdateVelMetrics()
{
    g_velDbg.horiz = HorizontalLength(velocity);
    g_velDbg.vert = velocity.y;
    g_velDbg.total = Vector3Length(velocity);
    if (g_velDbg.horiz > g_velDbg.peakH) g_velDbg.peakH = g_velDbg.horiz;
}

static MoveTrace CastPlayerShape(JPH::PhysicsSystem *ps, Vector3 start, Vector3 delta)
{
    MoveTrace trace;
    trace.endPos = Vector3Add(start, delta);

    if (Vector3LengthSq(delta) <= 1e-10f)
        return trace;

    const JPH::RShapeCast cast(
        gPlayerShape.GetPtr(),
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(ToJoltRVec3(start)),
        ToJoltVec3(delta)
    );

    JPH::ShapeCastSettings settings;
    settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mUseShrunkenShapeAndConvexRadius = false;
    settings.mReturnDeepestPoint = true;
    settings.mActiveEdgeMovementDirection = ToJoltVec3(delta);

    JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
    ps->GetNarrowPhaseQuery().CastShape(
        cast,
        settings,
        JPH::RVec3::sZero(),
        collector,
        ps->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        s_player_query_layer_filter
    );

    if (!collector.HadHit())
        return trace;

    trace.hit = true;
    trace.fraction = collector.mHit.mFraction;
    trace.endPos = Vector3Add(start, Vector3Scale(delta, trace.fraction));

    JPH::Vec3 axis = collector.mHit.mPenetrationAxis;
    if (axis.LengthSq() > 1e-12f)
        trace.normal = FromJoltVec3(-axis.Normalized());

    trace.startSolid = trace.fraction <= 0.0f && collector.mHit.mPenetrationDepth > POSITION_EPSILON;
    trace.bodyID = collector.mHit.mBodyID2;
    return trace;
}

static bool ResolvePlayerPenetration(JPH::PhysicsSystem *ps, Vector3 &position)
{
    bool moved = false;

    for (int i = 0; i < MAX_PENETRATION_ITERS; ++i)
    {
        JPH::CollideShapeSettings settings;
        settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

        JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> collector;
        ps->GetNarrowPhaseQuery().CollideShapeWithInternalEdgeRemoval(
            gPlayerShape.GetPtr(),
            JPH::Vec3::sReplicate(1.0f),
            JPH::RMat44::sTranslation(ToJoltRVec3(position)),
            settings,
            JPH::RVec3::sZero(),
            collector,
            ps->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
            s_player_query_layer_filter
        );

        if (!collector.HadHit() || collector.mHit.mPenetrationDepth <= 0.0f)
            return moved;

        JPH::Vec3 axis = collector.mHit.mPenetrationAxis;
        if (axis.LengthSq() <= 1e-12f)
            return moved;

        Vector3 resolve = FromJoltVec3(-axis.Normalized());
        const float resolveDistance = collector.mHit.mPenetrationDepth + POSITION_EPSILON;

        if (resolve.y >= GROUND_NORMAL_MIN)
        {
            GroundSupport support = FindGroundSupport(ps, position, GROUND_PROBE_DISTANCE);
            if (support.found && IsWalkableNormal(support.normal))
            {
                const float verticalDistance = resolveDistance / fmaxf(resolve.y, GROUND_NORMAL_MIN);
                const float snappedY = support.centerY + POSITION_EPSILON;
                const float raisedY = position.y + verticalDistance;
                position.y = fmaxf(raisedY, snappedY);
                moved = true;
                continue;
            }
        }

        position = Vector3Add(position, Vector3Scale(resolve, resolveDistance));
        moved = true;
    }

    return moved;
}

static GroundSupport FindGroundSupport(JPH::PhysicsSystem *ps, Vector3 position, float maxDistance)
{
    GroundSupport best;

    if (!gGroundProbeShape || maxDistance <= 0.0f)
    {
        gGroundProbeDebug.valid = false;
        return best;
    }

    Vector3 probeStart = {
        position.x,
        position.y - PLAYER_HALF_HEIGHT + GROUND_PROBE_HALF_THICKNESS + POSITION_EPSILON,
        position.z
    };

    const float castDistance = maxDistance + POSITION_EPSILON;
    gGroundProbeDebug.valid = true;
    gGroundProbeDebug.hadHit = false;
    gGroundProbeDebug.startCenter = probeStart;
    gGroundProbeDebug.endCenter = { probeStart.x, probeStart.y - castDistance, probeStart.z };
    gGroundProbeDebug.hitCenter = gGroundProbeDebug.endCenter;
    gGroundProbeDebug.hitNormal = { 0.0f, 1.0f, 0.0f };
    const JPH::RShapeCast cast(
        gGroundProbeShape.GetPtr(),
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(ToJoltRVec3(probeStart)),
        JPH::Vec3(0.0f, -castDistance, 0.0f)
    );

    JPH::ShapeCastSettings settings;
    settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mUseShrunkenShapeAndConvexRadius = false;
    settings.mReturnDeepestPoint = true;
    settings.mActiveEdgeMovementDirection = JPH::Vec3(0.0f, -1.0f, 0.0f);

    JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
    ps->GetNarrowPhaseQuery().CastShape(
        cast,
        settings,
        JPH::RVec3::sZero(),
        collector,
        ps->GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
        s_player_query_layer_filter
    );

    if (!collector.HadHit())
        return best;

    collector.Sort();
    for (const JPH::ShapeCastResult &hit : collector.mHits)
    {
        JPH::Vec3 axis = hit.mPenetrationAxis;
        if (axis.LengthSq() <= 1e-12f)
            continue;

        Vector3 normal = FromJoltVec3(-axis.Normalized());
        if (!IsWalkableNormal(normal))
            continue;

        const float probeCenterY = probeStart.y - castDistance * hit.mFraction;
        const float centerY = probeCenterY + (PLAYER_HALF_HEIGHT - GROUND_PROBE_HALF_THICKNESS);
        if (!best.found || centerY > best.centerY)
        {
            best.found = true;
            best.centerY = centerY;
            best.normal = normal;
            gGroundProbeDebug.hadHit = true;
            gGroundProbeDebug.hitCenter = { probeStart.x, probeCenterY, probeStart.z };
            gGroundProbeDebug.hitNormal = normal;
        }
    }

    return best;
}

static inline void PM_ClipVelocity(const Vector3 &in, const Vector3 &normal, Vector3 &out, float overbounce)
{
    float backoff = Vector3DotProduct(in, normal) * overbounce;

    out.x = in.x - normal.x * backoff;
    out.y = in.y - normal.y * backoff;
    out.z = in.z - normal.z * backoff;

    if (out.x > -STOP_EPSILON && out.x < STOP_EPSILON) out.x = 0.0f;
    if (out.y > -STOP_EPSILON && out.y < STOP_EPSILON) out.y = 0.0f;
    if (out.z > -STOP_EPSILON && out.z < STOP_EPSILON) out.z = 0.0f;
}

static bool RecoverTangentVelocity(const Vector3 *planes, int numPlanes, const Vector3 &sourceVelocity, Vector3 &outVelocity)
{
    float bestSpeedSq = 0.0f;
    Vector3 bestVelocity = Vector3Zero();

    for (int i = 0; i < numPlanes; ++i)
    {
        Vector3 candidate;
        PM_ClipVelocity(sourceVelocity, planes[i], candidate, 1.0f);

        int j = 0;
        for (; j < numPlanes; ++j)
        {
            if (j != i && Vector3DotProduct(candidate, planes[j]) < -CLIP_PLANE_EPSILON)
                break;
        }

        if (j != numPlanes)
            continue;

        if (Vector3DotProduct(candidate, sourceVelocity) <= STOP_EPSILON)
            continue;

        float speedSq = Vector3LengthSq(candidate);
        if (speedSq <= bestSpeedSq)
            continue;

        bestSpeedSq = speedSq;
        bestVelocity = candidate;
    }

    if (bestSpeedSq <= 1e-8f)
        return false;

    outVelocity = bestVelocity;
    return true;
}

static void PM_Friction(float dt)
{
    if (!OnWalkableGround())
        return;

    Vector3 groundVelocity = ProjectVectorOntoPlane(velocity, GroundNormalVector());
    float speed = Vector3Length(groundVelocity);
    if (speed <= 0.0f)
        return;

    float control = speed < STOP_SPEED ? STOP_SPEED : speed;
    float drop = control * FRICTION * dt;
    float newGroundSpeed = speed - drop;
    if (newGroundSpeed < 0.0f) newGroundSpeed = 0.0f;

    if (speed > 0.0f)
        newGroundSpeed /= speed;

    groundVelocity = Vector3Scale(groundVelocity, newGroundSpeed);
    velocity = groundVelocity;
}

static void PM_ApplyWalkableSlopeStop()
{
    if (!OnWalkableGround())
        return;

    const float groundSpeed = Vector3Length(velocity);
    const bool hasInput = wishSpeed > 0.001f;
    const bool stopOnSlope = !hasInput && groundSpeed <= SLOPE_STOP_SPEED_EPSILON;
    if (stopOnSlope)
        velocity = Vector3Zero();
}

static inline void PM_BuildWish()
{
    float fmove = 0.0f;
    float smove = 0.0f;

    if (Input_KeyDown(WKEY_W)) fmove += 1.0f;
    if (Input_KeyDown(WKEY_S)) fmove -= 1.0f;
    if (Input_KeyDown(WKEY_D)) smove += 1.0f;
    if (Input_KeyDown(WKEY_A)) smove -= 1.0f;

    const float yawRad = DEG2RAD * playerYaw;
    const Vector3 forward = { cosf(yawRad), 0.0f, sinf(yawRad) };
    const Vector3 right = { -sinf(yawRad), 0.0f, cosf(yawRad) };

    wishVel = Vector3Add(Vector3Scale(forward, fmove), Vector3Scale(right, smove));

    float inputMag = Vector3Length(wishVel);
    wishDir = inputMag > 0.0f ? Vector3Scale(wishVel, 1.0f / inputMag) : Vector3Zero();
    wishSpeed = MAX_SPEED * fminf(1.0f, inputMag);
}

static void PM_Accelerate(Vector3 inWishDir, float inWishSpeed, float accel, float dt)
{
    float currentSpeed = Vector3DotProduct(velocity, inWishDir);
    float addSpeed = inWishSpeed - currentSpeed;
    if (addSpeed <= 0.0f)
        return;

    float accelSpeed = accel * inWishSpeed * dt;
    if (accelSpeed > addSpeed)
        accelSpeed = addSpeed;

    velocity = Vector3Add(velocity, Vector3Scale(inWishDir, accelSpeed));
}

static void PM_AirAccelerate(Vector3 inWishDir, float inWishSpeed, float accel, float dt)
{
    float wishspd = inWishSpeed;
    if (wishspd > AIR_WISH_SPEED_CAP)
        wishspd = AIR_WISH_SPEED_CAP;

    float currentSpeed = Vector3DotProduct(velocity, inWishDir);
    float addSpeed = wishspd - currentSpeed;
    if (addSpeed <= 0.0f)
        return;

    float accelSpeed = accel * inWishSpeed * dt;
    if (accelSpeed > addSpeed)
        accelSpeed = addSpeed;

    velocity = Vector3Add(velocity, Vector3Scale(inWishDir, accelSpeed));
}

static inline void PM_Jump(float dt)
{
    velocity.y = JUMP_FORCE;
    velocity.y -= GRAVITY * dt * 0.5f;
}

static void CategorizeGround(JPH::PhysicsSystem *ps, Vector3 &position, bool allowSnap)
{
    const bool wasGroundedState = isGrounded;

    if (!allowSnap && velocity.y > 0.0f)
    {
        isGrounded = false;
        gGroundNormal = JPH::Vec3::sZero();
        return;
    }

    const float probeDistance = allowSnap ? GROUND_PROBE_DISTANCE : GROUND_CONTACT_DISTANCE;
    GroundSupport support = FindGroundSupport(ps, position, probeDistance);

    isGrounded = false;
    gGroundNormal = JPH::Vec3::sZero();

    if (!support.found)
        return;

    isGrounded = true;
    gGroundNormal = ToJoltVec3(support.normal);

    if (allowSnap)
        position.y = support.centerY;

    const float targetGroundSpeed = wasGroundedState ? Vector3Length(velocity) : HorizontalLength(velocity);
    velocity = ReprojectVelocityPreserveSpeed(velocity, support.normal, targetGroundSpeed);
}

static void SlideMove(JPH::PhysicsSystem *ps, Vector3 &position, Vector3 &moveVelocity, float dt, bool preserveFallVelocityOnWalkableImpact = false, bool *landedOnWalkable = nullptr)
{
    float timeLeft = dt;
    Vector3 planes[MAX_CLIP_PLANES];
    int numPlanes = 0;
    Vector3 primalVelocity = moveVelocity;

    if (landedOnWalkable)
        *landedOnWalkable = false;

    for (int bump = 0; bump < MAX_BUMPS; ++bump)
    {
        if (Vector3LengthSq(moveVelocity) <= 1e-8f)
            break;

        MoveTrace trace = CastPlayerShape(ps, position, Vector3Scale(moveVelocity, timeLeft));

        if (trace.startSolid)
        {
            if (!ResolvePlayerPenetration(ps, position))
            {
                moveVelocity = Vector3Zero();
                break;
            }
            continue;
        }

        if (trace.fraction > 0.0f)
        {
            position = trace.endPos;
            primalVelocity = moveVelocity;
            numPlanes = 0;
        }

        if (!trace.hit || trace.fraction >= 1.0f)
            break;

        position = Vector3Add(position, Vector3Scale(trace.normal, POSITION_EPSILON));

        if (trace.fraction <= 0.0f && IsWalkableNormal(trace.normal))
        {
            continue;
        }

        if (preserveFallVelocityOnWalkableImpact && IsWalkableNormal(trace.normal) && primalVelocity.y < 0.0f)
        {
            Vector3 landingVelocity = primalVelocity;
            landingVelocity.y = 0.0f;
            moveVelocity = landingVelocity;
            if (landedOnWalkable)
                *landedOnWalkable = true;
            break;
        }

        bool duplicatePlane = false;
        for (int i = 0; i < numPlanes; ++i)
        {
            if (Vector3DotProduct(trace.normal, planes[i]) > DUPLICATE_PLANE_DOT)
            {
                moveVelocity = Vector3Add(moveVelocity, trace.normal);
                duplicatePlane = true;
                break;
            }
        }

        if (duplicatePlane)
            continue;

        if (numPlanes < MAX_CLIP_PLANES)
            planes[numPlanes++] = trace.normal;

        timeLeft *= fmaxf(0.0f, 1.0f - trace.fraction);

        Vector3 oldVelocity = moveVelocity;
        Vector3 clipped = Vector3Zero();
        bool found = false;

        for (int i = 0; i < numPlanes; ++i)
        {
            Vector3 candidate;
            PM_ClipVelocity(primalVelocity, planes[i], candidate, OVERCLIP);

            int j = 0;
            for (; j < numPlanes; ++j)
            {
                if (j != i && Vector3DotProduct(candidate, planes[j]) < -CLIP_PLANE_EPSILON)
                    break;
            }

            if (j == numPlanes)
            {
                clipped = candidate;
                found = true;
                break;
            }
        }

        if (!found)
        {
            if (numPlanes == 2)
            {
                Vector3 dir = Vector3CrossProduct(planes[0], planes[1]);
                float lenSq = Vector3LengthSq(dir);
                clipped = lenSq > 1e-8f ? Vector3Scale(dir, Vector3DotProduct(dir, moveVelocity) / lenSq) : Vector3Zero();
            }
            else
            {
                clipped = Vector3Zero();
            }
        }

        moveVelocity = clipped;
        if (Vector3DotProduct(moveVelocity, oldVelocity) <= 0.0f)
        {
            Vector3 recoveredVelocity;
            if (!RecoverTangentVelocity(planes, numPlanes, oldVelocity, recoveredVelocity))
            {
                moveVelocity = Vector3Zero();
                break;
            }

            moveVelocity = recoveredVelocity;
        }

        primalVelocity = moveVelocity;
    }
}

static void StepSlideMove(JPH::PhysicsSystem *ps, Vector3 &position, Vector3 &moveVelocity, float dt)
{
    Vector3 downPos = position;
    Vector3 downVel = moveVelocity;
    SlideMove(ps, downPos, downVel, dt, false, nullptr);

    MoveTrace upTrace = CastPlayerShape(ps, position, { 0.0f, STEP_HEIGHT, 0.0f });
    if (upTrace.hit && upTrace.fraction < 1.0f)
    {
        position = downPos;
        moveVelocity = downVel;
        return;
    }

    Vector3 upPos = upTrace.endPos;
    Vector3 upVel = moveVelocity;
    SlideMove(ps, upPos, upVel, dt, false, nullptr);

    MoveTrace downTrace = CastPlayerShape(ps, upPos, { 0.0f, -STEP_HEIGHT, 0.0f });
    if (downTrace.hit && IsWalkableNormal(downTrace.normal))
    {
        upPos = downTrace.endPos;
        if (upVel.y < 0.0f)
            upVel.y = 0.0f;
    }

    float downDist = HorizontalLengthSq(Vector3Subtract(downPos, position));
    float upDist = HorizontalLengthSq(Vector3Subtract(upPos, position));

    if (upDist > downDist)
    {
        position = upPos;
        moveVelocity = upVel;
    }
    else
    {
        position = downPos;
        moveVelocity = downVel;
    }
}

void InitJoltCharacter(Player *player, JPH::PhysicsSystem *physicsSystem)
{
    (void)physicsSystem;

    if (!gPlayerShape)
        gPlayerShape = new JPH::BoxShape(JPH::Vec3(PLAYER_RADIUS, PLAYER_HALF_HEIGHT, PLAYER_RADIUS), 0.0f);
    if (!gGroundProbeShape)
        gGroundProbeShape = new JPH::BoxShape(
            JPH::Vec3(PLAYER_RADIUS, GROUND_PROBE_HALF_THICKNESS, PLAYER_RADIUS),
            0.0f
        );

    ResolvePlayerPenetration(s_physics_system, player->center);
    CategorizeGround(s_physics_system, player->center, true);
}

void InitPlayer(Player *player, Vector3 center, Vector3 target, Vector3 up, float fovy)
{
    player->center = center;

    player->camera.position = { center.x, center.y + eyeOffset, center.z };
    player->camera.target = target;
    player->camera.up = up;
    player->camera.fovy = fovy;

    player->yaw = 0.0f;
    player->pitch = 0.0f;
    UpdateCameraTarget(player);

    player->speed = 320.0f;
    player->rotationSpeed = 90.0f;
    player->halfExt = { 16.0f, 28.0f, 16.0f };

    Input_LockMouse(true);
    cursorEnabled = false;
}

void UpdatePlayerMove(Player *player, JPH::PhysicsSystem *ps, float dt)
{
    ResolvePlayerPenetration(ps, player->center);

    float mdx = Input_MouseDeltaX();
    float mdy = Input_MouseDeltaY();
    playerYaw += mdx * MOUSE_SENSITIVITY;
    playerPitch -= mdy * MOUSE_SENSITIVITY;
    playerPitch = fmaxf(fminf(playerPitch, 89.0f), -89.0f);
    player->yaw = playerYaw;
    player->pitch = playerPitch;

    bool wasGrounded = isGrounded;
    CategorizeGround(ps, player->center, wasGrounded);

    PM_BuildWish();

    bool jumped = false;
    if (OnWalkableGround())
    {
        const Vector3 groundWishDir = ProjectDirectionOntoPlane(wishDir, GroundNormalVector());

        if (Input_KeyDown(WKEY_SPACE))
        {
            PM_Jump(dt);
            isGrounded = false;
            gGroundNormal = JPH::Vec3::sZero();
            jumped = true;
            PM_AirAccelerate(wishDir, wishSpeed, AIR_ACCELERATION, dt);
        }
        else
        {
            PM_Friction(dt);
            PM_Accelerate(groundWishDir, wishSpeed, ACCELERATION, dt);
        }
    }
    else
    {
        PM_AirAccelerate(wishDir, wishSpeed, AIR_ACCELERATION, dt);
        velocity.y -= GRAVITY * dt;
    }

    bool landedOnWalkable = false;

    if (OnWalkableGround())
    {
        Vector3 moveVelocity = velocity;
        PM_ClipVelocity(moveVelocity, GroundNormalVector(), moveVelocity, OVERCLIP);
        StepSlideMove(ps, player->center, moveVelocity, dt);
        velocity = moveVelocity;
    }
    else
        SlideMove(ps, player->center, velocity, dt, true, &landedOnWalkable);

    ResolvePlayerPenetration(ps, player->center);
    CategorizeGround(ps, player->center, landedOnWalkable || (wasGrounded && !jumped));
    PM_ApplyWalkableSlopeStop();

    DebugUpdateVelMetrics();

    player->camera.position = { player->center.x, player->center.y + eyeOffset, player->center.z };
    float yawRad = DEG2RAD * playerYaw;
    float pitchRad = DEG2RAD * playerPitch;
    Vector3 fwd = { cosf(pitchRad) * cosf(yawRad), sinf(pitchRad), cosf(pitchRad) * sinf(yawRad) };
    player->camera.target = Vector3Add(player->camera.position, fwd);
}

void UpdatePlayer(Player *player, JPH::PhysicsSystem *mPhysicsSystem, float deltaTime)
{
    (void)mPhysicsSystem;

    float mdx = Input_MouseDeltaX();
    float mdy = Input_MouseDeltaY();

    player->yaw += mdx * MOUSE_SENSITIVITY;
    player->pitch -= mdy * MOUSE_SENSITIVITY;

    if (player->pitch > 89.0f) player->pitch = 89.0f;
    if (player->pitch < -89.0f) player->pitch = -89.0f;

    UpdateCameraTarget(player);

    Vector3 direction = { 0.0f, 0.0f, 0.0f };
    Vector3 forward = Vector3Normalize(Vector3Subtract(player->camera.target, player->camera.position));
    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, player->camera.up));

    if (Input_KeyDown(WKEY_W)) direction = Vector3Add(direction, forward);
    if (Input_KeyDown(WKEY_S)) direction = Vector3Subtract(direction, forward);
    if (Input_KeyDown(WKEY_A)) direction = Vector3Subtract(direction, right);
    if (Input_KeyDown(WKEY_D)) direction = Vector3Add(direction, right);
    if (Input_KeyDown(WKEY_SPACE)) direction = Vector3Add(direction, player->camera.up);
    if (Input_KeyDown(WKEY_LEFT_SHIFT)) direction = Vector3Subtract(direction, player->camera.up);

    if (Input_KeyPressed(WKEY_M) && !cursorEnabled)
    {
        Input_LockMouse(false);
        cursorEnabled = true;
    }
    else if (Input_KeyPressed(WKEY_M) && cursorEnabled)
    {
        Input_LockMouse(true);
        cursorEnabled = false;
    }

    if (Vector3LengthSq(direction) > 0.0f)
        direction = Vector3Normalize(direction);

    player->center = Vector3Add(player->center, Vector3Scale(direction, player->speed * deltaTime));
    player->camera.position = { player->center.x, player->center.y + eyeOffset, player->center.z };
    player->camera.target = Vector3Add(player->camera.position, direction);
}

void UpdateCameraTarget(Player *player)
{
    float yawRad = player->yaw * DEG2RAD;
    float pitchRad = player->pitch * DEG2RAD;

    Vector3 direction;
    direction.x = cosf(pitchRad) * cosf(yawRad);
    direction.y = sinf(pitchRad);
    direction.z = cosf(pitchRad) * sinf(yawRad);

    direction = Vector3Normalize(direction);
    player->camera.target = Vector3Add(player->camera.position, direction);
}

void DebugDrawPlayerAABB(Player *player)
{
    Debug_WireBox(player->center, player->halfExt, C_RED);
}

void DebugDrawGroundProbe(void)
{
    if (!gGroundProbeDebug.valid)
        return;

    const Vector3 probeHalfExt = { PLAYER_RADIUS, GROUND_PROBE_HALF_THICKNESS, PLAYER_RADIUS };
    const Color probeStartCol = WCOLOR(120, 180, 255, 255);
    const Color probeEndCol = WCOLOR(70, 110, 180, 255);

    Debug_WireBox(gGroundProbeDebug.startCenter, probeHalfExt, probeStartCol);
    Debug_WireBox(gGroundProbeDebug.endCenter, probeHalfExt, probeEndCol);
    Debug_Line(gGroundProbeDebug.startCenter, gGroundProbeDebug.endCenter, probeStartCol);

    if (!gGroundProbeDebug.hadHit)
        return;

    Debug_WireBox(gGroundProbeDebug.hitCenter, probeHalfExt, C_GREEN);
    Debug_Line(
        gGroundProbeDebug.hitCenter,
        Vector3Add(gGroundProbeDebug.hitCenter, Vector3Scale(gGroundProbeDebug.hitNormal, 12.0f)),
        C_YELLOW
    );
}

void DebugDir(Player *player)
{
    Vector3 start = player->center;

    float wishDirLen = Vector3Length(wishDir);
    if (wishDirLen > 0.0001f)
    {
        Vector3 wishEnd = Vector3Add(start, wishDir * 50.0f);
        Debug_Line(start, wishEnd, C_GREEN);
        Debug_Point(wishEnd, 4.0f, C_LIME);
    }
    else
    {
        Debug_Point(start, 6.0f, C_YELLOW);
    }

    float wishVelLen = Vector3Length(wishVel);
    if (wishVelLen > 0.0001f)
    {
        Vector3 wishVelEnd = Vector3Add(start, wishVel);
        Debug_Line(start, wishVelEnd, C_ORANGE);
        Debug_Point(wishVelEnd, 4.0f, C_GOLD);
    }

    Vector3 horizVelocity = velocity;
    horizVelocity.y = 0.0f;
    float horizMag = Vector3Length(horizVelocity);

    if (horizMag > 0.0001f)
    {
        Vector3 velEnd = Vector3Add(start, horizVelocity);
        Debug_Line(start, velEnd, C_BLUE);
        Debug_Point(velEnd, 4.0f, C_DBLUE);
    }
    else
    {
        Debug_Point(start, 6.0f, C_PURPLE);
    }
}

void DebugDrawPlayerPos(const Player *player, int col, int row)
{
    sdtx_pos((float)col, (float)row);
    sdtx_color3b(230, 41, 55);
    sdtx_printf("Player Pos: X=%.2f  Y=%.2f  Z=%.2f",
                player->center.x, player->center.y, player->center.z);
}

void DebugDrawPlayerVel(int col, int row)
{
    float dh = g_velDbg.horiz - g_velDbg.lastH;

    if      (dh >  0.1f) sdtx_color3b(  0,228, 48);
    else if (dh < -0.1f) sdtx_color3b(230, 41, 55);
    else                 sdtx_color3b(200,200,200);

    sdtx_pos((float)col, (float)row);
    sdtx_printf("H:%6.2f  V:%6.2f  |v|:%6.2f  PeakH:%6.2f  %s",
                g_velDbg.horiz, g_velDbg.vert, g_velDbg.total, g_velDbg.peakH,
                isGrounded ? "GROUND" : "AIR");

    const float maxShow = 600.0f;
    int cells = (int)(30.0f * Clamp01(g_velDbg.horiz / maxShow));
    sdtx_pos((float)col, (float)row + 1.0f);
    sdtx_color3b(0, 121, 241);
    sdtx_putc('[');
    for (int i = 0; i < 30; ++i) sdtx_putc(i < cells ? '=' : ' ');
    sdtx_putc(']');

    g_velDbg.lastH = g_velDbg.horiz;
}
