#include "physics.h"
#include "collision_data.h"
#include "../compiler/map_geometry.h"
#include "../entities/entities.h"
#include "../render/debug_draw.h"

#include "box3d/box3d.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

/* Here we will setup all Box3D physics functions for collisions with the world.
 *  Theoretical structure:
 *      - The worldspawn layer in TB will contain all of our brushes, brushes
 *        inside the worldspawn layer that arent entities (besides clip brushes and triggers), will be
 *        calculated as convex hulls for applying the minkowski difference and GJK algorithm
 *        for collision detection. This is heavily inspired by Quake's collision system.
 *
 *        NOTE: See "Real-Time Collision Detection by Christer Ericson" page 400, Ch. 9.5.
 *              This chapter reviews many convex collision detection algorithms, the section
 *              specified defines the Gilbert-Johnson-Keerthi algorithm (GJK).
 *
 *      - Box3D is a C API built around convex hulls, so the parsed brush point clouds feed
 *        directly into b3CreateHull. Triggers are static sensor shapes, the player never has
 *        a body and instead queries the world with casts and overlaps similar to Quake and GoldSrc.
 * */

//--------------------------------------//
// Global or static variables
//--------------------------------------//
b3WorldId g_physicsWorld = B3_NULL_ID;
b3BodyId  debugSphereID  = B3_NULL_ID;

namespace {

static constexpr float PHYSICS_MAP_UNITS_PER_METER = 64.0f;
static constexpr float PHYSICS_DENSITY_SCALE =
    1.0f / (PHYSICS_MAP_UNITS_PER_METER * PHYSICS_MAP_UNITS_PER_METER * PHYSICS_MAP_UNITS_PER_METER);

struct DynamicPhysicsRenderHull {
    b3BodyId bodyId = B3_NULL_ID;
    std::vector<std::array<b3Vec3, 2>> edges;
    std::vector<std::array<b3Vec3, 3>> triangles;
};

std::vector<DynamicPhysicsRenderHull> sDynamicRenderHulls;
std::vector<b3BodyId> sMapHullBodies;
std::unordered_map<uint64_t, bool> sCanPlayerPushBody;

static bool GetEntityProp(const std::vector<Entity>& entities,
                          const MeshCollisionData& mcd,
                          const char* key,
                          std::string& out)
{
    if (mcd.entityIndex < 0 || (size_t)mcd.entityIndex >= entities.size()) {
        return false;
    }

    const Entity& entity = entities[(size_t)mcd.entityIndex];
    auto it = entity.properties.find(key);
    if (it == entity.properties.end()) {
        return false;
    }

    out = it->second;
    return true;
}

static const char* GetEntityClassname(const std::vector<Entity>& entities, const MeshCollisionData& mcd)
{
    if (mcd.entityIndex < 0 || (size_t)mcd.entityIndex >= entities.size()) {
        return "?";
    }

    const Entity& entity = entities[(size_t)mcd.entityIndex];
    auto it = entity.properties.find("classname");
    return it != entity.properties.end() ? it->second.c_str() : "?";
}

static bool ParseFloatProperty(const std::vector<Entity>& entities,
                               const MeshCollisionData& mcd,
                               const char* key,
                               float& out)
{
    std::string value;
    if (!GetEntityProp(entities, mcd, key, value)) {
        return false;
    }

    char* end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0') {
        printf("[physics] %s entity=%d has invalid %s='%s'\n",
               GetEntityClassname(entities, mcd),
               mcd.entityIndex,
               key,
               value.c_str());
        return false;
    }

    out = parsed;
    return true;
}

static bool ParseIntProperty(const std::vector<Entity>& entities,
                             const MeshCollisionData& mcd,
                             const char* key,
                             int& out)
{
    float parsed = 0.0f;
    if (!ParseFloatProperty(entities, mcd, key, parsed)) {
        return false;
    }

    out = (int)parsed;
    return true;
}

static bool ParseU64Property(const std::vector<Entity>& entities,
                             const MeshCollisionData& mcd,
                             const char* key,
                             uint64_t& out)
{
    std::string value;
    if (!GetEntityProp(entities, mcd, key, value)) {
        return false;
    }

    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(value.c_str(), &end, 0);
    if (end == value.c_str() || *end != '\0') {
        printf("[physics] %s entity=%d has invalid %s='%s'\n",
               GetEntityClassname(entities, mcd),
               mcd.entityIndex,
               key,
               value.c_str());
        return false;
    }

    out = (uint64_t)parsed;
    return true;
}

static bool ParseBoolProperty(const std::vector<Entity>& entities,
                              const MeshCollisionData& mcd,
                              const char* key,
                              bool& out)
{
    std::string value;
    if (!GetEntityProp(entities, mcd, key, value)) {
        return false;
    }

    if (value == "1" || value == "true" || value == "True" || value == "yes" || value == "on") {
        out = true;
        return true;
    }

    if (value == "0" || value == "false" || value == "False" || value == "no" || value == "off") {
        out = false;
        return true;
    }

    printf("[physics] %s entity=%d has invalid %s='%s'\n",
           GetEntityClassname(entities, mcd),
           mcd.entityIndex,
           key,
           value.c_str());
    return false;
}

static bool ParseVec3Property(const std::vector<Entity>& entities,
                              const MeshCollisionData& mcd,
                              const char* key,
                              Vector3& out)
{
    std::string value;
    if (!GetEntityProp(entities, mcd, key, value)) {
        return false;
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (std::sscanf(value.c_str(), "%f %f %f", &x, &y, &z) != 3) {
        printf("[physics] %s entity=%d has invalid %s='%s'\n",
               GetEntityClassname(entities, mcd),
               mcd.entityIndex,
               key,
               value.c_str());
        return false;
    }

    out = { x, y, z };
    return true;
}

static b3Vec3 ToB3Vec3(Vector3 v)
{
    return b3Vec3{ v.x, v.y, v.z };
}

static Vector3 TransformLocalPoint(b3WorldTransform transform, b3Vec3 local)
{
    b3Vec3 rotated = b3RotateVector(transform.q, local);
    return Vector3{
        (float)transform.p.x + rotated.x,
        (float)transform.p.y + rotated.y,
        (float)transform.p.z + rotated.z
    };
}

static Vector3 ComputePointCenter(const std::vector<Vector3>& points)
{
    Vector3 minPoint = points[0];
    Vector3 maxPoint = points[0];
    for (const Vector3& point : points) {
        minPoint.x = std::min(minPoint.x, point.x);
        minPoint.y = std::min(minPoint.y, point.y);
        minPoint.z = std::min(minPoint.z, point.z);
        maxPoint.x = std::max(maxPoint.x, point.x);
        maxPoint.y = std::max(maxPoint.y, point.y);
        maxPoint.z = std::max(maxPoint.z, point.z);
    }

    return Vector3Scale(Vector3Add(minPoint, maxPoint), 0.5f);
}

static b3BodyType ReadBodyType(const std::vector<Entity>& entities,
                               const MeshCollisionData& mcd,
                               b3BodyType defaultType)
{
    std::string value;
    if (!GetEntityProp(entities, mcd, "body_type", value)) {
        return defaultType;
    }

    if (value == "2" || value == "static") {
        return b3_staticBody;
    }
    if (value == "1" || value == "kinematic") {
        return b3_kinematicBody;
    }
    if (value == "0" || value == "dynamic") {
        return b3_dynamicBody;
    }

    printf("[physics] %s entity=%d has invalid body_type='%s'\n",
           GetEntityClassname(entities, mcd),
           mcd.entityIndex,
           value.c_str());
    return defaultType;
}

static void ApplyPhysicsEntityProperties(const std::vector<Entity>& entities,
                                         const MeshCollisionData& mcd,
                                         b3BodyDef& bodyDef,
                                         b3ShapeDef& shapeDef)
{
    float f = 0.0f;
    int i = 0;
    bool b = false;
    uint64_t u64 = 0;
    Vector3 vec{};

    if (ParseVec3Property(entities, mcd, "velocity", vec) ||
        ParseVec3Property(entities, mcd, "linear_velocity", vec)) {
        bodyDef.linearVelocity = ToB3Vec3(ConvertTBPointEntityToWorld(vec));
    }
    if (ParseVec3Property(entities, mcd, "angular_velocity", vec)) {
        bodyDef.angularVelocity = ToB3Vec3(ConvertTBPointEntityToWorld(vec));
    }

    if (ParseFloatProperty(entities, mcd, "linear_damping", f)) {
        bodyDef.linearDamping = std::max(0.0f, f);
    }
    if (ParseFloatProperty(entities, mcd, "angular_damping", f)) {
        bodyDef.angularDamping = std::max(0.0f, f);
    }
    if (ParseFloatProperty(entities, mcd, "gravity_scale", f)) {
        bodyDef.gravityScale = f;
    }
    if (ParseFloatProperty(entities, mcd, "sleep_threshold", f)) {
        bodyDef.sleepThreshold = std::max(0.0f, f);
    }

    if (ParseBoolProperty(entities, mcd, "enable_sleep", b)) bodyDef.enableSleep = b;
    if (ParseBoolProperty(entities, mcd, "start_awake", b)) bodyDef.isAwake = b;
    if (ParseBoolProperty(entities, mcd, "enabled", b)) bodyDef.isEnabled = b;
    if (ParseBoolProperty(entities, mcd, "bullet", b)) bodyDef.isBullet = b;
    if (ParseBoolProperty(entities, mcd, "allow_fast_rotation", b)) bodyDef.allowFastRotation = b;
    if (ParseBoolProperty(entities, mcd, "contact_recycling", b)) bodyDef.enableContactRecycling = b;

    if (ParseBoolProperty(entities, mcd, "lock_linear_x", b)) bodyDef.motionLocks.linearX = b;
    if (ParseBoolProperty(entities, mcd, "lock_linear_y", b)) bodyDef.motionLocks.linearY = b;
    if (ParseBoolProperty(entities, mcd, "lock_linear_z", b)) bodyDef.motionLocks.linearZ = b;
    if (ParseBoolProperty(entities, mcd, "lock_angular_x", b)) bodyDef.motionLocks.angularX = b;
    if (ParseBoolProperty(entities, mcd, "lock_angular_y", b)) bodyDef.motionLocks.angularY = b;
    if (ParseBoolProperty(entities, mcd, "lock_angular_z", b)) bodyDef.motionLocks.angularZ = b;

    if (ParseFloatProperty(entities, mcd, "density", f)) {
        shapeDef.density = std::max(0.0f, f);
    }
    if (ParseFloatProperty(entities, mcd, "friction", f)) {
        shapeDef.baseMaterial.friction = std::max(0.0f, f);
    }
    if (ParseFloatProperty(entities, mcd, "restitution", f)) {
        shapeDef.baseMaterial.restitution = std::max(0.0f, f);
    }
    if (ParseFloatProperty(entities, mcd, "rolling_resistance", f)) {
        shapeDef.baseMaterial.rollingResistance = std::max(0.0f, f);
    }
    if (ParseFloatProperty(entities, mcd, "explosion_scale", f)) {
        shapeDef.explosionScale = std::max(0.0f, f);
    }
    if (ParseVec3Property(entities, mcd, "tangent_velocity", vec)) {
        shapeDef.baseMaterial.tangentVelocity = ToB3Vec3(ConvertTBPointEntityToWorld(vec));
    }

    if (ParseBoolProperty(entities, mcd, "sensor", b)) shapeDef.isSensor = b;
    if (ParseBoolProperty(entities, mcd, "sensor_events", b)) shapeDef.enableSensorEvents = b;
    if (ParseBoolProperty(entities, mcd, "contact_events", b)) shapeDef.enableContactEvents = b;
    if (ParseBoolProperty(entities, mcd, "hit_events", b)) shapeDef.enableHitEvents = b;
    if (ParseBoolProperty(entities, mcd, "presolve_events", b)) shapeDef.enablePreSolveEvents = b;
    if (ParseBoolProperty(entities, mcd, "invoke_contact_creation", b)) shapeDef.invokeContactCreation = b;
    if (ParseBoolProperty(entities, mcd, "custom_filtering", b)) shapeDef.enableCustomFiltering = b;
    if (ParseBoolProperty(entities, mcd, "update_body_mass", b)) shapeDef.updateBodyMass = b;

    if (ParseIntProperty(entities, mcd, "group_index", i)) {
        shapeDef.filter.groupIndex = i;
    }
    if (ParseU64Property(entities, mcd, "material_id", u64)) {
        shapeDef.baseMaterial.userMaterialId = u64;
    }
    if (ParseU64Property(entities, mcd, "custom_color", u64)) {
        shapeDef.baseMaterial.customColor = (uint32_t)u64;
    }
    if (ParseU64Property(entities, mcd, "category_bits", u64)) {
        shapeDef.filter.categoryBits = u64;
    }
    if (ParseU64Property(entities, mcd, "mask_bits", u64)) {
        shapeDef.filter.maskBits = u64;
    }

    shapeDef.density *= PHYSICS_DENSITY_SCALE;
}

static void ApplyMassOverride(const std::vector<Entity>& entities,
                              const MeshCollisionData& mcd,
                              const b3HullData* hull,
                              const b3ShapeDef& shapeDef,
                              b3BodyId body)
{
    float mass = 0.0f;
    if (!ParseFloatProperty(entities, mcd, "mass", mass) || mass <= 0.0f) {
        return;
    }

    b3MassData massData = b3ComputeHullMass(hull, shapeDef.density);
    if (massData.mass <= 0.0f) {
        printf("[physics] %s entity=%d mass override ignored because computed mass is zero\n",
               GetEntityClassname(entities, mcd),
               mcd.entityIndex);
        return;
    }

    const float scale = mass / massData.mass;
    massData.mass = mass;
    massData.inertia = b3MulSM(scale, massData.inertia);
    b3Body_SetMassData(body, massData);
}

static void AddDynamicRenderHull(b3BodyId bodyId, const b3HullData* hull)
{
    const b3Vec3* points = b3GetHullPoints(hull);
    const b3HullHalfEdge* edges = b3GetHullEdges(hull);
    const b3HullFace* faces = b3GetHullFaces(hull);
    if (points == nullptr || edges == nullptr || faces == nullptr) {
        return;
    }

    DynamicPhysicsRenderHull renderHull;
    renderHull.bodyId = bodyId;
    renderHull.edges.reserve((size_t)hull->edgeCount / 2);

    for (int edgeIndex = 0; edgeIndex < hull->edgeCount; ++edgeIndex) {
        const b3HullHalfEdge& edge = edges[edgeIndex];
        if (edgeIndex > (int)edge.twin) {
            continue;
        }

        const b3HullHalfEdge& twin = edges[edge.twin];
        renderHull.edges.push_back({ points[edge.origin], points[twin.origin] });
    }

    for (int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex) {
        std::vector<b3Vec3> facePoints;
        uint8_t edgeIndex = faces[faceIndex].edge;
        const uint8_t startEdge = edgeIndex;
        do {
            const b3HullHalfEdge& edge = edges[edgeIndex];
            facePoints.push_back(points[edge.origin]);
            edgeIndex = edge.next;
        } while (edgeIndex != startEdge && facePoints.size() <= (size_t)hull->edgeCount);

        if (facePoints.size() < 3) {
            continue;
        }

        for (size_t i = 1; i + 1 < facePoints.size(); ++i) {
            renderHull.triangles.push_back({ facePoints[0], facePoints[i], facePoints[i + 1] });
        }
    }

    sDynamicRenderHulls.push_back(std::move(renderHull));
}

} // namespace

//--------------------------------------//
// Implementation
//--------------------------------------//

void InitPhysicsSystem()
{
    if (b3World_IsValid(g_physicsWorld))
        return;

    // Length units are left at the default (1.0). The player controller keeps
    // POSITION_EPSILON (0.02) of separation from surfaces, which must stay
    // above Box3D's linear slope (0.005 * lengthUnits) or casts that start near
    // a surface report an initial overlap with a zero normal.
    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.gravity = b3Vec3{ 0.0f, -800.1f, 0.0f };

    const unsigned hw = std::thread::hardware_concurrency();
    worldDef.workerCount = std::clamp(hw > 1 ? hw - 1 : 1u, 1u, (unsigned)B3_MAX_WORKERS);

    g_physicsWorld = b3CreateWorld(&worldDef);

    printf("[InitPhysicsSystem] Box3D setup complete\n");
}

void ShutdownPhysicsSystem()
{
    if (!b3World_IsValid(g_physicsWorld))
        return;

    b3DestroyWorld(g_physicsWorld);
    g_physicsWorld = B3_NULL_ID;
    debugSphereID  = B3_NULL_ID;
    sDynamicRenderHulls.clear();
    sMapHullBodies.clear();
    sCanPlayerPushBody.clear();

    printf("[ShutdownPhysicsSystem] Freed Box3D resources.\n");
}

void UpdatePhysicsSystem(float delta_time)
{
    if (!b3World_IsValid(g_physicsWorld))
        return;

    const int subStepCount = 12;
    b3World_Step(g_physicsWorld, delta_time, subStepCount);
    GameplayEntities::UpdateDynamicBodyEffects(delta_time);

    // Trigger volumes are sensor shapes, dynamic bodies entering them show up here.
    b3SensorEvents sensorEvents = b3World_GetSensorEvents(g_physicsWorld);
    for (int i = 0; i < sensorEvents.beginCount; ++i) {
        printf("\n Trigger Activated\n");
    }

    b3BodyEvents bodyEvents = b3World_GetBodyEvents(g_physicsWorld);
    for (int i = 0; i < bodyEvents.moveCount; ++i) {
        if (bodyEvents.moveEvents[i].fellAsleep) {
            printf("A body went to sleep\n");
        }
    }
}

void BuildMapPhysics(const std::vector<MeshCollisionData> &meshCollisionData,
                     const std::vector<Entity> &entities)
{
    int count = 0;
    sDynamicRenderHulls.clear();
    sCanPlayerPushBody.clear();
    sMapHullBodies.assign(meshCollisionData.size(), B3_NULL_ID);
    GameplayEntities::Reset();
    GameplayEntities::RegisterPointEntities(entities);

    for (size_t hullIndex = 0; hullIndex < meshCollisionData.size(); ++hullIndex) {
        const MeshCollisionData& mcd = meshCollisionData[hullIndex];
        // If NO_COLLIDE or something similar, we skip
        if (mcd.collisionType == CollisionType::NO_COLLIDE) {
            continue;
        }

        Vector3 bodyCenter = ComputePointCenter(mcd.vertices);

        // Convert engine vectors to Box3D local-space points.
        std::vector<b3Vec3> points;
        points.reserve(mcd.vertices.size());
        for (auto &v : mcd.vertices) {
            Vector3 local = Vector3Subtract(v, bodyCenter);
            points.push_back(b3Vec3{ local.x, local.y, local.z });
        }

        if (points.size() < 4) {
            printf("Skipping brush with too few points for a hull (%zu)\n", points.size());
            continue;
        }

        // Build a convex hull from these points
        b3HullData *hull = b3CreateHull(points.data(), (int)points.size(), (int)points.size());
        if (hull == nullptr) {
            printf("Error building hull shape from %zu points\n", points.size());
            continue;
        }

        // Decide body type and filter bits from collisionType
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3ShapeDef shapeDef = b3DefaultShapeDef();

        switch (mcd.collisionType) {
            case CollisionType::STATIC: {
                bodyDef.type = b3_staticBody;
                shapeDef.filter.categoryBits = Layers::STATIC;
                shapeDef.filter.maskBits = Layers::MOVING;
                break;
            }
            case CollisionType::TRIGGER: {
                // Static sensor volume: generates overlap events but no collision response.
                bodyDef.type = b3_staticBody;
                shapeDef.filter.categoryBits = Layers::SENSOR;
                shapeDef.filter.maskBits = Layers::MOVING;
                shapeDef.isSensor = true;
                shapeDef.enableSensorEvents = true;
                printf("\n SETTING IS SENSOR TO TRUE \n");
                break;
            }
            case CollisionType::DYNAMIC: {
                bodyDef.type = ReadBodyType(entities, mcd, b3_dynamicBody);
                shapeDef.filter.categoryBits = Layers::MOVING;
                shapeDef.filter.maskBits = Layers::STATIC | Layers::MOVING | Layers::SENSOR;
                shapeDef.enableSensorEvents = true;
                ApplyPhysicsEntityProperties(entities, mcd, bodyDef, shapeDef);
                break;
            }
            // NO_COLLIDE or UNKNOWN => skip
            default: {
                b3DestroyHull(hull);
                continue;
            }
        }

        bodyDef.position = b3Pos{ bodyCenter.x, bodyCenter.y, bodyCenter.z };
        b3BodyId body = b3CreateBody(g_physicsWorld, &bodyDef);
        if (B3_IS_NULL(body)) {
            printf("Failed to create body for a brush\n");
            b3DestroyHull(hull);
            continue;
        }

        b3ShapeId shape = b3CreateHullShape(body, &shapeDef, hull);

        if (B3_IS_NULL(shape)) {
            printf("Failed to create hull shape for a brush\n");
            b3DestroyBody(body);
            b3DestroyHull(hull);
            continue;
        }

        sMapHullBodies[hullIndex] = body;

        if (mcd.collisionType == CollisionType::DYNAMIC) {
            bool canPush = true;
            ParseBoolProperty(entities, mcd, "can_push", canPush);
            sCanPlayerPushBody[b3StoreBodyId(body)] = canPush;

            ApplyMassOverride(entities, mcd, hull, shapeDef, body);
            AddDynamicRenderHull(body, hull);
        }
        b3DestroyHull(hull); // the world interns hull data in its hull database

        if (mcd.entityIndex >= 0 && (size_t)mcd.entityIndex < entities.size()) {
            GameplayEntities::RegisterBrushEntity(entities[(size_t)mcd.entityIndex], mcd.entityIndex, body);
        }
        ++count;
    }

    printf("\n\n %d MAP COLLISIONS SUCCESSFULLY CREATED \n\n", count);
}

b3BodyId GetMapPhysicsBodyForHull(size_t hullIndex)
{
    if (hullIndex >= sMapHullBodies.size()) {
        return B3_NULL_ID;
    }
    return sMapHullBodies[hullIndex];
}

bool CanPlayerPushBody(b3BodyId bodyId)
{
    if (!b3Body_IsValid(bodyId)) {
        return false;
    }

    const auto it = sCanPlayerPushBody.find(b3StoreBodyId(bodyId));
    if (it == sCanPlayerPushBody.end()) {
        return true;
    }

    return it->second;
}

void DebugDrawPhysicsObjects()
{
    const Color fillColor = WCOLOR(70, 100, 220, 180);
    const Color edgeColor = WCOLOR(220, 230, 255, 255);

    for (const DynamicPhysicsRenderHull& renderHull : sDynamicRenderHulls) {
        if (!b3Body_IsValid(renderHull.bodyId)) {
            continue;
        }

        const b3WorldTransform transform = b3Body_GetTransform(renderHull.bodyId);
        for (const auto& tri : renderHull.triangles) {
            Debug_Triangle(TransformLocalPoint(transform, tri[0]),
                           TransformLocalPoint(transform, tri[1]),
                           TransformLocalPoint(transform, tri[2]),
                           fillColor);
        }

        for (const auto& edge : renderHull.edges) {
            Debug_Line(TransformLocalPoint(transform, edge[0]),
                       TransformLocalPoint(transform, edge[1]),
                       edgeColor);
        }
    }
}
