#include "physics.h"
#include "Jolt/Core/Core.h"
#include "Jolt/Core/JobSystem.h"
#include "Jolt/Geometry/Sphere.h"
#include "Jolt/Math/MathTypes.h"
#include "Jolt/Math/Real.h"
#include "Jolt/Math/Vec3.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Body/BodyID.h"
#include "Jolt/Physics/Body/BodyInterface.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "collision_data.h"

#include "Jolt/Jolt.h"

#define JPH_ENABLE_ASSERTS

// Jolt includes
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

#include "Jolt/Math/Float3.h"
#include "Jolt/Physics/Body/MotionType.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/Shape/ConvexShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Geometry/ConvexSupport.h"
#include "Jolt/Geometry/GJKClosestPoint.h"

#include <iostream>
#include <thread>

/* Here we will setup all Jolt Physics functions for collisions with the world.
 *  Theoretical structure:
 *      - The worldspawn layer in TB will contain all of our brushes, brushes
 *        inside the worldspawn layer that arent entities (besides clip brushes and triggers), will be
 *        calculated as "Convex Shapes" for applying the minkowski difference and GJK algorithm
 *        for collision detection. This is heavily inspired by Quake's collision system
 *        which has stood the test of time.
 *
 *        NOTE: See "Real-Time Collision Detection by Christer Ericson" page 400, Ch. 9.5.
 *              This chapter reviews many convex collision detection algorithms, the section
 *              specified defines the Gilbert-Johnson-Keerthi algorithm (GJK).
 *
 *      - Other brush entites like triggers, physics objects, etc.. will need to be handled
 *        differently. Potentially in a similar way to world brushes but there isn't much
 *        documentation or examples on convex collisions specific to the use case I intend
 *        for this engine. A lot of this will be trial and error when wrangling the custom
 *        mesh building code into something usable within Jolt's context.
 *
 *        NOTE: Maybe the polygon code doesn't need to be used, just the planes themselves? 
 *              I believe this would save a ton of compute if possible.
 *
 *        TODO: For entities that will live on different physics layers, we must assign the layers when parsing
 *  
 *      - Unfortunately Jolt is written in OOP style, it is still a feature rich performant library though.
 *        Maybe in the future I will write a custom DOD style physics implementation for this specific engine.
 *        For now though I will bite the OOP bullet and implement some class hierarchies in this file. All of *MY*
 *        code will avoid this whenever possible.              
 * */

/* Post implementation of initial physics system: 
 *      - Implementing Jolt's systems here to be used in main.cpp was relatively easy. Following the HelloWorld.cpp
 *        Jolt example the implementation was pretty straight forward. A few things needed to be changed but it worked
 *        out in the end.
 *      
 *      - However there must be further work done on checking the resulting collision hull, currently it works rather
 *        well but a wireframe draw implementation will be highly effective in checking if the hulls are being created
 *        correctly. Since there is alot of operations done on the parsed geometry from the .map file we need to ensure
 *        the vertices are being read properly from the point cloud.
 *
 *  TODO: *DONE* Implement trigger volumes using Jolt's sensor system. I believe this can be achieved by reading the trigger
 *        entity data from the map parser and assigning it's geometry as a collision hull but without uploading it to
 *        the GPU since we don't want to draw it and cause any artifacts. *DONE*
 * */

static void TraceImpl(const char *inFMT, ...)
{
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);
    std::cout << buffer << std::endl;
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
    std::cout << inFile << ":" << inLine << ": (" << inExpression << ") " 
              << (inMessage ? inMessage : "") << std::endl;
    return true; // return true to break into debugger
}
#endif

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl()
    {
        m_object_to_broadphase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        m_object_to_broadphase[Layers::MOVING]     = BroadPhaseLayers::MOVING;
        m_object_to_broadphase[Layers::SENSOR]     = BroadPhaseLayers::SENSOR;
    }

    virtual JPH::uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::NUM_LAYERS; // 2
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
    {
        // inLayer must be < Layers::NUM_LAYERS
        return m_object_to_broadphase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    // debugging
    virtual const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        switch ((JPH::BroadPhaseLayer::Type)inLayer)
        {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:  return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:      return "MOVING";
            default: return "UNKNOWN";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer m_object_to_broadphase[Layers::NUM_LAYERS];
};

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
    {
        // Example:
        // Non-moving only collides with moving
        if (inObject1 == Layers::NON_MOVING && inObject2 == Layers::NON_MOVING) return false;
        if (inObject1 == Layers::SENSOR && inObject2 == Layers::SENSOR) return false;
        if (inObject1 == Layers::SENSOR && inObject2 == Layers::NON_MOVING) return false;
        // Otherwise everything else collides
        return true;
    }
};

class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer, JPH::BroadPhaseLayer inBroadPhaseLayer) const override
    {
        // Non-moving only collides with moving
        if (inLayer == Layers::NON_MOVING && inBroadPhaseLayer == BroadPhaseLayers::NON_MOVING)
            return false;
        // Otherwise collide
        return true;
    }
};

class MyContactListener : public JPH::ContactListener
{
public:
    virtual JPH::ValidateResult OnContactValidate(const JPH::Body &inBody1, const JPH::Body &inBody2, 
                                                  JPH::RVec3Arg inBaseOffset, 
                                                  const JPH::CollideShapeResult &inCollisionResult) override
    {
        // Let all collisions happen
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    virtual void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, 
                                const JPH::ContactManifold &inManifold, 
                                JPH::ContactSettings &ioSettings) override
    {
        // Called when new contact begins
        if ((inBody1.GetObjectLayer() == Layers::SENSOR)|| (inBody2.GetObjectLayer() == Layers::SENSOR)) {
            printf("\n TRIGGER ACTIVATED!\n");
        } 
    }

    virtual void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, 
                                    const JPH::ContactManifold &inManifold,
                                    JPH::ContactSettings &ioSettings) override
    {
        // Called every frame contact persists
        if ((inBody1.GetObjectLayer() == Layers::SENSOR) || (inBody2.GetObjectLayer() == Layers::SENSOR)) {
            //printf("TRIGGER ACTIVATED!!!\n");
        }
    }

    virtual void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
    {
        // Called when contact stops
    }
};

class MyBodyActivationListener : public JPH::BodyActivationListener
{
public:
	virtual void		OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
            std::cout << "A body got activated" << std::endl;
	}

	virtual void		OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
            std::cout << "A body went to sleep" << std::endl;
	}
};

//--------------------------------------//
// Global or static variables
//--------------------------------------//
JPH::PhysicsSystem                   *s_physics_system = nullptr;
static BPLayerInterfaceImpl                 s_broadphase_layer_interface;
static ObjectVsBroadPhaseLayerFilterImpl    s_object_vs_broadphase_layer_filter;
static ObjectLayerPairFilterImpl            s_object_layer_pair_filter;

JPH::TempAllocatorImpl               *s_temp_allocator  = nullptr;
static JPH::JobSystem                       *s_job_system      = nullptr;

static constexpr JPH::uint                  maxPhysicsJobs     = 1024;
static constexpr JPH::uint                  maxPhysicsBarriers = 8;


static MyBodyActivationListener             s_body_activation_listener;
static MyContactListener                    s_contact_listener;

JPH::BodyID                          debugSphereID;

//--------------------------------------//
// Implementation
//--------------------------------------//

void InitPhysicsSystem()
{
    // 1) Set up the allocation / trace / asserts
    JPH::RegisterDefaultAllocator();
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    // 2) Create factory + register
    if (!JPH::Factory::sInstance)
        JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    // 3) Create a TempAllocator for physics in this case we use 10MB
    s_temp_allocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);

    // 4) Create a JobSystem
    s_job_system = new JPH::JobSystemThreadPool(
        maxPhysicsJobs,
        maxPhysicsBarriers,
        std::thread::hardware_concurrency() - 1 
    );

    // 5) Create the PhysicsSystem
    s_physics_system = new JPH::PhysicsSystem();

    // capacity
    const uint maxBodies             = 1024;
    const uint numBodyMutexes        = 0;     // auto
    const uint maxBodyPairs          = 1024;
    const uint maxContactConstraints = 1024;

    s_physics_system->Init(
        maxBodies, numBodyMutexes,
        maxBodyPairs, maxContactConstraints,
        s_broadphase_layer_interface,
        s_object_vs_broadphase_layer_filter,
        s_object_layer_pair_filter
    );

    s_physics_system->SetGravity(JPH::Vec3(0.0f, -98.1f, 0.0f));

    s_physics_system->SetBodyActivationListener(&s_body_activation_listener);
    s_physics_system->SetContactListener(&s_contact_listener);
    
    std::cout << "[InitPhysicsSystem] Jolt setup complete!" << std::endl;
}

void ShutdownPhysicsSystem()
{
    if (!s_physics_system)
        return;

    // Example: remove bodies, etc. if needed

    // Then destroy
    delete s_physics_system;
    s_physics_system = nullptr;

    // Also delete job system, temp allocator
    delete s_job_system;
    s_job_system = nullptr;

    delete s_temp_allocator;
    s_temp_allocator = nullptr;

    // Unregister Jolt
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    std::cout << "[ShutdownPhysicsSystem] Freed Jolt resources." << std::endl;
}

JPH::BodyInterface &GetBodyInterface()
{
    // Must exist
    JPH_ASSERT(s_physics_system);
    return s_physics_system->GetBodyInterface();
}

void UpdatePhysicsSystem(float delta_time, JPH::BodyInterface *bodyInterface)
{
    // step dynamic bodies each frame if needed
    if (!s_physics_system)
        return;

    JPH::uint step = 0;
    if (bodyInterface->IsActive(debugSphereID)) {
        ++step;
        JPH::RVec3 position = bodyInterface->GetCenterOfMassPosition(debugSphereID);
        JPH::Vec3 velocity = bodyInterface->GetLinearVelocity(debugSphereID);
        // Uncomment for position updates.
       // std::cout << "Step " << step << ": Position = (" << position.GetX() << ", " << position.GetY() << ", " << position.GetZ() << "), Velocity = (" << velocity.GetX() << ", " << velocity.GetY() << ", " << velocity.GetZ() << ")" << std::endl;
    }

    // Do one step
    // If we intend on substeps or otherwise, edit this
    const int cCollisionSteps = 1;

    s_physics_system->Update(delta_time, cCollisionSteps, s_temp_allocator, s_job_system);
}

void SpawnDebugPhysObj(JPH::BodyInterface *bodyInterface) {

    JPH::RefConst<JPH::Shape> sphere_shape = new JPH::SphereShape(10.0f); 

    JPH::BodyCreationSettings sphere_settings (
        sphere_shape, JPH::RVec3( 0.0f, 1500.0f, -180.0f ),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Dynamic,
        Layers::MOVING);
    debugSphereID = bodyInterface->CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);
 
    printf("\n --TEST OBJECT SPAWNED-- \n");
}

void BuildMapPhysics(std::vector<MeshCollisionData> &meshCollisionData, JPH::BodyInterface *bodyInterface)
{
    int count = 0;

    for (auto &mcd : meshCollisionData) {
        // If NO_COLLIDE or something similar, we skip
        if (mcd.collisionType == CollisionType::NO_COLLIDE) {
            continue; 
        }

        // Convert Raylib vectors to Jolt float3
        std::vector<JPH::Float3> jolt_points;
        jolt_points.reserve(mcd.vertices.size());
        for (auto &v : mcd.vertices) {
            jolt_points.push_back(JPH::Float3(v.x, v.y, v.z));
        }

        // Build a convex hull from these points
        JPH::ConvexHullShapeSettings hull_settings;
        hull_settings.mPoints.resize(jolt_points.size());
        for (size_t i = 0; i < jolt_points.size(); ++i) {
            hull_settings.mPoints[i] = JPH::Vec3(
                jolt_points[i].x,
                jolt_points[i].y,
                jolt_points[i].z
            );
        }

        auto shape_result = hull_settings.Create();
        if (shape_result.HasError()) {
            printf("Error building hull shape: %s\n", shape_result.GetError().c_str());
            continue;
        }

        JPH::RefConst<JPH::Shape> hull_shape = shape_result.Get();

        // Decide motion type and layer from collisionType
        JPH::EMotionType motionType     = JPH::EMotionType::Static;
        JPH::ObjectLayer objectLayer    = Layers::NON_MOVING; // default

        switch (mcd.collisionType) {
            case CollisionType::STATIC: {
                motionType = JPH::EMotionType::Static;
                objectLayer = Layers::NON_MOVING;
                break;
            }
            case CollisionType::TRIGGER: {
                // Often we keep this static but on a special "trigger" layer,
                // or still NON_MOVING if the filters treat triggers differently.
                motionType = JPH::EMotionType::Static; 
                objectLayer = Layers::SENSOR; // Or a custom TRIGGER layer if we implement one.
                break;
            }
            case CollisionType::DYNAMIC: {
                motionType = JPH::EMotionType::Dynamic;
                objectLayer = Layers::MOVING;
                break;
            }
            // NO_COLLIDE or UNKNOWN => skip
            default: {
                continue;
            }
        }

        // Create a body creation settings with the chosen motion/layer
        JPH::BodyCreationSettings bcs(
            hull_shape,
            JPH::RVec3::sZero(),       // offset if local geometry
            JPH::Quat::sIdentity(),
            motionType,
            objectLayer
        );

        if (mcd.collisionType == CollisionType::TRIGGER) {
            bcs.mIsSensor = true;
            printf("\n SETTING IS SENSOR TO TRUE \n");
        }

        if (JPH::Body *body = bodyInterface->CreateBody(bcs)) {
            bodyInterface->AddBody(body->GetID(), JPH::EActivation::Activate);
            ++count;
        }
        else {
            printf("Failed to create body for a brush\n");
        }
    }

    printf("\n\n %d MAP COLLISIONS SUCCESSFULLY CREATED \n\n", count);
}

