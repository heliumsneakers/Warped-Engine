#include "physics.h"
#include "collision_data.h"

#include "Jolt/Jolt.h"
#include "Jolt/Math/Float3.h"
#include "Jolt/Physics/Body/MotionType.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/Shape/ConvexShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Geometry/ConvexSupport.h"
#include "Jolt/Geometry/GJKClosestPoint.h"


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


void BuildMapPhysics(const std::vector<MeshCollisionData> &meshCollisionData, JPH::BodyInterface &bodyInterface) { 
    
    // For each set of points in our point cloud stored in MeshCollisonData we convert to Jolt and 
    // create a convex hull shape.
    for (auto &mcd : meshCollisionData) {
        
        // Convert to Jolt
        std::vector<JPH::Float3> jolt_points;
        jolt_points.reserve(mcd.vertices.size());
        for (auto &v : mcd.vertices) {
            jolt_points.push_back(JPH::Float3(v.x, v.y, v.z));
        }

        // Build the Convex Hull
        JPH::ConvexHullShapeSettings hull_settings;

        hull_settings.mPoints.clear();
        hull_settings.mPoints.resize(jolt_points.size());

        for (size_t i = 0; i < jolt_points.size(); ++i) {
            hull_settings.mPoints[i] = JPH::Vec3(jolt_points[i].x,
                                                 jolt_points[i].y,
                                                 jolt_points[i].z);
        }

        auto shape_result = hull_settings.Create(); 
        if (shape_result.HasError()) {
            printf("Error building hull shape: %s\n", shape_result.GetError().c_str());
            continue;
        }

        JPH::Shape *hull_shape = shape_result.Get();

        // For now create static body of brushes for debug purposes to see if everything is working as intended.
        JPH::BodyCreationSettings bcs (
            hull_shape,
            JPH::RVec3::sZero(),
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Static,
            World_Layers::WORLD_STATIC
        );

        // create and add body to the physics interface 
        if (JPH::Body *body = bodyInterface.CreateBody(bcs)) {
            bodyInterface.AddBody(body->GetID(), JPH::EActivation::DontActivate);
        } else printf("Failed to create body for a brush\n");
    }
}


