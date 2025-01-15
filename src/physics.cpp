#include "physics.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/Collision/Shape/ConvexShape.h"
#include "Jolt/Geometry/ConvexSupport.h"
#include "Jolt/Geometry/GJKClosestPoint.h"


/* Here we will setup all Jolt Physics functions for collisions with the world.
 *  Theoretical structure:
 *      - The worldspawn layer in TB will contain all of our brushes, brushes
 *        inside the worldspawn layer that arent entities (besides clip brushes), will be
 *        calculated as "Convex Shapes" for applying the minkowski difference and GJK algorithm
 *        for collision detection. This is heavily inspired by Quake's collision system
 *        which has stood the test of time.
 *
 *      - Other brush entites like triggers, physics objects, etc.. will need to be handled
 *        differently. Potentially in a similar way to world brushes but there isn't much
 *        documentation or examples on convex collisions specific to the use case I intend
 *        for this engine. A lot of this will be trial and error when wrangling the custom
 *        mesh building code into something usable within Jolt's context.
 *
 *        NOTE: Maybe the polygon code doesn't need to be used, just the planes themselves? 
 *              I believe this would save a ton of compute if possible.
 * */



