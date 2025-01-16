#include "collision_data.h"
#include <vector>

std::vector<MeshCollisionData> ExtractCollisionData(const Model &model) {
    std::vector<MeshCollisionData> result;
    result.reserve(model.meshCount);

    // Iterate over meshes from the parsed and constructed map model.
    for (int meshIndex = 0; meshIndex < model.meshCount; ++meshIndex) {
        const Mesh &mesh = model.meshes[meshIndex];
        if (mesh.vertexCount <= 0 || mesh.vertices == nullptr) {
            continue;
        }

        // Store unique positions
        std::vector<Vector3> points;
        points.reserve(mesh.vertexCount);

        // Read mesh verts and store them
        for (int v = 0; v < mesh.vertexCount; ++v) {
            float x = mesh.vertices[v*3 + 0];
            float y = mesh.vertices[v*3 + 1];
            float z = mesh.vertices[v*3 + 2];

            points.push_back( {x, y, z} );
        }

        /* NOTE: Duplicate points are already removed during the parsing and building of the model.
         * There should be no reason to redo this step but it is always best to debug draw this collision
         * geometry to ensure that this will work properly.
         * */

        MeshCollisionData mcd;
        mcd.vertices = std::move(points);
        result.push_back(std::move(mcd));
    }
    return result;
}
