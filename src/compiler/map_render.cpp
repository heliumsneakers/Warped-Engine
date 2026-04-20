#include "map_render.h"

#ifndef WARPED_COMPILER_BUILD
#include "map_geometry.h"
#include "map_polygons.h"
#include "../utils/parameters.h"
#include "../render/renderer.h"

#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

std::vector<MapMeshBucket> BuildMapGeometry(const Map &map, TextureManager &textureManager)
{
    std::vector<MapPolygon> polys = BuildExteriorMapPolygons(map, DEVMODE);
    if (polys.empty()) {
        polys = BuildMapPolygons(map, DEVMODE);
    }

    std::unordered_map<std::string, size_t> bucketIdx;
    std::vector<MapMeshBucket> buckets;
    auto GetBucket = [&](const std::string& n) -> MapMeshBucket& {
        auto it = bucketIdx.find(n);
        if (it != bucketIdx.end()) return buckets[it->second];
        bucketIdx[n] = buckets.size();
        buckets.push_back(MapMeshBucket{});
        buckets.back().texture = n;
        return buckets.back();
    };

    for (auto &p : polys) {
        std::string texName = p.texture;
        const TextureEntry* tex = LoadTextureByName(textureManager, texName);
        if (!tex || tex->width == 0) {
            tex = LoadTextureByName(textureManager, "default");
            texName = "default";
        }
        float tW=(float)tex->width, tH=(float)tex->height;

        MapMeshBucket& b = GetBucket(texName);
        const std::vector<uint32_t> triIndices = TriangulatePolygonIndices(p.verts, p.normal);
        for (size_t t=0; t+2<triIndices.size(); t += 3) {
            Vector3 v0 = p.verts[triIndices[t]];
            Vector3 v1 = p.verts[triIndices[t + 1]];
            Vector3 v2 = p.verts[triIndices[t + 2]];
            Vector2 uv0 = ComputeFaceUV(v0,p.texAxisU,p.texAxisV,p.offU,p.offV,p.rot,p.scaleU,p.scaleV,tW,tH);
            Vector2 uv1 = ComputeFaceUV(v1,p.texAxisU,p.texAxisV,p.offU,p.offV,p.rot,p.scaleU,p.scaleV,tW,tH);
            Vector2 uv2 = ComputeFaceUV(v2,p.texAxisU,p.texAxisV,p.offU,p.offV,p.rot,p.scaleU,p.scaleV,tW,tH);
            uint32_t base=(uint32_t)b.vertices.size();
            b.vertices.push_back({v0.x,v0.y,v0.z, p.normal.x,p.normal.y,p.normal.z, uv0.x,uv0.y, 0,0});
            b.vertices.push_back({v1.x,v1.y,v1.z, p.normal.x,p.normal.y,p.normal.z, uv1.x,uv1.y, 0,0});
            b.vertices.push_back({v2.x,v2.y,v2.z, p.normal.x,p.normal.y,p.normal.z, uv2.x,uv2.y, 0,0});
            b.indices.push_back(base); b.indices.push_back(base+1); b.indices.push_back(base+2);
        }
    }
    printf("[MapParser] Built %zu texture buckets.\n", buckets.size());
    return buckets;
}

#endif // !WARPED_COMPILER_BUILD
