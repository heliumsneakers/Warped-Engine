// compile_map.cpp  —  offline .map → .bsp compiler.
//
//   Usage:  ./compile_map <PATH_TO_MAP_FILE> <COMPILED_MAP_NAME>
//
// Produces <COMPILED_MAP_NAME>.bsp containing pre-triangulated render
// geometry with baked lightmap UVs, convex-hull collision data, the
// entity text block, and the lightmap atlas pixels.

#include "../utils/map_parser.h"
#include "../utils/bsp_format.h"
#include "../utils/asset_pack.h"
#include "../physx/collision_data.h"
#include "lightmap.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <unordered_map>

// --------------------------------------------------------------------------
//  Texture-size probe (no GPU).  Mirrors renderer's lookup path.
// --------------------------------------------------------------------------
struct TexInfo { int w=64, h=64; };   // default if PNG missing

static bool IsPackableTextureName(const std::string& name)
{
    return name.rfind("__light_brush_", 0) != 0;
}

static TexInfo ProbeTexture(const std::string& mapDir, const std::string& name,
                            std::unordered_map<std::string,TexInfo>& cache)
{
    auto it = cache.find(name);
    if (it != cache.end()) return it->second;

    // assets/maps/<file>.map  →  ../textures/<name>.png
    std::string path = mapDir + "/../textures/" + name + ".png";
    int w,h,comp;
    TexInfo ti;
    if (stbi_info(path.c_str(), &w, &h, &comp)) { ti.w=w; ti.h=h; }
    else printf("[compile_map] texture '%s' not found, assuming 64x64\n", name.c_str());
    cache[name] = ti;
    return ti;
}

// --------------------------------------------------------------------------
//  Lump writer helper
// --------------------------------------------------------------------------
struct LumpWriter {
    FILE* f;
    BSPHeader hdr{};
    long hdrPos;

    void Begin() {
        hdr.magic = WBSP_MAGIC;
        hdr.version = WBSP_VERSION;
        hdrPos = ftell(f);
        fwrite(&hdr, sizeof(hdr), 1, f);      // placeholder
    }
    template<typename T>
    void Write(int lump, const std::vector<T>& v) {
        hdr.lumps[lump].offset = (uint32_t)ftell(f);
        hdr.lumps[lump].length = (uint32_t)(v.size()*sizeof(T));
        if (!v.empty()) fwrite(v.data(), sizeof(T), v.size(), f);
    }
    void WriteRaw(int lump, const void* data, size_t len) {
        hdr.lumps[lump].offset = (uint32_t)ftell(f);
        hdr.lumps[lump].length = (uint32_t)len;
        if (len) fwrite(data, 1, len, f);
    }
    void End() {
        fseek(f, hdrPos, SEEK_SET);
        fwrite(&hdr, sizeof(hdr), 1, f);
    }
};

// --------------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <PATH_TO_MAP_FILE> <COMPILED_MAP_NAME>\n", argv[0]);
        return 1;
    }
    std::string mapPath = argv[1];
    std::string outName = argv[2];
    if (outName.size()<4 || outName.substr(outName.size()-4)!=".bsp")
        outName += ".bsp";
    std::string outPackName = GetCompanionRresPath(outName);

    std::string mapDir = ".";
    size_t slash = mapPath.find_last_of("/\\");
    if (slash != std::string::npos) mapDir = mapPath.substr(0, slash);

    printf("[compile_map] parsing %s\n", mapPath.c_str());
    Map map = ParseMapFile(mapPath);
    if (map.entities.empty()) { fprintf(stderr,"No entities parsed.\n"); return 1; }

    // ----- geometry + lights ----------------------------------------------
    std::vector<MapPolygon> polys  = BuildMapPolygons(map, /*devMode=*/false);
    std::vector<PointLight> lights = GetPointLights(map);

    printf("[compile_map] baking lightmap...\n");
    LightmapAtlas lm = BakeLightmap(polys, lights);

    // ----- triangulate into buckets ---------------------------------------
    std::unordered_map<std::string,TexInfo> texCache;
    std::unordered_map<std::string,uint32_t> texIdx;
    std::vector<BSPTexture> textures;
    std::vector<PackagedAssetEntry> packagedAssets;
    std::vector<BSPVertex>  vertices;
    std::vector<uint32_t>   indices;
    std::vector<BSPMesh>    meshes;

    struct Bucket { uint32_t firstIdx, firstVtx; uint32_t lightmapPage = 0; std::vector<BSPVertex> v; std::vector<uint32_t> i; };
    std::unordered_map<uint64_t,Bucket> buckets;

    auto GetTex = [&](const std::string& n)->uint32_t {
        auto it = texIdx.find(n);
        if (it!=texIdx.end()) return it->second;
        TexInfo ti = ProbeTexture(mapDir, n, texCache);
        BSPTexture bt{}; strncpy(bt.name, n.c_str(), 63); bt.width=ti.w; bt.height=ti.h;
        uint32_t idx=(uint32_t)textures.size(); textures.push_back(bt);
        if (IsPackableTextureName(n)) {
            packagedAssets.push_back({
                "textures/" + n + ".png",
                mapDir + "/../textures/" + n + ".png"
            });
        }
        texIdx[n]=idx; return idx;
    };

    for (size_t pi=0; pi<lm.patches.size(); ++pi) {
        const LightmapPatch& patch = lm.patches[pi];
        const MapPolygon& p = patch.poly;
        uint32_t ti = GetTex(p.texture);
        TexInfo  td = texCache[p.texture];
        const uint32_t lightmapPage = patch.page;
        const uint64_t bucketKey = ((uint64_t)lightmapPage << 32) | (uint64_t)ti;
        Bucket& b = buckets[bucketKey];
        b.lightmapPage = lightmapPage;

        auto MakeV = [&](size_t vi)->BSPVertex {
            const Vector3& vp = p.verts[vi];
            Vector2 uv = ComputeFaceUV(vp,p.texAxisU,p.texAxisV,p.offU,p.offV,
                                       p.rot,p.scaleU,p.scaleV,(float)td.w,(float)td.h);
            const Vector2& luv = patch.uv[vi];
            return BSPVertex{vp.x,vp.y,vp.z, p.normal.x,p.normal.y,p.normal.z,
                             uv.x,uv.y, luv.x,luv.y};
        };

        uint32_t base = (uint32_t)b.v.size();
        for (size_t vi=0; vi<p.verts.size(); ++vi) b.v.push_back(MakeV(vi));
        for (size_t t=1; t+1<p.verts.size(); ++t) {
            b.i.push_back(base);
            b.i.push_back(base+(uint32_t)t);
            b.i.push_back(base+(uint32_t)t+1);
        }
    }

    // flatten buckets → lumps
    for (auto& kv : buckets) {
        BSPMesh m{};
        m.textureIndex = (uint32_t)(kv.first & 0xFFFFFFFFu);
        m.lightmapPage = kv.second.lightmapPage;
        m.firstVertex  = (uint32_t)vertices.size();
        m.vertexCount  = (uint32_t)kv.second.v.size();
        m.firstIndex   = (uint32_t)indices.size();
        m.indexCount   = (uint32_t)kv.second.i.size();
        for (auto& v : kv.second.v) vertices.push_back(v);
        for (auto  i : kv.second.i) indices.push_back(i + m.firstVertex);
        meshes.push_back(m);
    }

    // ----- collision -------------------------------------------------------
    std::vector<MeshCollisionData> coll = ExtractCollisionData(map);
    std::vector<BSPHull> hulls;
    std::vector<BSPVec3> hullPts;
    for (auto& c : coll) {
        BSPHull h{};
        h.firstPoint    = (uint32_t)hullPts.size();
        h.pointCount    = (uint32_t)c.vertices.size();
        h.collisionType = (uint32_t)c.collisionType;
        for (auto& v : c.vertices) hullPts.push_back({v.x,v.y,v.z});
        hulls.push_back(h);
    }

    // ----- entity text -----------------------------------------------------
    std::string entText;
    for (auto& e : map.entities) {
        if (!e.brushes.empty()) continue;          // brush ents handled via geom/hulls
        entText += "{\n";
        for (auto& kv : e.properties)
            entText += "\""+kv.first+"\" \""+kv.second+"\"\n";
        entText += "}\n";
    }
    entText += '\0';

    // ----- lightmap lump ---------------------------------------------------
    size_t lmLumpSize = sizeof(BSPLightmapLumpHeader) + lm.pages.size() * sizeof(BSPLightmapPageHeader);
    for (const LightmapPage& page : lm.pages) {
        lmLumpSize += page.pixels.size();
    }
    std::vector<uint8_t> lmLump(lmLumpSize);
    uint8_t* lmWrite = lmLump.data();

    BSPLightmapLumpHeader lmHeader{ (uint32_t)lm.pages.size() };
    memcpy(lmWrite, &lmHeader, sizeof(lmHeader));
    lmWrite += sizeof(lmHeader);

    for (const LightmapPage& page : lm.pages) {
        BSPLightmapPageHeader pageHeader{
            (uint32_t)page.width,
            (uint32_t)page.height,
            (uint32_t)page.pixels.size()
        };
        memcpy(lmWrite, &pageHeader, sizeof(pageHeader));
        lmWrite += sizeof(pageHeader);
    }

    for (const LightmapPage& page : lm.pages) {
        if (!page.pixels.empty()) {
            memcpy(lmWrite, page.pixels.data(), page.pixels.size());
            lmWrite += page.pixels.size();
        }
    }

    // ----- write -----------------------------------------------------------
    FILE* f = fopen(outName.c_str(),"wb");
    if (!f) { fprintf(stderr,"Cannot open %s for write\n",outName.c_str()); return 1; }

    LumpWriter lw{f};
    lw.Begin();
    lw.Write   (LUMP_TEXTURES, textures);
    lw.Write   (LUMP_VERTICES, vertices);
    lw.Write   (LUMP_INDICES,  indices);
    lw.Write   (LUMP_MESHES,   meshes);
    lw.Write   (LUMP_HULLS,    hulls);
    lw.Write   (LUMP_HULL_PTS, hullPts);
    lw.WriteRaw(LUMP_ENTITIES, entText.data(), entText.size());
    lw.WriteRaw(LUMP_LIGHTMAP, lmLump.data(),  lmLump.size());
    lw.End();
    fclose(f);

    std::string packError;
    if (!WriteAssetPackRres(outPackName, packagedAssets, &packError)) {
        fprintf(stderr, "[compile_map] failed to write %s: %s\n", outPackName.c_str(), packError.c_str());
        return 1;
    }

    printf("\n[compile_map] wrote %s\n", outName.c_str());
    printf("[compile_map] wrote %s\n", outPackName.c_str());
    printf("  textures : %zu\n  vertices : %zu\n  indices  : %zu\n"
           "  meshes   : %zu\n  hulls    : %zu\n  lightmap pages : %zu\n",
           textures.size(), vertices.size(), indices.size(),
           meshes.size(), hulls.size(), lm.pages.size());
    return 0;
}
