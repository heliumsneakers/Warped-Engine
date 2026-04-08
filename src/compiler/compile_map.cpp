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
#include "structural_bsp.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
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

static uint16_t Float32ToHalfBits(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));

    const uint32_t sign = (bits >> 16u) & 0x8000u;
    const uint32_t absBits = bits & 0x7FFFFFFFu;

    if (absBits >= 0x7F800000u) {
        const uint32_t mantissa = absBits & 0x007FFFFFu;
        if (mantissa != 0u) {
            return (uint16_t)(sign | 0x7C00u | std::max<uint32_t>(1u, mantissa >> 13u));
        }
        return (uint16_t)(sign | 0x7C00u);
    }

    if (absBits > 0x477FEFFFu) {
        return (uint16_t)(sign | 0x7BFFu);
    }

    if (absBits < 0x38800000u) {
        if (absBits < 0x33000000u) {
            return (uint16_t)sign;
        }

        uint32_t mantissa = (absBits & 0x007FFFFFu) | 0x00800000u;
        const uint32_t exp = absBits >> 23u;
        const uint32_t shift = 126u - exp;
        mantissa = (mantissa + (1u << (shift - 1u))) >> shift;
        return (uint16_t)(sign | mantissa);
    }

    uint32_t rounded = absBits + 0x00001000u;
    if (rounded >= 0x47800000u) {
        return (uint16_t)(sign | 0x7BFFu);
    }
    return (uint16_t)(sign | ((rounded - 0x38000000u) >> 13u));
}

static std::vector<uint8_t> EncodeLightmapPageRGBA16F(const LightmapPage& page)
{
    std::vector<uint8_t> encoded(page.pixels.size() * sizeof(uint16_t), 0);
    for (size_t i = 0; i < page.pixels.size(); ++i) {
        const uint16_t half = Float32ToHalfBits(std::max(0.0f, page.pixels[i]));
        memcpy(encoded.data() + i * sizeof(uint16_t), &half, sizeof(half));
    }
    return encoded;
}

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
    std::vector<MapPolygon> rawPolys  = BuildMapPolygons(map, /*devMode=*/false);
    std::vector<PointLight> lights = GetPointLights(map);
    std::vector<SurfaceLightTemplate> surfaceLights = GetSurfaceLightTemplates(map);
    LightBakeSettings lightSettings = GetLightBakeSettings(map);
    std::unordered_map<std::string,TexInfo> texCache;
    std::unordered_map<std::string,uint32_t> texIdx;
    std::vector<BSPTexture> textures;
    std::vector<PackagedAssetEntry> packagedAssets;

    auto GetTex = [&](const std::string& n)->uint32_t {
        const std::string& resolvedName = n.empty() ? std::string("default") : n;
        auto it = texIdx.find(resolvedName);
        if (it!=texIdx.end()) return it->second;
        TexInfo ti = ProbeTexture(mapDir, resolvedName, texCache);
        BSPTexture bt{}; strncpy(bt.name, resolvedName.c_str(), 63); bt.width=ti.w; bt.height=ti.h;
        uint32_t idx=(uint32_t)textures.size(); textures.push_back(bt);
        if (IsPackableTextureName(resolvedName)) {
            packagedAssets.push_back({
                "textures/" + resolvedName + ".png",
                mapDir + "/../textures/" + resolvedName + ".png"
            });
        }
        texIdx[resolvedName]=idx; return idx;
    };

    StructuralBSPData structural = BuildStructuralBSP(rawPolys, GetTex);
    const std::vector<MapPolygon>& bspPolys =
        structural.splitPolygons.empty() ? rawPolys : structural.splitPolygons;
    printf("[compile_map] structural bsp: %zu raw faces -> %zu bsp faces, %zu planes, %zu nodes, %zu leaves\n",
           rawPolys.size(), bspPolys.size(), structural.planes.size(), structural.nodes.size(), structural.leaves.size());

    printf("[compile_map] baking lightmap...\n");
    LightmapAtlas lm = BakeLightmap(bspPolys, bspPolys, lights, surfaceLights, lightSettings);

    // ----- triangulate into buckets ---------------------------------------
    std::vector<BSPVertex>  vertices;
    std::vector<uint32_t>   indices;
    std::vector<BSPMesh>    meshes;

    struct Bucket { uint32_t firstIdx, firstVtx; uint32_t lightmapPage = 0; std::vector<BSPVertex> v; std::vector<uint32_t> i; };
    std::unordered_map<uint64_t,Bucket> buckets;

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
        h.entityIndex   = c.entityIndex;
        for (auto& v : c.vertices) hullPts.push_back({v.x,v.y,v.z});
        hulls.push_back(h);
    }

    // ----- entity text -----------------------------------------------------
    std::string entText;
    for (auto& e : map.entities) {
        entText += "{\n";
        for (auto& kv : e.properties)
            entText += "\""+kv.first+"\" \""+kv.second+"\"\n";
        entText += "}\n";
    }
    entText += '\0';

    // ----- lightmap lump ---------------------------------------------------
    std::vector<std::vector<uint8_t>> encodedLightmapPages;
    encodedLightmapPages.reserve(lm.pages.size());
    size_t lmLumpSize = sizeof(BSPLightmapLumpHeader) + lm.pages.size() * sizeof(BSPLightmapPageHeader);
    for (const LightmapPage& page : lm.pages) {
        encodedLightmapPages.push_back(EncodeLightmapPageRGBA16F(page));
        lmLumpSize += encodedLightmapPages.back().size();
    }
    std::vector<uint8_t> lmLump(lmLumpSize);
    uint8_t* lmWrite = lmLump.data();

    BSPLightmapLumpHeader lmHeader{ (uint32_t)lm.pages.size() };
    memcpy(lmWrite, &lmHeader, sizeof(lmHeader));
    lmWrite += sizeof(lmHeader);

    for (size_t pageIndex = 0; pageIndex < lm.pages.size(); ++pageIndex) {
        const LightmapPage& page = lm.pages[pageIndex];
        BSPLightmapPageHeader pageHeader{
            (uint32_t)page.width,
            (uint32_t)page.height,
            (uint32_t)encodedLightmapPages[pageIndex].size(),
            BSP_LIGHTMAP_FORMAT_RGBA16F
        };
        memcpy(lmWrite, &pageHeader, sizeof(pageHeader));
        lmWrite += sizeof(pageHeader);
    }

    for (const std::vector<uint8_t>& encodedPage : encodedLightmapPages) {
        if (!encodedPage.empty()) {
            memcpy(lmWrite, encodedPage.data(), encodedPage.size());
            lmWrite += encodedPage.size();
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
    lw.WriteRaw(LUMP_BSP_TREE, &structural.tree, sizeof(structural.tree));
    lw.Write   (LUMP_BSP_PLANES, structural.planes);
    lw.Write   (LUMP_BSP_FACES, structural.faces);
    lw.Write   (LUMP_BSP_FACE_VERTS, structural.faceVerts);
    lw.Write   (LUMP_BSP_NODES, structural.nodes);
    lw.Write   (LUMP_BSP_LEAVES, structural.leaves);
    lw.Write   (LUMP_BSP_FACE_REFS, structural.faceRefs);
    lw.End();
    fclose(f);

    std::string packError;
    if (!WriteAssetPackRresWithMipmaps(outPackName, packagedAssets, &packError)) {
        fprintf(stderr, "[compile_map] failed to write %s: %s\n", outPackName.c_str(), packError.c_str());
        return 1;
    }

    printf("\n[compile_map] wrote %s\n", outName.c_str());
    printf("[compile_map] wrote %s\n", outPackName.c_str());
    printf("  textures : %zu\n  vertices : %zu\n  indices  : %zu\n"
           "  meshes   : %zu\n  hulls    : %zu\n  lightmap pages : %zu\n"
           "  bsp faces: %zu\n  bsp nodes: %zu\n  bsp leaves: %zu\n",
           textures.size(), vertices.size(), indices.size(),
           meshes.size(), hulls.size(), lm.pages.size(),
           structural.faces.size(), structural.nodes.size(), structural.leaves.size());
    return 0;
}
