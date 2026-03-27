// bsp_loader.cpp
#include "bsp_loader.h"
#include "bsp_format.h"
#include "asset_pack.h"
#include <cstdio>
#include <cstring>
#include <sstream>

template<typename T>
static std::vector<T> ReadLump(FILE* f, const BSPLump& l) {
    std::vector<T> v(l.length / sizeof(T));
    if (l.length) { fseek(f, l.offset, SEEK_SET); fread(v.data(), 1, l.length, f); }
    return v;
}

bool LoadBSP(const char* path, BSPData& out)
{
    out = {};

    FILE* f = fopen(path, "rb");
    if (!f) { printf("[BSP] cannot open %s\n", path); return false; }

    BSPHeader hdr{};
    fread(&hdr, sizeof(hdr), 1, f);
    if (hdr.magic != WBSP_MAGIC || hdr.version != WBSP_VERSION) {
        printf("[BSP] bad header in %s\n", path); fclose(f); return false;
    }

    auto textures = ReadLump<BSPTexture>(f, hdr.lumps[LUMP_TEXTURES]);
    auto verts    = ReadLump<BSPVertex> (f, hdr.lumps[LUMP_VERTICES]);
    auto idx      = ReadLump<uint32_t>  (f, hdr.lumps[LUMP_INDICES]);
    auto meshes   = ReadLump<BSPMesh>   (f, hdr.lumps[LUMP_MESHES]);
    auto hulls    = ReadLump<BSPHull>   (f, hdr.lumps[LUMP_HULLS]);
    auto hullPts  = ReadLump<BSPVec3>   (f, hdr.lumps[LUMP_HULL_PTS]);

    // ----- render buckets -------------------------------------------------
    out.buckets.reserve(meshes.size());
    for (auto& m : meshes) {
        MapMeshBucket b;
        b.texture = textures[m.textureIndex].name;
        b.vertices.reserve(m.vertexCount);
        for (uint32_t i=0;i<m.vertexCount;++i) {
            const BSPVertex& v = verts[m.firstVertex+i];
            b.vertices.push_back({v.x,v.y,v.z, v.nx,v.ny,v.nz, v.u,v.v, v.lu,v.lv});
        }
        b.indices.reserve(m.indexCount);
        for (uint32_t i=0;i<m.indexCount;++i)
            b.indices.push_back(idx[m.firstIndex+i] - m.firstVertex);
        out.buckets.push_back(std::move(b));
    }

    // ----- collision hulls -----------------------------------------------
    out.hulls.reserve(hulls.size());
    for (auto& h : hulls) {
        MeshCollisionData mcd;
        mcd.collisionType = (CollisionType)h.collisionType;
        mcd.vertices.reserve(h.pointCount);
        for (uint32_t i=0;i<h.pointCount;++i) {
            const BSPVec3& p = hullPts[h.firstPoint+i];
            mcd.vertices.push_back({p.x,p.y,p.z});
        }
        out.hulls.push_back(std::move(mcd));
    }

    // ----- entities (re-parse simple key/value text) ---------------------
    {
        const BSPLump& l = hdr.lumps[LUMP_ENTITIES];
        std::string text(l.length, '\0');
        if (l.length) { fseek(f,l.offset,SEEK_SET); fread(&text[0],1,l.length,f); }

        std::istringstream ss(text);
        std::string line; Entity cur; bool in=false;
        while (std::getline(ss,line)) {
            if (line=="{") { cur=Entity(); in=true; continue; }
            if (line=="}") { if(in){out.entities.push_back(cur);in=false;} continue; }
            if (!in) continue;
            size_t q1=line.find('"'); if(q1==std::string::npos) continue;
            size_t q2=line.find('"',q1+1); if(q2==std::string::npos) continue;
            size_t q3=line.find('"',q2+1); if(q3==std::string::npos) continue;
            size_t q4=line.find('"',q3+1); if(q4==std::string::npos) continue;
            cur.properties[line.substr(q1+1,q2-q1-1)] = line.substr(q3+1,q4-q3-1);
        }
    }

    // ----- lightmap ------------------------------------------------------
    {
        const BSPLump& l = hdr.lumps[LUMP_LIGHTMAP];
        if (l.length >= sizeof(BSPLightmapHeader)) {
            BSPLightmapHeader lh{};
            fseek(f,l.offset,SEEK_SET); fread(&lh,sizeof(lh),1,f);
            out.lightmapW = (int)lh.width;
            out.lightmapH = (int)lh.height;
            size_t px = (size_t)lh.width*lh.height*4;
            out.lightmapPixels.resize(px);
            fread(out.lightmapPixels.data(),1,px,f);
        }
    }

    fclose(f);
    out.assetPackPath = GetCompanionRresPath(path);
    printf("[BSP] loaded %s: %zu meshes, %zu hulls, %zu ents, lm %dx%d\n",
           path, out.buckets.size(), out.hulls.size(), out.entities.size(),
           out.lightmapW, out.lightmapH);
    return true;
}

std::vector<PlayerStart> GetPlayerStarts(const BSPData& bsp) {
    Map tmp; tmp.entities = bsp.entities;
    return GetPlayerStarts(tmp);
}
