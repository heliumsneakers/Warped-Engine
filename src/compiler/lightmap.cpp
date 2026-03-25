// lightmap.cpp  —  ray-traced lightmap baker with hard shadows.
//
// Per luxel: fire N² stratified sub-pixel samples on a regular grid.
// Each sample traces one shadow ray per light toward the exact light
// position (point light → hard shadow edge). Averaging the grid samples
// anti-aliases the shadow boundary across ~1 luxel without Monte-Carlo
// noise — you get a sharp but not stair-stepped edge.
//
// Edge-bleed fix: only luxels whose world position actually lies on the
// polygon are considered "valid". After the bake a dilation pass flood-fills
// invalid/padding luxels from their nearest valid neighbour so bilinear
// filtering at runtime never samples garbage.

#include "lightmap.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

// --------------------------------------------------------------------------
//  Tunables
// --------------------------------------------------------------------------
static constexpr float LUXEL_SIZE    = 1.0f;   // world units per luxel
static constexpr int   LM_PAD        = 2;      // border luxels (filled by dilate)
static constexpr int   ATLAS_MAX     = 4096;
static constexpr float AMBIENT       = 0.12f;
static constexpr float SHADOW_BIAS   = 0.25f;
static constexpr float RAY_EPS       = 1e-4f;
static constexpr int   AA_GRID       = 4;      // AA_GRID² samples per luxel
static constexpr int   DILATE_PASSES = 4;

// --------------------------------------------------------------------------
//  Planar basis
// --------------------------------------------------------------------------
static void FaceBasis(const Vector3& n, Vector3& u, Vector3& v) {
    Vector3 ref = (fabsf(n.y) < 0.9f) ? (Vector3){0,1,0} : (Vector3){1,0,0};
    u = Vector3Normalize(Vector3CrossProduct(n, ref));
    v = Vector3CrossProduct(n, u);
}

// --------------------------------------------------------------------------
//  Occluders
// --------------------------------------------------------------------------
struct Tri { Vector3 a, b, c; };
struct OccluderSet {
    std::vector<Tri>  tris;
    std::vector<AABB> bounds;
};

static OccluderSet BuildOccluders(const std::vector<MapPolygon>& polys) {
    OccluderSet o;
    for (auto& p : polys)
        for (size_t t=1; t+1<p.verts.size(); ++t) {
            Tri tr{p.verts[0], p.verts[t], p.verts[t+1]};
            AABB bb = AABBInvalid();
            AABBExtend(&bb,tr.a); AABBExtend(&bb,tr.b); AABBExtend(&bb,tr.c);
            o.tris.push_back(tr); o.bounds.push_back(bb);
        }
    return o;
}

static bool RayTri(const Vector3& ro, const Vector3& rd,
                   const Tri& tr, float tmin, float tmax)
{
    Vector3 e1 = Vector3Subtract(tr.b, tr.a);
    Vector3 e2 = Vector3Subtract(tr.c, tr.a);
    Vector3 p  = Vector3CrossProduct(rd, e2);
    float det  = Vector3DotProduct(e1, p);
    if (fabsf(det) < RAY_EPS) return false;
    float inv = 1.0f/det;
    Vector3 s = Vector3Subtract(ro, tr.a);
    float u = Vector3DotProduct(s,p)*inv;      if (u<0||u>1) return false;
    Vector3 q = Vector3CrossProduct(s,e1);
    float v = Vector3DotProduct(rd,q)*inv;     if (v<0||u+v>1) return false;
    float t = Vector3DotProduct(e2,q)*inv;
    return t>tmin && t<tmax;
}

static bool RayAABB(const Vector3& ro, const Vector3& inv_rd,
                    const AABB& b, float tmax)
{
    float t1,t2,tn=0,tf=tmax;
    t1=(b.min.x-ro.x)*inv_rd.x; t2=(b.max.x-ro.x)*inv_rd.x;
    tn=std::max(tn,std::min(t1,t2)); tf=std::min(tf,std::max(t1,t2));
    t1=(b.min.y-ro.y)*inv_rd.y; t2=(b.max.y-ro.y)*inv_rd.y;
    tn=std::max(tn,std::min(t1,t2)); tf=std::min(tf,std::max(t1,t2));
    t1=(b.min.z-ro.z)*inv_rd.z; t2=(b.max.z-ro.z)*inv_rd.z;
    tn=std::max(tn,std::min(t1,t2)); tf=std::min(tf,std::max(t1,t2));
    return tf>=tn;
}

static bool Occluded(const OccluderSet& o, const Vector3& ro,
                     const Vector3& rd, float dist)
{
    Vector3 inv = { 1.0f/(fabsf(rd.x)>RAY_EPS?rd.x:RAY_EPS),
                    1.0f/(fabsf(rd.y)>RAY_EPS?rd.y:RAY_EPS),
                    1.0f/(fabsf(rd.z)>RAY_EPS?rd.z:RAY_EPS) };
    for (size_t i=0;i<o.tris.size();++i) {
        if (!RayAABB(ro,inv,o.bounds[i],dist)) continue;
        if (RayTri(ro,rd,o.tris[i],RAY_EPS,dist)) return true;
    }
    return false;
}

// --------------------------------------------------------------------------
//  Point-in-convex-polygon test (2-D, polygon already CCW in plane space).
//  Luxels outside the poly are marked invalid → fixed by dilation.
// --------------------------------------------------------------------------
static bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py) {
    size_t n = poly.size();
    for (size_t i=0;i<n;++i) {
        size_t j=(i+1)%n;
        float ex=poly[j].x-poly[i].x, ey=poly[j].y-poly[i].y;
        float cx=px-poly[i].x,        cy=py-poly[i].y;
        if (ex*cy - ey*cx < -1e-3f) return false;   // right of edge → outside
    }
    return true;
}

// --------------------------------------------------------------------------
struct FaceRect {
    int w,h, x,y;
    float minU,minV;
    Vector3 origin, axisU, axisV;
    std::vector<Vector2> poly2d;     // verts in luxel-space for inside test
};

static bool ShelfPack(std::vector<FaceRect>& r, int W, int& H) {
    std::vector<size_t> ord(r.size());
    for (size_t i=0;i<ord.size();++i) ord[i]=i;
    std::sort(ord.begin(),ord.end(),[&](size_t a,size_t b){return r[a].h>r[b].h;});
    int x=0,y=0,rowH=0;
    for (size_t i:ord) {
        if (r[i].w>W) return false;
        if (x+r[i].w>W){y+=rowH;x=0;rowH=0;}
        r[i].x=x;r[i].y=y;x+=r[i].w;if(r[i].h>rowH)rowH=r[i].h;
    }
    H=y+rowH; return true;
}

// --------------------------------------------------------------------------
LightmapAtlas BakeLightmap(const std::vector<MapPolygon>& polys,
                           const std::vector<PointLight>& lights)
{
    LightmapAtlas atlas;
    atlas.polyUV.resize(polys.size());

    // ----- 1. rects -------------------------------------------------------
    std::vector<FaceRect> rects(polys.size());
    for (size_t i=0;i<polys.size();++i) {
        const MapPolygon& p=polys[i];
        Vector3 U,V; FaceBasis(p.normal,U,V);
        float minU=1e30f,maxU=-1e30f,minV=1e30f,maxV=-1e30f;
        for (auto& vv:p.verts){
            float u=Vector3DotProduct(vv,U),v=Vector3DotProduct(vv,V);
            minU=std::min(minU,u);maxU=std::max(maxU,u);
            minV=std::min(minV,v);maxV=std::max(maxV,v);
        }
        FaceRect& r=rects[i];
        r.w=std::max(2,(int)ceilf((maxU-minU)/LUXEL_SIZE))+LM_PAD*2;
        r.h=std::max(2,(int)ceilf((maxV-minV)/LUXEL_SIZE))+LM_PAD*2;
        r.minU=minU;r.minV=minV;r.axisU=U;r.axisV=V;
        float d=Vector3DotProduct(p.verts[0],p.normal);
        r.origin=Vector3Add(Vector3Scale(p.normal,d),
                  Vector3Add(Vector3Scale(U,minU),Vector3Scale(V,minV)));
        // polygon verts in luxel-space for inside test
        r.poly2d.reserve(p.verts.size());
        for (auto& vv:p.verts)
            r.poly2d.push_back({(Vector3DotProduct(vv,U)-minU)/LUXEL_SIZE,
                                (Vector3DotProduct(vv,V)-minV)/LUXEL_SIZE});
    }

    // ----- 2. pack --------------------------------------------------------
    int W=256,H=0;
    while(!ShelfPack(rects,W,H)||H>W){W*=2;if(W>ATLAS_MAX){W=ATLAS_MAX;ShelfPack(rects,W,H);break;}}
    H=std::max(1,H);
    atlas.width=W;atlas.height=H;
    atlas.pixels.assign((size_t)W*H*4,0);

    OccluderSet occ=BuildOccluders(polys);
    size_t luxels=0; for(auto&r:rects)luxels+=(size_t)r.w*r.h;
    const int SAMPLES=AA_GRID*AA_GRID;
    printf("[Lightmap] atlas %dx%d, %zu faces, %zu lights, %zu tris, "
           "%zu luxels × %d samples = %zu rays/light\n",
           W,H,polys.size(),lights.size(),occ.tris.size(),
           luxels,SAMPLES,luxels*SAMPLES);

    // ----- 3. per-vertex UVs ---------------------------------------------
    for (size_t i=0;i<polys.size();++i){
        const MapPolygon&p=polys[i]; const FaceRect&r=rects[i];
        atlas.polyUV[i].uv.reserve(p.verts.size());
        for(auto&vv:p.verts){
            float u=(Vector3DotProduct(vv,r.axisU)-r.minU)/LUXEL_SIZE;
            float v=(Vector3DotProduct(vv,r.axisV)-r.minV)/LUXEL_SIZE;
            atlas.polyUV[i].uv.push_back({(r.x+LM_PAD+u+0.5f)/W,
                                          (r.y+LM_PAD+v+0.5f)/H});
        }
    }

    // ----- 4. path-traced bake -------------------------------------------
    // validity mask: 1 = luxel centre inside polygon (real surface)
    std::vector<uint8_t> valid((size_t)W*H,0);

    const float invG = 1.0f/(float)AA_GRID;
    for (size_t i=0;i<polys.size();++i){
        const MapPolygon&p=polys[i]; const FaceRect&r=rects[i];
        for(int ly=0;ly<r.h;++ly) for(int lx=0;lx<r.w;++lx){
            float cu=lx-LM_PAD+0.5f, cv=ly-LM_PAD+0.5f;
            bool inside=InsidePoly2D(r.poly2d,cu,cv);

            float ar=0,ag=0,ab=0;

            // stratified AA_GRID×AA_GRID grid — deterministic, no noise
            for(int sy=0;sy<AA_GRID;++sy) for(int sx=0;sx<AA_GRID;++sx){
                float ju = (lx-LM_PAD) + (sx+0.5f)*invG;
                float jv = (ly-LM_PAD) + (sy+0.5f)*invG;
                Vector3 wp=Vector3Add(r.origin,
                            Vector3Add(Vector3Scale(r.axisU,ju*LUXEL_SIZE),
                                       Vector3Scale(r.axisV,jv*LUXEL_SIZE)));
                Vector3 ro=Vector3Add(wp,Vector3Scale(p.normal,SHADOW_BIAS));

                float cr=AMBIENT,cg=AMBIENT,cb=AMBIENT;
                for(auto&L:lights){
                    Vector3 toL=Vector3Subtract(L.position,ro);
                    float dist=Vector3Length(toL);
                    if(dist>L.intensity||dist<1e-3f)continue;
                    Vector3 dir=Vector3Scale(toL,1.f/dist);
                    float ndl=Vector3DotProduct(p.normal,dir);
                    if(ndl<=0)continue;
                    if(Occluded(occ,ro,dir,dist-SHADOW_BIAS))continue;
                    float att=1.f-dist/L.intensity; att*=att;
                    cr+=L.color.x*ndl*att;
                    cg+=L.color.y*ndl*att;
                    cb+=L.color.z*ndl*att;
                }
                ar+=cr;ag+=cg;ab+=cb;
            }
            ar/=SAMPLES;ag/=SAMPLES;ab/=SAMPLES;

            size_t off=((size_t)(r.y+ly)*W+(r.x+lx))*4;
            auto C=[](float v){return(uint8_t)std::min(255,(int)(v*255.f));};
            atlas.pixels[off+0]=C(ar);
            atlas.pixels[off+1]=C(ag);
            atlas.pixels[off+2]=C(ab);
            atlas.pixels[off+3]=255;
            if(inside) valid[(size_t)(r.y+ly)*W+(r.x+lx)]=1;
        }
    }

    // ----- 5. dilate — flood invalid/padding luxels from valid neighbours -
    // This is what fixes the dark-corner bleed: padding luxels whose sample
    // point fell inside solid geometry get overwritten by the nearest real
    // surface sample, so bilinear filtering at runtime never pulls in black.
    for(int pass=0;pass<DILATE_PASSES;++pass){
        std::vector<uint8_t> nv=valid;
        for(int y=0;y<H;++y) for(int x=0;x<W;++x){
            size_t idx=(size_t)y*W+x;
            if(valid[idx])continue;
            int sr=0,sg=0,sb=0,n=0;
            const int dx[4]={-1,1,0,0},dy[4]={0,0,-1,1};
            for(int k=0;k<4;++k){
                int nx=x+dx[k],ny=y+dy[k];
                if(nx<0||ny<0||nx>=W||ny>=H)continue;
                size_t ni=(size_t)ny*W+nx;
                if(!valid[ni])continue;
                sr+=atlas.pixels[ni*4+0];
                sg+=atlas.pixels[ni*4+1];
                sb+=atlas.pixels[ni*4+2];n++;
            }
            if(n){
                atlas.pixels[idx*4+0]=(uint8_t)(sr/n);
                atlas.pixels[idx*4+1]=(uint8_t)(sg/n);
                atlas.pixels[idx*4+2]=(uint8_t)(sb/n);
                atlas.pixels[idx*4+3]=255;
                nv[idx]=1;
            }
        }
        valid.swap(nv);
    }

    return atlas;
}
