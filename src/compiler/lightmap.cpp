// lightmap.cpp  —  offline lightmap baker with optional GPU compute path.
//
// Atlas packing, per-vertex lightmap UV generation, validity masking and
// dilation remain on the CPU. The expensive per-luxel lighting and hard-shadow
// accumulation can run on a Sokol compute shader when the compiler is built on
// a backend that supports it. If compute initialization or dispatch fails, the
// baker falls back to the original CPU ray path.

#include "lightmap.h"
#include "lightmap_compute.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

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
struct OccluderSet {
    std::vector<LightmapComputeOccluderTri> tris;
};

static OccluderSet BuildOccluders(const std::vector<MapPolygon>& polys) {
    OccluderSet o;
    for (auto& p : polys) {
        for (size_t t = 1; t + 1 < p.verts.size(); ++t) {
            LightmapComputeOccluderTri tr{};
            tr.a = p.verts[0];
            tr.b = p.verts[t];
            tr.c = p.verts[t + 1];
            tr.bounds = AABBInvalid();
            tr.occluderGroup = p.occluderGroup;
            AABBExtend(&tr.bounds, tr.a);
            AABBExtend(&tr.bounds, tr.b);
            AABBExtend(&tr.bounds, tr.c);
            o.tris.push_back(tr);
        }
    }
    return o;
}

static bool RayTri(const Vector3& ro, const Vector3& rd,
                   const LightmapComputeOccluderTri& tr, float tmin, float tmax)
{
    Vector3 e1 = Vector3Subtract(tr.b, tr.a);
    Vector3 e2 = Vector3Subtract(tr.c, tr.a);
    Vector3 p  = Vector3CrossProduct(rd, e2);
    float det  = Vector3DotProduct(e1, p);
    if (fabsf(det) < RAY_EPS) return false;
    float inv = 1.0f / det;
    Vector3 s = Vector3Subtract(ro, tr.a);
    float u = Vector3DotProduct(s, p) * inv;      if (u < 0 || u > 1) return false;
    Vector3 q = Vector3CrossProduct(s, e1);
    float v = Vector3DotProduct(rd, q) * inv;     if (v < 0 || u + v > 1) return false;
    float t = Vector3DotProduct(e2, q) * inv;
    return t > tmin && t < tmax;
}

static bool RayAABB(const Vector3& ro, const Vector3& inv_rd,
                    const AABB& b, float tmax)
{
    float t1, t2, tn = 0, tf = tmax;
    t1 = (b.min.x - ro.x) * inv_rd.x; t2 = (b.max.x - ro.x) * inv_rd.x;
    tn = std::max(tn, std::min(t1, t2)); tf = std::min(tf, std::max(t1, t2));
    t1 = (b.min.y - ro.y) * inv_rd.y; t2 = (b.max.y - ro.y) * inv_rd.y;
    tn = std::max(tn, std::min(t1, t2)); tf = std::min(tf, std::max(t1, t2));
    t1 = (b.min.z - ro.z) * inv_rd.z; t2 = (b.max.z - ro.z) * inv_rd.z;
    tn = std::max(tn, std::min(t1, t2)); tf = std::min(tf, std::max(t1, t2));
    return tf >= tn;
}

static bool Occluded(const OccluderSet& o, const Vector3& ro,
                     const Vector3& rd, float dist, int ignoreOccluderGroup)
{
    Vector3 inv = {
        1.0f / (fabsf(rd.x) > RAY_EPS ? rd.x : RAY_EPS),
        1.0f / (fabsf(rd.y) > RAY_EPS ? rd.y : RAY_EPS),
        1.0f / (fabsf(rd.z) > RAY_EPS ? rd.z : RAY_EPS)
    };
    for (const auto& tri : o.tris) {
        if ((ignoreOccluderGroup >= 0) && (tri.occluderGroup == ignoreOccluderGroup)) {
            continue;
        }
        if (!RayAABB(ro, inv, tri.bounds, dist)) continue;
        if (RayTri(ro, rd, tri, RAY_EPS, dist)) return true;
    }
    return false;
}

// --------------------------------------------------------------------------
//  Point-in-convex-polygon test (2-D, polygon already CCW in plane space).
// --------------------------------------------------------------------------
static bool InsidePoly2D(const std::vector<Vector2>& poly, float px, float py) {
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const float ex = poly[j].x - poly[i].x;
        const float ey = poly[j].y - poly[i].y;
        const float cx = px - poly[i].x;
        const float cy = py - poly[i].y;
        if (ex * cy - ey * cx < -1e-3f) return false;
    }
    return true;
}

// --------------------------------------------------------------------------
struct FaceRect {
    LightmapComputeFaceRect gpu;
    std::vector<Vector2> poly2d;     // verts in luxel-space for inside test
};

static bool ShelfPack(std::vector<FaceRect>& r, int W, int& H) {
    std::vector<size_t> ord(r.size());
    for (size_t i = 0; i < ord.size(); ++i) ord[i] = i;
    std::sort(ord.begin(), ord.end(), [&](size_t a, size_t b) {
        return r[a].gpu.h > r[b].gpu.h;
    });
    int x = 0, y = 0, rowH = 0;
    for (size_t i : ord) {
        if (r[i].gpu.w > W) return false;
        if (x + r[i].gpu.w > W) {
            y += rowH;
            x = 0;
            rowH = 0;
        }
        r[i].gpu.x = x;
        r[i].gpu.y = y;
        x += r[i].gpu.w;
        if (r[i].gpu.h > rowH) rowH = r[i].gpu.h;
    }
    H = y + rowH;
    return true;
}

static std::vector<FaceRect> BuildFaceRects(const std::vector<MapPolygon>& polys) {
    std::vector<FaceRect> rects(polys.size());
    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& p = polys[i];
        Vector3 U, V;
        FaceBasis(p.normal, U, V);
        float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
        for (auto& vv : p.verts) {
            const float u = Vector3DotProduct(vv, U);
            const float v = Vector3DotProduct(vv, V);
            minU = std::min(minU, u); maxU = std::max(maxU, u);
            minV = std::min(minV, v); maxV = std::max(maxV, v);
        }
        FaceRect& r = rects[i];
        r.gpu.w = std::max(2, (int)ceilf((maxU - minU) / LUXEL_SIZE)) + LM_PAD * 2;
        r.gpu.h = std::max(2, (int)ceilf((maxV - minV) / LUXEL_SIZE)) + LM_PAD * 2;
        r.gpu.minU = minU;
        r.gpu.minV = minV;
        r.gpu.axisU = U;
        r.gpu.axisV = V;
        r.gpu.normal = p.normal;
        const float d = Vector3DotProduct(p.verts[0], p.normal);
        r.gpu.origin = Vector3Add(
            Vector3Scale(p.normal, d),
            Vector3Add(Vector3Scale(U, minU), Vector3Scale(V, minV)));
        r.poly2d.reserve(p.verts.size());
        for (auto& vv : p.verts) {
            r.poly2d.push_back({
                (Vector3DotProduct(vv, U) - minU) / LUXEL_SIZE,
                (Vector3DotProduct(vv, V) - minV) / LUXEL_SIZE
            });
        }
    }
    return rects;
}

static void FillPolyUVs(const std::vector<MapPolygon>& polys,
                        const std::vector<FaceRect>& rects,
                        int W, int H,
                        LightmapAtlas& atlas)
{
    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& p = polys[i];
        const FaceRect& r = rects[i];
        atlas.polyUV[i].uv.reserve(p.verts.size());
        for (auto& vv : p.verts) {
            const float u = (Vector3DotProduct(vv, r.gpu.axisU) - r.gpu.minU) / LUXEL_SIZE;
            const float v = (Vector3DotProduct(vv, r.gpu.axisV) - r.gpu.minV) / LUXEL_SIZE;
            atlas.polyUV[i].uv.push_back({
                (r.gpu.x + LM_PAD + u + 0.5f) / W,
                (r.gpu.y + LM_PAD + v + 0.5f) / H
            });
        }
    }
}

static std::vector<uint8_t> BuildValidMask(const std::vector<FaceRect>& rects, int W, int H) {
    std::vector<uint8_t> valid((size_t)W * H, 0);
    for (const FaceRect& r : rects) {
        for (int ly = 0; ly < r.gpu.h; ++ly) {
            for (int lx = 0; lx < r.gpu.w; ++lx) {
                const float cu = lx - LM_PAD + 0.5f;
                const float cv = ly - LM_PAD + 0.5f;
                if (!InsidePoly2D(r.poly2d, cu, cv)) continue;
                valid[(size_t)(r.gpu.y + ly) * W + (r.gpu.x + lx)] = 1;
            }
        }
    }
    return valid;
}

static void DilateAtlas(LightmapAtlas& atlas, std::vector<uint8_t>& valid) {
    const int W = atlas.width;
    const int H = atlas.height;
    for (int pass = 0; pass < DILATE_PASSES; ++pass) {
        std::vector<uint8_t> nv = valid;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t idx = (size_t)y * W + x;
                if (valid[idx]) continue;
                int sr = 0, sg = 0, sb = 0, n = 0;
                const int dx[4] = {-1, 1, 0, 0};
                const int dy[4] = {0, 0, -1, 1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = x + dx[k];
                    const int ny = y + dy[k];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    const size_t ni = (size_t)ny * W + nx;
                    if (!valid[ni]) continue;
                    sr += atlas.pixels[ni * 4 + 0];
                    sg += atlas.pixels[ni * 4 + 1];
                    sb += atlas.pixels[ni * 4 + 2];
                    n++;
                }
                if (n) {
                    atlas.pixels[idx * 4 + 0] = (uint8_t)(sr / n);
                    atlas.pixels[idx * 4 + 1] = (uint8_t)(sg / n);
                    atlas.pixels[idx * 4 + 2] = (uint8_t)(sb / n);
                    atlas.pixels[idx * 4 + 3] = 255;
                    nv[idx] = 1;
                }
            }
        }
        valid.swap(nv);
    }
}

static void BakeLightmapCPU(const std::vector<MapPolygon>& polys,
                            const std::vector<PointLight>& lights,
                            const std::vector<FaceRect>& rects,
                            const OccluderSet& occ,
                            LightmapAtlas& atlas)
{
    const int W = atlas.width;
    const int SAMPLES = AA_GRID * AA_GRID;
    const float invG = 1.0f / (float)AA_GRID;

    for (size_t i = 0; i < polys.size(); ++i) {
        const MapPolygon& p = polys[i];
        const FaceRect& r = rects[i];
        for (int ly = 0; ly < r.gpu.h; ++ly) {
            for (int lx = 0; lx < r.gpu.w; ++lx) {
                float ar = 0, ag = 0, ab = 0;

                for (int sy = 0; sy < AA_GRID; ++sy) {
                    for (int sx = 0; sx < AA_GRID; ++sx) {
                        const float ju = (lx - LM_PAD) + (sx + 0.5f) * invG;
                        const float jv = (ly - LM_PAD) + (sy + 0.5f) * invG;
                        const Vector3 wp = Vector3Add(
                            r.gpu.origin,
                            Vector3Add(Vector3Scale(r.gpu.axisU, ju * LUXEL_SIZE),
                                       Vector3Scale(r.gpu.axisV, jv * LUXEL_SIZE)));
                        const Vector3 ro = Vector3Add(wp, Vector3Scale(p.normal, SHADOW_BIAS));

                        float cr = AMBIENT, cg = AMBIENT, cb = AMBIENT;
                        for (auto& L : lights) {
                            const Vector3 toL = Vector3Subtract(L.position, ro);
                            const float dist = Vector3Length(toL);
                            if (dist > L.intensity || dist < 1e-3f) continue;
                            const Vector3 dir = Vector3Scale(toL, 1.f / dist);
                            float emit = 1.0f;
                            if (L.directional) {
                                const Vector3 lightToSurface = Vector3Scale(dir, -1.0f);
                                emit = Vector3DotProduct(L.emissionNormal, lightToSurface);
                                if (emit <= 0.0f) continue;
                            }
                            const float ndl = Vector3DotProduct(p.normal, dir);
                            if (ndl <= 0) continue;
                            if (Occluded(occ, ro, dir, dist - SHADOW_BIAS, L.ignoreOccluderGroup)) continue;
                            float att = 1.f - dist / L.intensity;
                            att *= att;
                            const float contrib = emit * ndl * att;
                            cr += L.color.x * contrib;
                            cg += L.color.y * contrib;
                            cb += L.color.z * contrib;
                        }
                        ar += cr; ag += cg; ab += cb;
                    }
                }
                ar /= SAMPLES; ag /= SAMPLES; ab /= SAMPLES;

                const size_t off = ((size_t)(r.gpu.y + ly) * W + (r.gpu.x + lx)) * 4;
                auto C = [](float v) { return (uint8_t)std::min(255, (int)(v * 255.f)); };
                atlas.pixels[off + 0] = C(ar);
                atlas.pixels[off + 1] = C(ag);
                atlas.pixels[off + 2] = C(ab);
                atlas.pixels[off + 3] = 255;
            }
        }
    }
}

// --------------------------------------------------------------------------
LightmapAtlas BakeLightmap(const std::vector<MapPolygon>& polys,
                           const std::vector<PointLight>& lights)
{
    LightmapAtlas atlas;
    atlas.polyUV.resize(polys.size());

    std::vector<FaceRect> rects = BuildFaceRects(polys);

    int W = 256, H = 0;
    while (!ShelfPack(rects, W, H) || H > W) {
        W *= 2;
        if (W > ATLAS_MAX) {
            W = ATLAS_MAX;
            ShelfPack(rects, W, H);
            break;
        }
    }
    H = std::max(1, H);
    atlas.width = W;
    atlas.height = H;
    atlas.pixels.assign((size_t)W * H * 4, 0);

    FillPolyUVs(polys, rects, W, H, atlas);

    size_t luxels = 0;
    for (const FaceRect& r : rects) {
        luxels += (size_t)r.gpu.w * r.gpu.h;
    }
    const int SAMPLES = AA_GRID * AA_GRID;

    OccluderSet occ = BuildOccluders(polys);
    printf("[Lightmap] atlas %dx%d, %zu faces, %zu lights, %zu tris, "
           "%zu luxels x %d samples = %zu rays/light\n",
           W, H, polys.size(), lights.size(), occ.tris.size(),
           luxels, SAMPLES, luxels * SAMPLES);

    std::vector<uint8_t> valid = BuildValidMask(rects, W, H);

    std::vector<LightmapComputeFaceRect> gpuRects;
    gpuRects.reserve(rects.size());
    for (const FaceRect& r : rects) {
        gpuRects.push_back(r.gpu);
    }

    std::string computeError;
    if (!BakeLightmapCompute(gpuRects, occ.tris, lights, W, H, atlas.pixels, &computeError)) {
        printf("[Lightmap] compute bake unavailable, falling back to CPU: %s\n", computeError.c_str());
        BakeLightmapCPU(polys, lights, rects, occ, atlas);
    }

    DilateAtlas(atlas, valid);
    return atlas;
}
