// wmath.h  — drop-in replacement for the subset of raymath used by Warped-Engine
// Keeps raylib struct names/layouts so existing code compiles unchanged.
#pragma once
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0f)
#endif
#ifndef RAD2DEG
#define RAD2DEG (180.0f / PI)
#endif

typedef struct Vector2 {
    float x, y;
} Vector2;

typedef struct Vector3 {
    float x, y, z;
} Vector3;

typedef struct Vector4 {
    float x, y, z, w;
} Vector4;

// Column-major 4×4 matrix — m[col][row].  m[16] accessor for uniform upload.
typedef struct Matrix {
    float m0,  m4,  m8,  m12;
    float m1,  m5,  m9,  m13;
    float m2,  m6,  m10, m14;
    float m3,  m7,  m11, m15;
} Matrix;

typedef struct Color {
    unsigned char r, g, b, a;
} Color;

#define WCOLOR(R,G,B,A) (Color){ (unsigned char)(R),(unsigned char)(G),(unsigned char)(B),(unsigned char)(A) }

// -----------------------------------------------------------------------------
// Vector2
// -----------------------------------------------------------------------------
static inline Vector2 Vector2Zero(void) { return (Vector2){0,0}; }

// -----------------------------------------------------------------------------
// Vector3
// -----------------------------------------------------------------------------
static inline Vector3 Vector3Zero(void)                          { return (Vector3){0,0,0}; }
static inline Vector3 Vector3Add(Vector3 a, Vector3 b)           { return (Vector3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vector3 Vector3Subtract(Vector3 a, Vector3 b)      { return (Vector3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline Vector3 Vector3Scale(Vector3 v, float s)           { return (Vector3){v.x*s, v.y*s, v.z*s}; }
static inline float   Vector3DotProduct(Vector3 a, Vector3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vector3 Vector3CrossProduct(Vector3 a, Vector3 b)  { return (Vector3){ a.y*b.z - a.z*b.y,
                                                                                     a.z*b.x - a.x*b.z,
                                                                                     a.x*b.y - a.y*b.x }; }
static inline float   Vector3LengthSq(Vector3 v)                 { return Vector3DotProduct(v,v); }
static inline float   Vector3Length(Vector3 v)                   { return sqrtf(Vector3LengthSq(v)); }
static inline Vector3 Vector3Normalize(Vector3 v)                { float l = Vector3Length(v);
                                                                   return (l > 1e-9f) ? Vector3Scale(v, 1.0f/l) : v; }
static inline Vector3 Vector3Negate(Vector3 v)                   { return (Vector3){ -v.x, -v.y, -v.z }; }

// C++ operator helpers mirroring the raymath ones used in player.cpp
#ifdef __cplusplus
static inline Vector3 operator+(Vector3 a, Vector3 b) { return Vector3Add(a,b); }
static inline Vector3 operator-(Vector3 a, Vector3 b) { return Vector3Subtract(a,b); }
static inline Vector3 operator*(Vector3 v, float s)   { return Vector3Scale(v,s); }
#endif

// -----------------------------------------------------------------------------
// Matrix
// -----------------------------------------------------------------------------
static inline Matrix MatrixIdentity(void) {
    Matrix r = {0};
    r.m0 = r.m5 = r.m10 = r.m15 = 1.0f;
    return r;
}

static inline Matrix MatrixMultiply(Matrix a, Matrix b) {
    Matrix r;
    // r = a * b  (column-major, same convention as raymath)
    r.m0  = a.m0*b.m0  + a.m4*b.m1  + a.m8 *b.m2  + a.m12*b.m3;
    r.m1  = a.m1*b.m0  + a.m5*b.m1  + a.m9 *b.m2  + a.m13*b.m3;
    r.m2  = a.m2*b.m0  + a.m6*b.m1  + a.m10*b.m2  + a.m14*b.m3;
    r.m3  = a.m3*b.m0  + a.m7*b.m1  + a.m11*b.m2  + a.m15*b.m3;

    r.m4  = a.m0*b.m4  + a.m4*b.m5  + a.m8 *b.m6  + a.m12*b.m7;
    r.m5  = a.m1*b.m4  + a.m5*b.m5  + a.m9 *b.m6  + a.m13*b.m7;
    r.m6  = a.m2*b.m4  + a.m6*b.m5  + a.m10*b.m6  + a.m14*b.m7;
    r.m7  = a.m3*b.m4  + a.m7*b.m5  + a.m11*b.m6  + a.m15*b.m7;

    r.m8  = a.m0*b.m8  + a.m4*b.m9  + a.m8 *b.m10 + a.m12*b.m11;
    r.m9  = a.m1*b.m8  + a.m5*b.m9  + a.m9 *b.m10 + a.m13*b.m11;
    r.m10 = a.m2*b.m8  + a.m6*b.m9  + a.m10*b.m10 + a.m14*b.m11;
    r.m11 = a.m3*b.m8  + a.m7*b.m9  + a.m11*b.m10 + a.m15*b.m11;

    r.m12 = a.m0*b.m12 + a.m4*b.m13 + a.m8 *b.m14 + a.m12*b.m15;
    r.m13 = a.m1*b.m12 + a.m5*b.m13 + a.m9 *b.m14 + a.m13*b.m15;
    r.m14 = a.m2*b.m12 + a.m6*b.m13 + a.m10*b.m14 + a.m14*b.m15;
    r.m15 = a.m3*b.m12 + a.m7*b.m13 + a.m11*b.m14 + a.m15*b.m15;
    return r;
}

static inline Matrix MatrixTranslate(float x, float y, float z) {
    Matrix r = MatrixIdentity();
    r.m12 = x;  r.m13 = y;  r.m14 = z;
    return r;
}

// Right-handed perspective (GL clip [-1,1] on Z)
static inline Matrix MatrixPerspective(float fovyRad, float aspect, float znear, float zfar) {
    Matrix r = {0};
    float f = 1.0f / tanf(fovyRad * 0.5f);
    r.m0  = f / aspect;
    r.m5  = f;
    r.m10 = (zfar + znear) / (znear - zfar);
    r.m11 = -1.0f;
    r.m14 = (2.0f * zfar * znear) / (znear - zfar);
    return r;
}

static inline Matrix MatrixLookAt(Vector3 eye, Vector3 target, Vector3 up) {
    Vector3 f = Vector3Normalize(Vector3Subtract(target, eye));
    Vector3 s = Vector3Normalize(Vector3CrossProduct(f, up));
    Vector3 u = Vector3CrossProduct(s, f);
    Matrix r = MatrixIdentity();
    r.m0 =  s.x;  r.m4 =  s.y;  r.m8  =  s.z;
    r.m1 =  u.x;  r.m5 =  u.y;  r.m9  =  u.z;
    r.m2 = -f.x;  r.m6 = -f.y;  r.m10 = -f.z;
    r.m12 = -Vector3DotProduct(s, eye);
    r.m13 = -Vector3DotProduct(u, eye);
    r.m14 =  Vector3DotProduct(f, eye);
    return r;
}

// Pack into float[16] column-major for GL uniform upload.
typedef struct float16 { float v[16]; } float16;
static inline float16 MatrixToFloat16(Matrix m) {
    float16 r;
    r.v[0]=m.m0;  r.v[4]=m.m4;  r.v[8]=m.m8;   r.v[12]=m.m12;
    r.v[1]=m.m1;  r.v[5]=m.m5;  r.v[9]=m.m9;   r.v[13]=m.m13;
    r.v[2]=m.m2;  r.v[6]=m.m6;  r.v[10]=m.m10; r.v[14]=m.m14;
    r.v[3]=m.m3;  r.v[7]=m.m7;  r.v[11]=m.m11; r.v[15]=m.m15;
    return r;
}

// -----------------------------------------------------------------------------
// AABB + frustum  (for culling)
// -----------------------------------------------------------------------------
typedef struct AABB { Vector3 min, max; } AABB;

static inline AABB  AABBInvalid(void)             { return (AABB){{ 1e30f, 1e30f, 1e30f},{-1e30f,-1e30f,-1e30f}}; }
static inline void  AABBExtend(AABB* b, Vector3 p){ if(p.x<b->min.x)b->min.x=p.x; if(p.y<b->min.y)b->min.y=p.y; if(p.z<b->min.z)b->min.z=p.z;
                                                    if(p.x>b->max.x)b->max.x=p.x; if(p.y>b->max.y)b->max.y=p.y; if(p.z>b->max.z)b->max.z=p.z; }

// Six clip-space planes extracted from a column-major VP matrix (GL, Z in [-1,1]).
// Normals point inward.  Gribb-Hartmann.
typedef struct Frustum { Vector4 p[6]; } Frustum;

static inline Vector4 _planeNormalize(Vector4 pl) {
    float l = sqrtf(pl.x*pl.x + pl.y*pl.y + pl.z*pl.z);
    return (l > 1e-9f) ? (Vector4){pl.x/l, pl.y/l, pl.z/l, pl.w/l} : pl;
}

static inline Frustum FrustumFromVP(Matrix vp) {
    // clip = vp * pos  (column-major).  rowN = {m[N],m[N+4],m[N+8],m[N+12]}.
    float r0x=vp.m0, r0y=vp.m4, r0z=vp.m8,  r0w=vp.m12;   // x clip
    float r1x=vp.m1, r1y=vp.m5, r1z=vp.m9,  r1w=vp.m13;   // y clip
    float r2x=vp.m2, r2y=vp.m6, r2z=vp.m10, r2w=vp.m14;   // z clip
    float r3x=vp.m3, r3y=vp.m7, r3z=vp.m11, r3w=vp.m15;   // w clip
    Frustum f;
    f.p[0] = _planeNormalize((Vector4){r3x+r0x, r3y+r0y, r3z+r0z, r3w+r0w}); // left
    f.p[1] = _planeNormalize((Vector4){r3x-r0x, r3y-r0y, r3z-r0z, r3w-r0w}); // right
    f.p[2] = _planeNormalize((Vector4){r3x+r1x, r3y+r1y, r3z+r1z, r3w+r1w}); // bottom
    f.p[3] = _planeNormalize((Vector4){r3x-r1x, r3y-r1y, r3z-r1z, r3w-r1w}); // top
    f.p[4] = _planeNormalize((Vector4){r3x+r2x, r3y+r2y, r3z+r2z, r3w+r2w}); // near
    f.p[5] = _planeNormalize((Vector4){r3x-r2x, r3y-r2y, r3z-r2z, r3w-r2w}); // far
    return f;
}

// Conservative AABB vs frustum — reject only if fully outside any plane.
static inline int FrustumAABB(const Frustum* f, AABB b) {
    for (int i=0;i<6;++i) {
        // positive vertex: pick max for components where normal >= 0, else min
        Vector4 pl = f->p[i];
        float px = (pl.x >= 0.0f) ? b.max.x : b.min.x;
        float py = (pl.y >= 0.0f) ? b.max.y : b.min.y;
        float pz = (pl.z >= 0.0f) ? b.max.z : b.min.z;
        if (pl.x*px + pl.y*py + pl.z*pz + pl.w < 0.0f) return 0;   // outside
    }
    return 1;  // intersects or inside
}
