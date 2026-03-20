#include "debug_draw.h"

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_log.h"
#include "sokol_gl.h"

#include <math.h>

static sgl_pipeline g_dbg_pipe = {};

void Debug_Init(void) {
    sgl_desc_t desc = {};
    desc.logger.func = slog_func;
    sgl_setup(&desc);

    // pipeline with depth testing so debug geo sits in the scene
    sg_pipeline_desc pd = {};
    pd.depth.compare       = SG_COMPAREFUNC_LESS_EQUAL;
    pd.depth.write_enabled = true;
    g_dbg_pipe = sgl_make_pipeline(&pd);
}

void Debug_Shutdown(void) {
    sgl_destroy_pipeline(g_dbg_pipe);
    sgl_shutdown();
}

void Debug_NewFrame(void) {
    sgl_defaults();
    sgl_load_pipeline(g_dbg_pipe);
    sgl_viewport(0, 0, sapp_width(), sapp_height(), true);
}

void Debug_SetCamera(const Matrix& proj, const Matrix& view) {
    sgl_matrix_mode_projection();
    float16 p = MatrixToFloat16(proj);
    sgl_load_matrix(p.v);

    sgl_matrix_mode_modelview();
    float16 v = MatrixToFloat16(view);
    sgl_load_matrix(v.v);
}

void Debug_Flush(void) {
    sgl_draw();
}

static inline void sglc(Color c) { sgl_c4b(c.r, c.g, c.b, c.a); }

void Debug_Line(Vector3 a, Vector3 b, Color c) {
    sgl_begin_lines();
        sglc(c);
        sgl_v3f(a.x, a.y, a.z);
        sgl_v3f(b.x, b.y, b.z);
    sgl_end();
}

void Debug_WireBox(Vector3 c, Vector3 he, Color col) {
    Vector3 mn = { c.x-he.x, c.y-he.y, c.z-he.z };
    Vector3 mx = { c.x+he.x, c.y+he.y, c.z+he.z };

    Vector3 v[8] = {
        {mn.x,mn.y,mn.z},{mx.x,mn.y,mn.z},{mx.x,mx.y,mn.z},{mn.x,mx.y,mn.z},
        {mn.x,mn.y,mx.z},{mx.x,mn.y,mx.z},{mx.x,mx.y,mx.z},{mn.x,mx.y,mx.z}
    };
    const int e[12][2] = {
        {0,1},{1,2},{2,3},{3,0},   // bottom
        {4,5},{5,6},{6,7},{7,4},   // top
        {0,4},{1,5},{2,6},{3,7}    // verticals
    };

    sgl_begin_lines();
        sglc(col);
        for (int i=0;i<12;++i) {
            sgl_v3f(v[e[i][0]].x, v[e[i][0]].y, v[e[i][0]].z);
            sgl_v3f(v[e[i][1]].x, v[e[i][1]].y, v[e[i][1]].z);
        }
    sgl_end();
}

void Debug_Sphere(Vector3 center, float r, Color col, int rings, int slices) {
    sgl_begin_lines();
    sglc(col);
    for (int i = 0; i <= rings; ++i) {
        float lat0 = PI * (-0.5f + (float)(i-1) / rings);
        float lat1 = PI * (-0.5f + (float) i     / rings);
        float y0 = sinf(lat0)*r, y1 = sinf(lat1)*r;
        float c0 = cosf(lat0)*r, c1 = cosf(lat1)*r;
        for (int j = 0; j < slices; ++j) {
            float lngA = 2.0f*PI * (float) j   /slices;
            float lngB = 2.0f*PI * (float)(j+1)/slices;
            float xA=cosf(lngA),zA=sinf(lngA), xB=cosf(lngB),zB=sinf(lngB);

            // meridian segment
            sgl_v3f(center.x+xA*c0, center.y+y0, center.z+zA*c0);
            sgl_v3f(center.x+xA*c1, center.y+y1, center.z+zA*c1);
            // latitude segment
            sgl_v3f(center.x+xA*c1, center.y+y1, center.z+zA*c1);
            sgl_v3f(center.x+xB*c1, center.y+y1, center.z+zB*c1);
        }
    }
    sgl_end();
}

void Debug_Point(Vector3 p, float size, Color col) {
    Debug_WireBox(p, (Vector3){size*0.5f,size*0.5f,size*0.5f}, col);
}
