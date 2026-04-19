// shaders.h  —  GLSL 330 sources for the GL 3.3 sokol backend.
// Diffuse texture × baked lightmap.
#pragma once

static const char* WARPED_VS_SRC =
    "#version 330\n"
    "uniform mat4 u_mvp;\n"
    "uniform mat4 u_model;\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec3 a_nrm;\n"
    "layout(location=2) in vec2 a_uv;\n"
    "layout(location=3) in vec2 a_lmuv;\n"
    "out vec3 v_nrm;\n"
    "out vec2 v_uv;\n"
    "out vec2 v_lmuv;\n"
    "void main() {\n"
    "    v_nrm  = mat3(u_model) * a_nrm;\n"
    "    v_uv   = a_uv;\n"
    "    v_lmuv = a_lmuv;\n"
    "    gl_Position = u_mvp * vec4(a_pos, 1.0);\n"
    "}\n";

static const char* WARPED_FS_SRC =
    "#version 330\n"
    "uniform sampler2D u_tex;\n"
    "uniform sampler2D u_lm;\n"
    "in vec3 v_nrm;\n"
    "in vec2 v_uv;\n"
    "in vec2 v_lmuv;\n"
    "out vec4 frag_color;\n"
    "void main() {\n"
    "    vec4 c  = texture(u_tex, v_uv);\n"
    "    vec3 lm = texture(u_lm,  v_lmuv).rgb;\n"
    "    frag_color = vec4(c.rgb * lm * 2.0, c.a);\n"
    "}\n";
