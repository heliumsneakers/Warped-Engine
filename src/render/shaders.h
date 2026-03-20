// shaders.h  —  GLSL 330 sources for the GL 3.3 sokol backend.
// Textured + simple N·L diffuse.
#pragma once

static const char* WARPED_VS_SRC =
    "#version 330\n"
    "uniform mat4 u_mvp;\n"
    "uniform mat4 u_model;\n"
    "layout(location=0) in vec3 a_pos;\n"
    "layout(location=1) in vec3 a_nrm;\n"
    "layout(location=2) in vec2 a_uv;\n"
    "out vec3 v_nrm;\n"
    "out vec2 v_uv;\n"
    "void main() {\n"
    "    v_nrm = mat3(u_model) * a_nrm;\n"
    "    v_uv  = a_uv;\n"
    "    gl_Position = u_mvp * vec4(a_pos, 1.0);\n"
    "}\n";

static const char* WARPED_FS_SRC =
    "#version 330\n"
    "uniform sampler2D u_tex;\n"
    "in vec3 v_nrm;\n"
    "in vec2 v_uv;\n"
    "out vec4 frag_color;\n"
    "void main() {\n"
    "    vec3  N = normalize(v_nrm);\n"
    "    vec3  L = normalize(vec3(0.4, 1.0, 0.3));\n"
    "    float d = max(dot(N, L), 0.0) * 0.7 + 0.3;\n"
    "    vec4  c = texture(u_tex, v_uv);\n"
    "    frag_color = vec4(c.rgb * d, c.a);\n"
    "}\n";
