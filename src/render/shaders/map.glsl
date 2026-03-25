@module warped_map_shader

@vs vs_map
layout(binding=0) uniform vs_params {
    mat4 u_mvp;
    mat4 u_model;
};

layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_nrm;
layout(location=2) in vec2 a_uv;
layout(location=3) in vec2 a_lmuv;

out vec3 v_nrm;
out vec2 v_uv;
out vec2 v_lmuv;

void main() {
    v_nrm = mat3(u_model) * a_nrm;
    v_uv = a_uv;
    v_lmuv = a_lmuv;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
@end

@fs fs_map
layout(binding=0) uniform texture2D u_tex;
layout(binding=1) uniform texture2D u_lm;
layout(binding=0) uniform sampler u_tex_smp;
layout(binding=1) uniform sampler u_lm_smp;

in vec3 v_nrm;
in vec2 v_uv;
in vec2 v_lmuv;

out vec4 frag_color;

void main() {
    vec4 c = texture(sampler2D(u_tex, u_tex_smp), v_uv);
    vec3 lm = texture(sampler2D(u_lm, u_lm_smp), v_lmuv).rgb;
    frag_color = vec4(c.rgb * lm * 2.0, c.a);
}
@end

@program map vs_map fs_map
