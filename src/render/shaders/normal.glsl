@module warped_normal_shader

@vs vs_normal
layout(binding=0) uniform vs_params {
    mat4 u_mvp;
    mat4 u_normal_model;
};

layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_nrm;

out vec3 v_nrm;
out float v_view_dist;

void main() {
    v_nrm = normalize(mat3(u_normal_model) * a_nrm);
    vec4 view_pos = u_normal_model * vec4(a_pos, 1.0);
    v_view_dist = max(-view_pos.z, 0.0);
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
@end

@fs fs_normal
in vec3 v_nrm;
in float v_view_dist;

layout(location=0) out vec4 frag_color;
layout(location=1) out vec4 frag_depth_out;

void main() {
    vec3 n = normalize(v_nrm) * 0.5 + 0.5;
    frag_color = vec4(n, 1.0);
    frag_depth_out = vec4(v_view_dist / 4096.0, 0.0, 0.0, 0.0);
}
@end

@program normal_pass vs_normal fs_normal
