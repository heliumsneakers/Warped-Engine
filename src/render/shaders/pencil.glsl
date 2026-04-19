@module warped_pencil_shader

@vs vs_pencil
layout(location=0) in vec2 a_pos;
layout(location=1) in vec2 a_uv;

out vec2 v_uv;

void main() {
    v_uv = a_uv;
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
@end

@fs fs_pencil
layout(binding=0) uniform fs_params {
    vec4 u_resolution_time; // xy = framebuffer size, z = time
    vec4 u_effect_params;   // x = edge gain, y = far line radius px, z = near line radius px, w = scene mix
};

layout(binding=0) uniform texture2D u_scene;
layout(binding=1) uniform texture2D u_normals;
layout(binding=2) uniform texture2D u_depth;
layout(binding=0) uniform sampler u_scene_smp;
layout(binding=1) uniform sampler u_normal_smp;
layout(binding=2) uniform sampler u_depth_smp;

in vec2 v_uv;
out vec4 frag_color;

float luminance(vec3 c) {
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec4 normal_sample(vec2 uv) {
    vec4 n = texture(sampler2D(u_normals, u_normal_smp), uv);
    return vec4(n.xyz * 2.0 - 1.0, n.a);
}

float depth_sample(vec2 uv) {
    return texture(sampler2D(u_depth, u_depth_smp), uv).r;
}

float normal_delta(vec4 center, vec4 sample_n) {
    float center_occ = step(0.0001, center.a);
    float sample_occ = step(0.0001, sample_n.a);
    float silhouette = abs(center_occ - sample_occ);
    float crease = length(center.xyz - sample_n.xyz) * center_occ * sample_occ;
    return max(silhouette, crease);
}

float geometry_edge(vec2 uv, vec2 texel, float radius, vec4 center_n, float center_d) {
    float center_occ = step(0.0001, center_n.a);

    vec4 s0 = normal_sample(uv + texel * vec2( radius, 0.0));
    vec4 s1 = normal_sample(uv + texel * vec2(-radius, 0.0));
    vec4 s2 = normal_sample(uv + texel * vec2(0.0,  radius));
    vec4 s3 = normal_sample(uv + texel * vec2(0.0, -radius));

    float nd0 = normal_delta(center_n, s0);
    float nd1 = normal_delta(center_n, s1);
    float nd2 = normal_delta(center_n, s2);
    float nd3 = normal_delta(center_n, s3);

    float support = step(0.12, nd0) + step(0.12, nd1) + step(0.12, nd2) + step(0.12, nd3);
    float normal_e = max(max(nd0, nd1), max(nd2, nd3)) * step(1.0, support);

    float d0 = depth_sample(uv + texel * vec2( radius, 0.0));
    float d1 = depth_sample(uv + texel * vec2(-radius, 0.0));
    float d2 = depth_sample(uv + texel * vec2(0.0,  radius));
    float d3 = depth_sample(uv + texel * vec2(0.0, -radius));

    float laplacian_h = abs(d0 + d1 - 2.0 * center_d);
    float laplacian_v = abs(d2 + d3 - 2.0 * center_d);
    float depth_e = smoothstep(0.008, 0.03, max(laplacian_h, laplacian_v) / max(center_d, 0.0001));
    float max_rel_depth = max(max(abs(d0 - center_d), abs(d1 - center_d)),
                              max(abs(d2 - center_d), abs(d3 - center_d))) / max(center_d, 0.0001);
    float max_normal_delta = max(max(nd0, nd1), max(nd2, nd3));
    float same_surface_noise = (1.0 - step(0.08, max_normal_delta)) *
                               (1.0 - smoothstep(0.015, 0.08, max_rel_depth));
    depth_e *= 1.0 - same_surface_noise;

    return max(normal_e, depth_e) * center_occ;
}

void main() {
    vec2 resolution = max(u_resolution_time.xy, vec2(1.0));
    vec2 texel = 1.0 / resolution;
    vec2 uv = clamp(v_uv, texel, 1.0 - texel);

    vec4 scene = texture(sampler2D(u_scene, u_scene_smp), uv);
    vec4 center_n = normal_sample(uv);
    float center_d = depth_sample(uv);
    float lum = luminance(scene.rgb);

    float edge_gain = max(u_effect_params.x, 0.01);
    float far_radius = max(u_effect_params.y, 0.25);
    float near_radius = max(u_effect_params.z, far_radius);
    float scene_mix = clamp(u_effect_params.w, 0.0, 1.0);

    float view_dist = center_d * 4096.0;
    float near_factor = clamp(1.0 - smoothstep(96.0, 896.0, view_dist), 0.0, 1.0);
    float radius = mix(far_radius, near_radius, near_factor);
    float edge_response = geometry_edge(uv, texel, radius, center_n, center_d);
    float edge = smoothstep(0.16, 0.42, edge_response * edge_gain);

    vec3 paper_color = vec3(0.965, 0.935, 0.865);
    vec3 graphite = vec3(0.105, 0.092, 0.082);
    vec3 washed_scene = mix(vec3(lum), scene.rgb, 0.34);
    vec3 base = mix(paper_color, washed_scene, scene_mix);

    float pencil = clamp(edge * 0.95, 0.0, 1.0) * step(0.0001, center_n.a);
    vec3 color = mix(base, graphite, pencil);

    frag_color = vec4(color, scene.a);
}
@end

@program pencil_post vs_pencil fs_pencil
