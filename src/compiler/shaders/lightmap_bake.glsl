@module warped_lightmap_bake

@cs cs_bake
layout(binding=0) uniform cs_face_params {
    int atlas_width;
    int atlas_height;
    int rect_x;
    int rect_y;
    int rect_w;
    int rect_h;
    int light_count;
    int tri_count;
    float min_u;
    float min_v;
    float luxel_size;
    float shadow_bias;
    vec3 ambient_color;
    float ray_eps;
    int global_dirt;
    int dirt_mode;
    int phong_neighbor_count;
    int _pad_scalar0;
    float dirt_depth;
    float dirt_scale;
    float dirt_gain;
    float dirt_angle;
    vec3 origin;
    float _pad0;
    vec3 axis_u;
    float _pad1;
    vec3 axis_v;
    float _pad2;
    vec3 normal;
    float _pad3;
    int poly_count;
    vec3 _pad_poly_count;
    vec4 phong_base_normal_weight;
    vec4 poly_verts[32];
    vec4 phong_neighbor_edge_a[32];
    vec4 phong_neighbor_edge_b[32];
    vec4 phong_neighbor_normal_weight[32];
};

struct point_light {
    vec3 position;
    float intensity;
    vec3 color;
    float angle_scale;
    vec3 emission_normal;
    int directional;
    vec3 parallel_direction;
    int parallel;
    vec3 spot_direction;
    float spot_outer_cos;
    int ignore_occluder_group;
    int attenuation_mode;
    float spot_inner_cos;
    int dirt;
    float dirt_scale;
    float dirt_gain;
    float _pad0;
    float _pad1;
};

struct occluder_tri {
    vec3 a;
    float _pad0;
    vec3 b;
    float _pad1;
    vec3 c;
    float _pad2;
    vec3 bounds_min;
    int occluder_group;
    vec3 bounds_max;
    float _pad4;
};

struct packed_pixel {
    uint value;
};

layout(binding=0) readonly buffer cs_lights {
    point_light lights[];
};

layout(binding=1) readonly buffer cs_occluders {
    occluder_tri tris[];
};

layout(binding=2) buffer cs_output {
    packed_pixel pixels[];
};

layout(local_size_x=8, local_size_y=8, local_size_z=1) in;

const int AA_GRID = 4;
const int SAMPLES = AA_GRID * AA_GRID;
const float EDGE_SEAM_GUARD_LUXELS = 0.75;
const float EDGE_SEAM_TMIN_BOOST = 0.75;
const int POINT_LIGHT_ATTEN_QUADRATIC = 0;
const int POINT_LIGHT_ATTEN_LINEAR = 1;
const int POINT_LIGHT_ATTEN_INVERSE = 2;
const int POINT_LIGHT_ATTEN_INVERSE_SQUARE = 3;
const int POINT_LIGHT_ATTEN_NONE = 4;
const int POINT_LIGHT_ATTEN_LOCAL_MINLIGHT = 5;
const int POINT_LIGHT_ATTEN_INVERSE_SQUARE_B = 6;
const int DIRT_RAY_COUNT = 16;

vec3 safe_normalize(vec3 v, vec3 fallback) {
    float len_sq = dot(v, v);
    if (len_sq <= 1e-8) {
        return fallback;
    }
    return v * inversesqrt(len_sq);
}

void face_basis(vec3 n, out vec3 u, out vec3 v) {
    vec3 ref = (abs(n.y) < 0.9) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    u = safe_normalize(cross(n, ref), vec3(1.0, 0.0, 0.0));
    v = cross(n, u);
}

bool inside_poly_2d(float px, float py) {
    for (int i = 0; i < poly_count; ++i) {
        int j = (i + 1) % poly_count;
        vec2 a = poly_verts[i].xy;
        vec2 b = poly_verts[j].xy;
        vec2 edge = b - a;
        vec2 rel = vec2(px, py) - a;
        if ((edge.x * rel.y - edge.y * rel.x) < -1e-3) {
            return false;
        }
    }
    return true;
}

float dist_sq_point_segment_2d(vec2 p, vec2 a, vec2 b) {
    vec2 ab = b - a;
    float ab_len_sq = dot(ab, ab);
    if (ab_len_sq <= 1e-8) {
        vec2 d = p - a;
        return dot(d, d);
    }
    float t = clamp(dot(p - a, ab) / ab_len_sq, 0.0, 1.0);
    vec2 q = a + ab * t;
    vec2 d = p - q;
    return dot(d, d);
}

float min_dist_to_poly_edge_2d(float px, float py) {
    vec2 p = vec2(px, py);
    float min_dist_sq = 3.402823e38;
    for (int i = 0; i < poly_count; ++i) {
        int j = (i + 1) % poly_count;
        min_dist_sq = min(min_dist_sq, dist_sq_point_segment_2d(p, poly_verts[i].xy, poly_verts[j].xy));
    }
    return sqrt(min_dist_sq);
}

float dist_point_segment_3d(vec3 p, vec3 a, vec3 b) {
    vec3 ab = b - a;
    float ab_len_sq = dot(ab, ab);
    if (ab_len_sq <= 1e-8) {
        vec3 d = p - a;
        return sqrt(dot(d, d));
    }
    float t = clamp(dot(p - a, ab) / ab_len_sq, 0.0, 1.0);
    vec3 q = a + ab * t;
    vec3 d = p - q;
    return sqrt(dot(d, d));
}

uint hash_float_component(float v) {
    return uint(int(round(v * 1000.0)));
}

uint hash_point_seed(vec3 position, int extra) {
    uint seed = 2166136261u;
    seed = (seed ^ hash_float_component(position.x)) * 16777619u;
    seed = (seed ^ hash_float_component(position.y)) * 16777619u;
    seed = (seed ^ hash_float_component(position.z)) * 16777619u;
    seed = (seed ^ uint(extra)) * 16777619u;
    return (seed != 0u) ? seed : 1u;
}

float random_float_01(inout uint state) {
    state = state * 1664525u + 1013904223u;
    return float(state & 0x00FFFFFFu) / float(0x01000000u);
}

vec3 random_point_in_unit_sphere(inout uint state) {
    for (int attempt = 0; attempt < 16; ++attempt) {
        vec3 p = vec3(
            random_float_01(state) * 2.0 - 1.0,
            random_float_01(state) * 2.0 - 1.0,
            random_float_01(state) * 2.0 - 1.0
        );
        if (dot(p, p) <= 1.0) {
            return p;
        }
    }
    return vec3(0.0);
}

bool ray_aabb(vec3 ro, vec3 inv_rd, vec3 bb_min, vec3 bb_max, float tmax) {
    float t1;
    float t2;
    float tn = 0.0;
    float tf = tmax;

    t1 = (bb_min.x - ro.x) * inv_rd.x;
    t2 = (bb_max.x - ro.x) * inv_rd.x;
    tn = max(tn, min(t1, t2));
    tf = min(tf, max(t1, t2));

    t1 = (bb_min.y - ro.y) * inv_rd.y;
    t2 = (bb_max.y - ro.y) * inv_rd.y;
    tn = max(tn, min(t1, t2));
    tf = min(tf, max(t1, t2));

    t1 = (bb_min.z - ro.z) * inv_rd.z;
    t2 = (bb_max.z - ro.z) * inv_rd.z;
    tn = max(tn, min(t1, t2));
    tf = min(tf, max(t1, t2));

    return tf >= tn;
}

bool ray_tri(vec3 ro, vec3 rd, occluder_tri tri, float tmin, float tmax) {
    vec3 e1 = tri.b - tri.a;
    vec3 e2 = tri.c - tri.a;
    vec3 p = cross(rd, e2);
    float det = dot(e1, p);
    if (abs(det) < ray_eps) {
        return false;
    }
    float inv = 1.0 / det;
    vec3 s = ro - tri.a;
    float u = dot(s, p) * inv;
    if ((u < 0.0) || (u > 1.0)) {
        return false;
    }
    vec3 q = cross(s, e1);
    float v = dot(rd, q) * inv;
    if ((v < 0.0) || ((u + v) > 1.0)) {
        return false;
    }
    float t = dot(e2, q) * inv;
    return (t > tmin) && (t < tmax);
}

vec3 safe_inverse_dir(vec3 rd) {
    vec3 denom = rd;
    if (abs(denom.x) <= ray_eps) {
        denom.x = (denom.x >= 0.0) ? ray_eps : -ray_eps;
    }
    if (abs(denom.y) <= ray_eps) {
        denom.y = (denom.y >= 0.0) ? ray_eps : -ray_eps;
    }
    if (abs(denom.z) <= ray_eps) {
        denom.z = (denom.z >= 0.0) ? ray_eps : -ray_eps;
    }
    return 1.0 / denom;
}

bool occluded(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group) {
    vec3 inv_rd = safe_inverse_dir(rd);
    for (int i = 0; i < tri_count; ++i) {
        occluder_tri tri = tris[i];
        if ((ignore_occluder_group >= 0) && (tri.occluder_group == ignore_occluder_group)) {
            continue;
        }
        if (!ray_aabb(ro, inv_rd, tri.bounds_min, tri.bounds_max, dist)) {
            continue;
        }
        if (ray_tri(ro, rd, tri, min_hit_t, dist)) {
            return true;
        }
    }
    return false;
}

uint pack_rgba8(vec4 color) {
    uvec4 c = uvec4(clamp(color, 0.0, 1.0) * 255.0);
    return (c.x) | (c.y << 8u) | (c.z << 16u) | (c.w << 24u);
}

float evaluate_light_attenuation(point_light light, float dist) {
    float safe_radius = max(1e-3, light.intensity);
    float linear = max(0.0, 1.0 - dist / safe_radius);
    float normalized_dist = dist / max(1e-3, safe_radius * 0.5);

    switch (light.attenuation_mode) {
        case POINT_LIGHT_ATTEN_LINEAR:
            return linear;
        case POINT_LIGHT_ATTEN_INVERSE:
            return linear / (1.0 + normalized_dist);
        case POINT_LIGHT_ATTEN_INVERSE_SQUARE:
            return linear / (1.0 + normalized_dist * normalized_dist);
        case POINT_LIGHT_ATTEN_NONE:
            return 1.0;
        case POINT_LIGHT_ATTEN_LOCAL_MINLIGHT:
            return max(0.35, linear);
        case POINT_LIGHT_ATTEN_INVERSE_SQUARE_B:
            return linear / (1.0 + normalized_dist * normalized_dist * 0.5);
        case POINT_LIGHT_ATTEN_QUADRATIC:
        default:
            return linear * linear;
    }
}

float evaluate_incidence_scale(point_light light, float ndl) {
    float lambert = max(0.0, ndl);
    float t = clamp(light.angle_scale, 0.0, 1.0);
    return (1.0 - t) + t * lambert;
}

float evaluate_spotlight_factor(point_light light, vec3 dir_to_light) {
    if ((light.spot_outer_cos <= -1.5) || (dot(light.spot_direction, light.spot_direction) <= 1e-6)) {
        return 1.0;
    }
    vec3 light_to_surface = -dir_to_light;
    float spot_cos = dot(safe_normalize(light.spot_direction, vec3(0.0, 1.0, 0.0)), light_to_surface);
    if (spot_cos <= light.spot_outer_cos) {
        return 0.0;
    }
    if (light.spot_inner_cos <= light.spot_outer_cos) {
        return 1.0;
    }
    return clamp((spot_cos - light.spot_outer_cos) / (light.spot_inner_cos - light.spot_outer_cos), 0.0, 1.0);
}

bool light_uses_dirt(point_light light) {
    int dirt_setting = (light.dirt == -2) ? global_dirt : light.dirt;
    return dirt_setting == 1;
}

float effective_light_dirt_scale(point_light light) {
    return (light.dirt_scale > 0.0) ? light.dirt_scale : dirt_scale;
}

float effective_light_dirt_gain(point_light light) {
    return (light.dirt_gain > 0.0) ? light.dirt_gain : dirt_gain;
}

float compute_dirt_attenuation(float occlusion_ratio, float dirt_scale_value, float dirt_gain_value) {
    float scaled = clamp(occlusion_ratio * max(0.0, dirt_scale_value), 0.0, 1.0);
    float gained = pow(scaled, max(0.01, dirt_gain_value));
    return clamp(1.0 - gained, 0.0, 1.0);
}

vec3 sample_cone_direction(vec3 axis, float cone_angle_deg, int sample_index, int sample_count) {
    vec3 dir_axis = safe_normalize(axis, vec3(0.0, 1.0, 0.0));
    if ((sample_count <= 1) || (cone_angle_deg <= 1e-3)) {
        return dir_axis;
    }
    vec3 tangent;
    vec3 bitangent;
    face_basis(dir_axis, tangent, bitangent);
    float u1 = (float(sample_index) + 0.5) / float(sample_count);
    float u2 = fract((float(sample_index) + 0.5) * 0.61803398875);
    float cos_max = cos(radians(clamp(cone_angle_deg * 0.5, 0.0, 89.0)));
    float cos_theta = 1.0 - u1 * (1.0 - cos_max);
    float sin_theta = sqrt(max(0.0, 1.0 - cos_theta * cos_theta));
    float phi = u2 * 6.28318530718;
    vec3 radial = tangent * cos(phi) + bitangent * sin(phi);
    return safe_normalize(dir_axis * cos_theta + radial * sin_theta, dir_axis);
}

vec3 evaluate_phong_normal(vec3 sample_point) {
    if (phong_neighbor_count <= 0) {
        return normal;
    }
    vec3 base_normal = safe_normalize(phong_base_normal_weight.xyz, normal);
    vec3 accum = base_normal * (phong_base_normal_weight.w * 2.0);
    float distance_bias = max(0.25, luxel_size);
    for (int i = 0; i < phong_neighbor_count; ++i) {
        float dist = dist_point_segment_3d(sample_point, phong_neighbor_edge_a[i].xyz, phong_neighbor_edge_b[i].xyz);
        float weight = phong_neighbor_normal_weight[i].w / max(distance_bias, dist);
        accum += phong_neighbor_normal_weight[i].xyz * weight;
    }
    return safe_normalize(accum, base_normal);
}

float compute_dirt_occlusion_ratio(vec3 sample_point, vec3 sample_normal) {
    bool randomized = dirt_mode != 0;
    float cone_angle = clamp(dirt_angle, 1.0, 90.0);
    uint seed = randomized ? hash_point_seed(sample_point, 7919) : 1u;
    int occluded_count = 0;
    for (int i = 0; i < DIRT_RAY_COUNT; ++i) {
        vec3 dir = sample_cone_direction(sample_normal, cone_angle, i, DIRT_RAY_COUNT);
        if (randomized) {
            dir = safe_normalize(dir + random_point_in_unit_sphere(seed) * 0.15, dir);
            if (dot(dir, sample_normal) < 0.0) {
                dir = -dir;
            }
        }
        vec3 ro = sample_point + sample_normal * shadow_bias;
        if (occluded(ro, dir, shadow_bias, max(1.0, dirt_depth), -1)) {
            occluded_count += 1;
        }
    }
    return float(occluded_count) / float(DIRT_RAY_COUNT);
}

void main() {
    ivec2 local_xy = ivec2(gl_GlobalInvocationID.xy);
    if ((local_xy.x >= rect_w) || (local_xy.y >= rect_h)) {
        return;
    }

    vec3 accum = vec3(0.0);
    int used_samples = 0;
    float inv_grid = 1.0 / float(AA_GRID);
    for (int sy = 0; sy < AA_GRID; ++sy) {
        for (int sx = 0; sx < AA_GRID; ++sx) {
            float ju = float(local_xy.x - 2) + (float(sx) + 0.5) * inv_grid;
            float jv = float(local_xy.y - 2) + (float(sy) + 0.5) * inv_grid;
            if (!inside_poly_2d(ju, jv)) {
                continue;
            }
            vec3 wp = origin + axis_u * (ju * luxel_size) + axis_v * (jv * luxel_size);
            vec3 sample_normal = evaluate_phong_normal(wp);
            float edge_dist_luxels = min_dist_to_poly_edge_2d(ju, jv);
            float edge_factor = clamp(1.0 - edge_dist_luxels / EDGE_SEAM_GUARD_LUXELS, 0.0, 1.0);
            float near_hit_t = shadow_bias + EDGE_SEAM_TMIN_BOOST * edge_factor;

            vec3 sample_rgb = ambient_color;
            bool uses_dirt = false;
            for (int li = 0; li < light_count; ++li) {
                if (light_uses_dirt(lights[li])) {
                    uses_dirt = true;
                    break;
                }
            }
            float dirt_occlusion = uses_dirt ? compute_dirt_occlusion_ratio(wp, sample_normal) : 0.0;
            for (int li = 0; li < light_count; ++li) {
                point_light light = lights[li];
                vec3 dir;
                float dist;
                float attenuation = 1.0;
                if (light.parallel != 0) {
                    dir = safe_normalize(light.parallel_direction, vec3(0.0, 1.0, 0.0));
                    dist = max(1.0, light.intensity);
                } else {
                    vec3 to_light = light.position - wp;
                    dist = length(to_light);
                    if ((dist > light.intensity) || (dist < 1e-3)) {
                        continue;
                    }
                    dir = to_light / dist;
                    attenuation = evaluate_light_attenuation(light, dist);
                    if (attenuation <= 0.0) {
                        continue;
                    }
                }
                vec3 ro = wp + sample_normal * shadow_bias + dir * shadow_bias;
                float emit = 1.0;
                if (light.directional != 0) {
                    emit = dot(light.emission_normal, -dir);
                    if (emit <= 0.0) {
                        continue;
                    }
                }
                float ndl = dot(sample_normal, dir);
                float incidence = evaluate_incidence_scale(light, ndl);
                if (incidence <= 0.0) {
                    continue;
                }
                float spotlight = evaluate_spotlight_factor(light, dir);
                if (spotlight <= 0.0) {
                    continue;
                }
                if (occluded(ro, dir, near_hit_t, max(0.0, dist - (shadow_bias * 2.0)), light.ignore_occluder_group)) {
                    continue;
                }
                float dirt = light_uses_dirt(light)
                    ? compute_dirt_attenuation(dirt_occlusion, effective_light_dirt_scale(light), effective_light_dirt_gain(light))
                    : 1.0;
                sample_rgb += light.color * (emit * spotlight * incidence * attenuation * dirt);
            }
            accum += sample_rgb;
            used_samples += 1;
        }
    }

    if (used_samples > 0) {
        accum /= float(used_samples);
    }
    ivec2 atlas_xy = ivec2(rect_x + local_xy.x, rect_y + local_xy.y);
    uint out_index = uint(atlas_xy.y * atlas_width + atlas_xy.x);
    pixels[out_index].value = pack_rgba8(vec4(accum, 1.0));
}
@end

@program bake cs_bake
