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
    float ambient;
    float shadow_bias;
    float ray_eps;
    float _pad_scalar0;
    float _pad_scalar1;
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
    vec4 poly_verts[32];
};

struct point_light {
    vec3 position;
    float intensity;
    vec3 color;
    int directional;
    vec3 emission_normal;
    int ignore_occluder_group;
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
    uvec4 c = uvec4(clamp(color, 0.0, 1.0) * 255.0 + 0.5);
    return (c.x) | (c.y << 8u) | (c.z << 16u) | (c.w << 24u);
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
            float edge_dist_luxels = min_dist_to_poly_edge_2d(ju, jv);
            float edge_factor = clamp(1.0 - edge_dist_luxels / EDGE_SEAM_GUARD_LUXELS, 0.0, 1.0);
            float near_hit_t = shadow_bias + EDGE_SEAM_TMIN_BOOST * edge_factor;

            vec3 sample_rgb = vec3(ambient);
            for (int li = 0; li < light_count; ++li) {
                point_light light = lights[li];
                vec3 to_light = light.position - wp;
                float dist = length(to_light);
                if ((dist > light.intensity) || (dist < 1e-3)) {
                    continue;
                }
                vec3 dir = to_light / dist;
                vec3 ro = wp + normal * shadow_bias + dir * shadow_bias;
                float emit = 1.0;
                if (light.directional != 0) {
                    emit = dot(light.emission_normal, -dir);
                    if (emit <= 0.0) {
                        continue;
                    }
                }
                float ndl = dot(normal, dir);
                if (ndl <= 0.0) {
                    continue;
                }
                if (occluded(ro, dir, near_hit_t, max(0.0, dist - (shadow_bias * 2.0)), light.ignore_occluder_group)) {
                    continue;
                }
                float attenuation = 1.0 - dist / light.intensity;
                attenuation *= attenuation;
                sample_rgb += light.color * (emit * ndl * attenuation);
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
