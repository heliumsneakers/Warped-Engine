@module warped_lightmap_bake

@cs cs_bake
struct brush_solid {
    int first_plane;
    int plane_count;
    vec2 _pad;
};

struct solid_plane {
    vec3 point;
    float _pad0;
    vec3 normal;
    float _pad1;
};

struct repair_source_neighbor {
    int source_poly_index;
    int edge_index;
    vec2 _pad;
};

struct repair_source_poly {
    vec3 plane_point;
    float _pad0;
    vec3 axis_u;
    float _pad1;
    vec3 axis_v;
    float _pad2;
    vec3 normal;
    int poly_count;
    int first_neighbor;
    int neighbor_count;
    vec2 _pad3;
    vec4 poly_verts[32];
};

struct phong_source_poly {
    vec3 normal;
    float area_weight;
    int enabled;
    int first_neighbor;
    int neighbor_count;
    float _pad0;
};

struct phong_neighbor {
    int source_poly_index;
    vec3 _pad0;
    vec3 edge_a;
    float _pad1;
    vec3 edge_b;
    float _pad2;
    vec3 normal;
    float area_weight;
};

layout(binding=0) uniform cs_face_params {
    int atlas_width;
    int atlas_height;
    int rect_x;
    int rect_y;
    int rect_w;
    int rect_h;
    int light_count;
    int surface_emitter_count_total;
    int surface_emitter_sample_count_total;
    int rect_surface_emitter_index_count_total;
    int tri_count;
    int bvh_node_count;
    int bvh_tri_index_count;
    int solid_count;
    int solid_plane_count;
    int repair_poly_count;
    int repair_link_count;
    int phong_poly_count;
    int phong_neighbor_count_total;
    int repair_source_poly_index;
    int _pad_repair0;
    float min_u;
    float min_v;
    float luxel_size;
    float shadow_bias;
    vec3 ambient_color;
    float ray_eps;
    int global_dirt;
    int dirt_mode;
    int skylight_dirt;
    int phong_neighbor_count;
    int rect_surface_emitter_index_first;
    int rect_surface_emitter_index_count;
    float dirt_depth;
    float dirt_scale;
    float dirt_gain;
    float dirt_angle;
    float skylight_angle_scale;
    float sky_trace_distance;
    int sunlight_nosky;
    // Super-sampling grid size for the bake (worldspawn `_extra_samples`
    // key). Valid values: 1 (off), 2 (2x2), 4 (4x4). The CPU path uses the
    // same derivation from settings.extraSamples via g_aaGrid.
    int extra_samples;
    int oversampled_output;
    float surface_sample_offset;
    float _pad_scalar0;
    vec3 sunlight2_color;
    float sunlight2_intensity;
    vec3 sunlight3_color;
    float sunlight3_intensity;
    vec3 origin;
    float _pad0;
    vec3 axis_u;
    float _pad1;
    vec3 axis_v;
    float _pad2;
    vec3 normal;
    float _pad3;
    int poly_count;
    int _pad_poly_count;
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

struct surface_emitter {
    vec3 color;
    float intensity;
    vec3 surface_normal;
    float sample_intensity_scale;
    vec3 spot_direction;
    float spot_outer_cos;
    float spot_inner_cos;
    float attenuation_scale;
    float transport_scale;
    float hotspot_clamp;
    float dirt_scale;
    float dirt_gain;
    int ignore_occluder_group;
    int dirt;
    int first_sample_point;
    int sample_point_count;
    int omnidirectional;
    int rescale;
    vec2 _pad0;
};

struct surface_emitter_sample {
    vec3 point;
    float _pad0;
};

struct surface_emitter_index {
    uint emitter_index;
    vec3 _pad0;
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
    int source_poly_index;
};

struct bvh_node {
    vec3 bounds_min;
    int left_first;
    vec3 bounds_max;
    int right_count;
};

struct bvh_tri_index {
    uint tri_index;
    vec3 _pad0;
};

struct baked_pixel {
    vec4 value;
};

layout(binding=0) readonly buffer cs_lights {
    point_light lights[];
};

layout(binding=1) readonly buffer cs_occluders {
    occluder_tri tris[];
};

layout(binding=12) readonly buffer cs_bvh_nodes {
    bvh_node bvh_nodes[];
};

layout(binding=13) readonly buffer cs_bvh_tri_indices {
    bvh_tri_index bvh_tri_indices[];
};

layout(binding=2) readonly buffer cs_solids {
    brush_solid solids[];
};

layout(binding=3) readonly buffer cs_solid_planes {
    solid_plane solid_planes[];
};

layout(binding=4) readonly buffer cs_repair_source_polys {
    repair_source_poly repair_source_polys[];
};

layout(binding=5) readonly buffer cs_repair_source_neighbors {
    repair_source_neighbor repair_source_neighbors[];
};

layout(binding=6) readonly buffer cs_phong_source_polys {
    phong_source_poly phong_source_polys[];
};

layout(binding=7) readonly buffer cs_phong_neighbors {
    phong_neighbor phong_neighbors[];
};

layout(binding=8) buffer cs_output {
    baked_pixel pixels[];
};

layout(binding=9) readonly buffer cs_surface_emitters {
    surface_emitter surface_emitters[];
};

layout(binding=10) readonly buffer cs_surface_emitter_samples {
    surface_emitter_sample surface_emitter_samples[];
};

layout(binding=11) readonly buffer cs_rect_surface_emitter_indices {
    surface_emitter_index rect_surface_emitter_indices[];
};

layout(local_size_x=8, local_size_y=8, local_size_z=1) in;

const int AA_GRID = 4;
const int SAMPLES = AA_GRID * AA_GRID;
const float EDGE_SEAM_GUARD_LUXELS = 0.75;
const float EDGE_SEAM_TMIN_SCALE = 3.0;
const int POINT_LIGHT_ATTEN_QUADRATIC = 0;
const int POINT_LIGHT_ATTEN_LINEAR = 1;
const int POINT_LIGHT_ATTEN_INVERSE = 2;
const int POINT_LIGHT_ATTEN_INVERSE_SQUARE = 3;
const int POINT_LIGHT_ATTEN_NONE = 4;
const int POINT_LIGHT_ATTEN_LOCAL_MINLIGHT = 5;
const int POINT_LIGHT_ATTEN_INVERSE_SQUARE_B = 6;
const int DIRT_NUM_ANGLE_STEPS = 16;
const int DIRT_NUM_ELEVATION_STEPS = 3;
const int DIRT_RAY_COUNT = DIRT_NUM_ANGLE_STEPS * DIRT_NUM_ELEVATION_STEPS;
const int SKYDOME_ELEVATION_STEPS = 5;
const int SKYDOME_ANGLE_STEPS = SKYDOME_ELEVATION_STEPS * 4;
const int SKYDOME_SAMPLE_COUNT = SKYDOME_ANGLE_STEPS * SKYDOME_ELEVATION_STEPS + 1;
const int REPAIR_RECURSION_MAX = 3;
const int BVH_STACK_SIZE = 64;

vec3 safe_normalize(vec3 v, vec3 fallback) {
    float len_sq = dot(v, v);
    if (len_sq <= 1e-8) {
        return fallback;
    }
    return v * inversesqrt(len_sq);
}

vec3 build_face_local_shadow_ray_origin(vec3 plane_point, vec3 face_normal) {
    return plane_point + safe_normalize(face_normal, vec3(0.0, 1.0, 0.0)) * shadow_bias;
}

void face_basis(vec3 n, out vec3 u, out vec3 v) {
    vec3 ref = (abs(n.y) < 0.9) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    u = safe_normalize(cross(n, ref), vec3(1.0, 0.0, 0.0));
    v = cross(n, u);
}

bool inside_poly_array(float px, float py, int count, vec4 verts[32]) {
    for (int i = 0; i < count; ++i) {
        int j = (i + 1) % count;
        vec2 a = verts[i].xy;
        vec2 b = verts[j].xy;
        vec2 edge = b - a;
        vec2 rel = vec2(px, py) - a;
        if ((edge.x * rel.y - edge.y * rel.x) < -1e-3) {
            return false;
        }
    }
    return true;
}

bool inside_poly_2d(float px, float py) {
    return inside_poly_array(px, py, poly_count, poly_verts);
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

vec2 closest_point_on_poly_boundary_2d(vec2 p, int count, vec4 verts[32]) {
    vec2 best = p;
    float best_dist_sq = 3.402823e38;
    for (int i = 0; i < count; ++i) {
        int j = (i + 1) % count;
        vec2 a = verts[i].xy;
        vec2 b = verts[j].xy;
        vec2 ab = b - a;
        float ab_len_sq = dot(ab, ab);
        float t = 0.0;
        if (ab_len_sq > 1e-8) {
            t = clamp(dot(p - a, ab) / ab_len_sq, 0.0, 1.0);
        }
        vec2 q = a + ab * t;
        vec2 d = p - q;
        float dist_sq = dot(d, d);
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best = q;
        }
    }
    return best;
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

float ray_tri_t(vec3 ro, vec3 rd, occluder_tri tri) {
    vec3 e1 = tri.b - tri.a;
    vec3 e2 = tri.c - tri.a;
    vec3 p = cross(rd, e2);
    float det = dot(e1, p);
    if (abs(det) < ray_eps) {
        return -1.0;
    }
    float inv = 1.0 / det;
    vec3 s = ro - tri.a;
    float u = dot(s, p) * inv;
    if ((u < -ray_eps) || (u > 1.0 + ray_eps)) {
        return -1.0;
    }
    vec3 q = cross(s, e1);
    float v = dot(rd, q) * inv;
    if ((v < -ray_eps) || ((u + v) > 1.0 + ray_eps)) {
        return -1.0;
    }
    return dot(e2, q) * inv;
}

bool ray_tri(vec3 ro, vec3 rd, occluder_tri tri, float tmin, float tmax) {
    float t = ray_tri_t(ro, rd, tri);
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

bool occluded_bruteforce(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group, int ignore_source_poly_index) {
    vec3 inv_rd = safe_inverse_dir(rd);
    for (int i = 0; i < tri_count; ++i) {
        occluder_tri tri = tris[i];
        if ((ignore_occluder_group >= 0) && (tri.occluder_group == ignore_occluder_group)) {
            continue;
        }
        if ((ignore_source_poly_index >= 0) && (tri.source_poly_index == ignore_source_poly_index)) {
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

float closest_hit_distance_bruteforce(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group, int ignore_source_poly_index) {
    vec3 inv_rd = safe_inverse_dir(rd);
    float closest = dist;
    for (int i = 0; i < tri_count; ++i) {
        occluder_tri tri = tris[i];
        if ((ignore_occluder_group >= 0) && (tri.occluder_group == ignore_occluder_group)) {
            continue;
        }
        if ((ignore_source_poly_index >= 0) && (tri.source_poly_index == ignore_source_poly_index)) {
            continue;
        }
        if (!ray_aabb(ro, inv_rd, tri.bounds_min, tri.bounds_max, closest)) {
            continue;
        }
        float t = ray_tri_t(ro, rd, tri);
        if ((t > min_hit_t) && (t < closest)) {
            closest = t;
        }
    }
    return closest;
}

bool occluded_bvh(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group, int ignore_source_poly_index) {
    vec3 inv_rd = safe_inverse_dir(rd);
    int stack[BVH_STACK_SIZE];
    int stack_size = 0;
    int node_index = 0;

    while (true) {
        if ((node_index >= 0) && (node_index < bvh_node_count)) {
            bvh_node node = bvh_nodes[node_index];
            if (ray_aabb(ro, inv_rd, node.bounds_min, node.bounds_max, dist)) {
                if (node.right_count > 0) {
                    for (int i = 0; i < node.right_count; ++i) {
                        int index_index = node.left_first + i;
                        if ((index_index < 0) || (index_index >= bvh_tri_index_count)) {
                            continue;
                        }
                        int tri_index = int(bvh_tri_indices[index_index].tri_index);
                        if ((tri_index < 0) || (tri_index >= tri_count)) {
                            continue;
                        }
                        occluder_tri tri = tris[tri_index];
                        if ((ignore_occluder_group >= 0) && (tri.occluder_group == ignore_occluder_group)) {
                            continue;
                        }
                        if ((ignore_source_poly_index >= 0) && (tri.source_poly_index == ignore_source_poly_index)) {
                            continue;
                        }
                        if (ray_tri(ro, rd, tri, min_hit_t, dist)) {
                            return true;
                        }
                    }
                } else {
                    int left_child = node.left_first;
                    int right_child = -node.right_count;
                    if (stack_size >= BVH_STACK_SIZE) {
                        return occluded_bruteforce(ro, rd, min_hit_t, dist, ignore_occluder_group, ignore_source_poly_index);
                    }
                    stack[stack_size] = right_child;
                    stack_size += 1;
                    node_index = left_child;
                    continue;
                }
            }
        }

        if (stack_size <= 0) {
            break;
        }
        stack_size -= 1;
        node_index = stack[stack_size];
    }
    return false;
}

float closest_hit_distance_bvh(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group, int ignore_source_poly_index) {
    vec3 inv_rd = safe_inverse_dir(rd);
    float closest = dist;
    int stack[BVH_STACK_SIZE];
    int stack_size = 0;
    int node_index = 0;

    while (true) {
        if ((node_index >= 0) && (node_index < bvh_node_count)) {
            bvh_node node = bvh_nodes[node_index];
            if (ray_aabb(ro, inv_rd, node.bounds_min, node.bounds_max, closest)) {
                if (node.right_count > 0) {
                    for (int i = 0; i < node.right_count; ++i) {
                        int index_index = node.left_first + i;
                        if ((index_index < 0) || (index_index >= bvh_tri_index_count)) {
                            continue;
                        }
                        int tri_index = int(bvh_tri_indices[index_index].tri_index);
                        if ((tri_index < 0) || (tri_index >= tri_count)) {
                            continue;
                        }
                        occluder_tri tri = tris[tri_index];
                        if ((ignore_occluder_group >= 0) && (tri.occluder_group == ignore_occluder_group)) {
                            continue;
                        }
                        if ((ignore_source_poly_index >= 0) && (tri.source_poly_index == ignore_source_poly_index)) {
                            continue;
                        }
                        float t = ray_tri_t(ro, rd, tri);
                        if ((t > min_hit_t) && (t < closest)) {
                            closest = t;
                        }
                    }
                } else {
                    int left_child = node.left_first;
                    int right_child = -node.right_count;
                    if (stack_size >= BVH_STACK_SIZE) {
                        return closest_hit_distance_bruteforce(ro, rd, min_hit_t, dist, ignore_occluder_group, ignore_source_poly_index);
                    }
                    stack[stack_size] = right_child;
                    stack_size += 1;
                    node_index = left_child;
                    continue;
                }
            }
        }

        if (stack_size <= 0) {
            break;
        }
        stack_size -= 1;
        node_index = stack[stack_size];
    }
    return closest;
}

bool occluded(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group, int ignore_source_poly_index) {
    if (bvh_node_count > 0) {
        return occluded_bvh(ro, rd, min_hit_t, dist, ignore_occluder_group, ignore_source_poly_index);
    }
    return occluded_bruteforce(ro, rd, min_hit_t, dist, ignore_occluder_group, ignore_source_poly_index);
}

float closest_hit_distance(vec3 ro, vec3 rd, float min_hit_t, float dist, int ignore_occluder_group, int ignore_source_poly_index) {
    if (bvh_node_count > 0) {
        return closest_hit_distance_bvh(ro, rd, min_hit_t, dist, ignore_occluder_group, ignore_source_poly_index);
    }
    return closest_hit_distance_bruteforce(ro, rd, min_hit_t, dist, ignore_occluder_group, ignore_source_poly_index);
}

vec3 project_point_onto_plane(vec3 point, vec3 plane_point, vec3 plane_normal) {
    vec3 n = safe_normalize(plane_normal, vec3(0.0, 1.0, 0.0));
    float plane_dist = dot(n, point - plane_point);
    return point - n * plane_dist;
}

bool point_inside_brush_solid(brush_solid solid, vec3 point) {
    for (int i = 0; i < solid.plane_count; ++i) {
        solid_plane plane = solid_planes[solid.first_plane + i];
        float plane_dist = dot(plane.normal, point - plane.point);
        if (plane_dist > 0.05) {
            return false;
        }
    }
    return solid.plane_count > 0;
}

bool point_inside_any_solid(vec3 point) {
    for (int i = 0; i < solid_count; ++i) {
        if (point_inside_brush_solid(solids[i], point)) {
            return true;
        }
    }
    return false;
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

float evaluate_angle_scale(float angle_scale, float ndl) {
    float lambert = max(0.0, ndl);
    float t = clamp(angle_scale, 0.0, 1.0);
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

float evaluate_surface_emitter_spotlight_factor(surface_emitter emitter, vec3 dir_to_light) {
    if ((emitter.spot_outer_cos <= -1.5) || (dot(emitter.spot_direction, emitter.spot_direction) <= 1e-6)) {
        return 1.0;
    }
    vec3 light_to_surface = -dir_to_light;
    float spot_cos = dot(safe_normalize(emitter.spot_direction, vec3(0.0, 1.0, 0.0)), light_to_surface);
    if (spot_cos <= emitter.spot_outer_cos) {
        return 0.0;
    }
    if (emitter.spot_inner_cos <= emitter.spot_outer_cos) {
        return 1.0;
    }
    return clamp((spot_cos - emitter.spot_outer_cos) / (emitter.spot_inner_cos - emitter.spot_outer_cos), 0.0, 1.0);
}

bool surface_emitter_uses_dirt(surface_emitter emitter) {
    int dirt_setting = (emitter.dirt == -2) ? global_dirt : emitter.dirt;
    return dirt_setting == 1;
}

float effective_surface_emitter_dirt_scale(surface_emitter emitter) {
    return (emitter.dirt_scale > 0.0) ? emitter.dirt_scale : dirt_scale;
}

float effective_surface_emitter_dirt_gain(surface_emitter emitter) {
    return (emitter.dirt_gain > 0.0) ? emitter.dirt_gain : dirt_gain;
}

float evaluate_surface_emitter_distance_falloff(surface_emitter emitter, float dist) {
    float scaled_dist = max(emitter.hotspot_clamp, max(0.01, emitter.attenuation_scale * dist));
    return (emitter.transport_scale * emitter.sample_intensity_scale) / (scaled_dist * scaled_dist);
}

float compute_dirt_attenuation(float occlusion_ratio, float dirt_scale_value, float dirt_gain_value) {
    float scaled = clamp(occlusion_ratio * max(0.0, dirt_scale_value), 0.0, 1.0);
    float gained = pow(scaled, max(0.01, dirt_gain_value));
    return clamp(1.0 - gained, 0.0, 1.0);
}

vec3 transform_to_tangent_space(vec3 normal, vec3 tangent, vec3 bitangent, vec3 local) {
    return bitangent * local.x + tangent * local.y + normal * local.z;
}

vec3 build_ericw_dirt_vector(int sample_index, inout uint state) {
    float dirt_angle_clamped = clamp(dirt_angle, 1.0, 90.0);
    if (dirt_mode == 1) {
        float angle = random_float_01(state) * 6.28318530718;
        float elevation = random_float_01(state) * radians(dirt_angle_clamped);
        float sin_elevation = sin(elevation);
        return vec3(cos(angle) * sin_elevation, sin(angle) * sin_elevation, cos(elevation));
    }

    int angle_index = sample_index / DIRT_NUM_ELEVATION_STEPS;
    int elevation_index = sample_index % DIRT_NUM_ELEVATION_STEPS;
    float angle_step = 6.28318530718 / float(DIRT_NUM_ANGLE_STEPS);
    float elevation_step = radians(dirt_angle_clamped) / float(DIRT_NUM_ELEVATION_STEPS);
    float angle = float(angle_index) * angle_step;
    float elevation = elevation_step * (0.5 + float(elevation_index));
    float sin_elevation = sin(elevation);
    return vec3(sin_elevation * cos(angle), sin_elevation * sin(angle), cos(elevation));
}

struct repaired_sample {
    vec3 plane_point;
    vec3 sample_point;
    int source_poly_index;
};

bool repair_source_poly_valid(int source_poly_index) {
    return (source_poly_index >= 0) &&
           (source_poly_index < repair_poly_count) &&
           (repair_source_polys[source_poly_index].poly_count > 0);
}

bool inside_repair_source_poly(int source_poly_index, vec2 point_uv) {
    if (!repair_source_poly_valid(source_poly_index)) {
        return false;
    }

    int count = repair_source_polys[source_poly_index].poly_count;
    for (int i = 0; i < count; ++i) {
        int j = (i + 1) % count;
        vec2 a = repair_source_polys[source_poly_index].poly_verts[i].xy;
        vec2 b = repair_source_polys[source_poly_index].poly_verts[j].xy;
        vec2 edge = b - a;
        vec2 rel = point_uv - a;
        if ((edge.x * rel.y - edge.y * rel.x) < -1e-3) {
            return false;
        }
    }
    return count > 0;
}

vec2 closest_point_on_repair_source_poly_boundary_2d(vec2 point_uv, int source_poly_index) {
    if (!repair_source_poly_valid(source_poly_index)) {
        return point_uv;
    }

    int count = repair_source_polys[source_poly_index].poly_count;
    vec2 best = point_uv;
    float best_dist_sq = 3.402823e38;
    for (int i = 0; i < count; ++i) {
        int j = (i + 1) % count;
        vec2 a = repair_source_polys[source_poly_index].poly_verts[i].xy;
        vec2 b = repair_source_polys[source_poly_index].poly_verts[j].xy;
        vec2 ab = b - a;
        float ab_len_sq = dot(ab, ab);
        float t = 0.0;
        if (ab_len_sq > 1e-8) {
            t = clamp(dot(point_uv - a, ab) / ab_len_sq, 0.0, 1.0);
        }
        vec2 q = a + ab * t;
        vec2 d = point_uv - q;
        float dist_sq = dot(d, d);
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best = q;
        }
    }
    return best;
}

float repair_edge_distance(int source_poly_index, int edge_index, vec2 point_uv) {
    if (!repair_source_poly_valid(source_poly_index)) {
        return 3.402823e38;
    }
    int count = repair_source_polys[source_poly_index].poly_count;
    if ((edge_index < 0) || (edge_index >= count)) {
        return 3.402823e38;
    }
    vec2 a = repair_source_polys[source_poly_index].poly_verts[edge_index].xy;
    vec2 b = repair_source_polys[source_poly_index].poly_verts[(edge_index + 1) % count].xy;
    return sqrt(dist_sq_point_segment_2d(point_uv, a, b));
}

int find_best_repair_edge_index(int source_poly_index, vec2 point_uv) {
    if (!repair_source_poly_valid(source_poly_index)) {
        return -1;
    }

    int count = repair_source_polys[source_poly_index].poly_count;
    int best_edge_index = -1;
    float best_edge_dist = 3.402823e38;
    for (int i = 0; i < count; ++i) {
        int j = (i + 1) % count;
        vec2 a = repair_source_polys[source_poly_index].poly_verts[i].xy;
        vec2 b = repair_source_polys[source_poly_index].poly_verts[j].xy;
        vec2 edge = b - a;
        vec2 rel = point_uv - a;
        if ((edge.x * rel.y - edge.y * rel.x) >= -1e-3) {
            continue;
        }

        float edge_len = length(edge);
        float edge_len_sq = edge_len * edge_len;
        float t = (edge_len_sq > 1e-8) ? dot(point_uv - a, edge) / edge_len_sq : 0.0;
        float edge_dist = 0.0;
        if (t < 0.0) {
            edge_dist = abs(t) * edge_len;
        } else if (t > 1.0) {
            edge_dist = (t - 1.0) * edge_len;
        }
        if (edge_dist < best_edge_dist) {
            best_edge_dist = edge_dist;
            best_edge_index = i;
        }
    }
    return best_edge_index;
}

bool source_poly_visited(int visited[REPAIR_RECURSION_MAX + 1], int visited_count, int source_poly_index) {
    for (int i = 0; i < visited_count; ++i) {
        if (visited[i] == source_poly_index) {
            return true;
        }
    }
    return false;
}

bool try_repair_candidate(vec3 plane_point, vec3 face_normal, out vec3 repaired_point) {
    repaired_point = plane_point + safe_normalize(face_normal, normal) * surface_sample_offset;
    if (!point_inside_any_solid(repaired_point)) {
        return true;
    }

    for (int x = -1; x <= 1; x += 2) {
        for (int y = -1; y <= 1; y += 2) {
            for (int z = -1; z <= 1; z += 2) {
                vec3 jitter = vec3(float(x), float(y), float(z)) * 0.5;
                vec3 jittered = repaired_point + jitter;
                if (!point_inside_any_solid(jittered)) {
                    repaired_point = jittered;
                    return true;
                }
            }
        }
    }

    return false;
}

bool try_recursive_repair_walk(int start_source_poly_index, vec3 start_seed_point, out repaired_sample result) {
    int stack_poly[REPAIR_RECURSION_MAX + 1];
    vec3 stack_seed[REPAIR_RECURSION_MAX + 1];
    vec3 stack_projected[REPAIR_RECURSION_MAX + 1];
    vec2 stack_projected_uv[REPAIR_RECURSION_MAX + 1];
    vec3 stack_face_seed[REPAIR_RECURSION_MAX + 1];
    int stack_stage[REPAIR_RECURSION_MAX + 1];
    int stack_inside_face[REPAIR_RECURSION_MAX + 1];
    int stack_best_edge[REPAIR_RECURSION_MAX + 1];
    int stack_last_neighbor[REPAIR_RECURSION_MAX + 1];
    float stack_last_metric[REPAIR_RECURSION_MAX + 1];
    int visited[REPAIR_RECURSION_MAX + 1];

    int depth = 0;
    stack_poly[0] = start_source_poly_index;
    stack_seed[0] = start_seed_point;
    stack_stage[0] = 0;
    stack_inside_face[0] = 0;
    stack_best_edge[0] = -1;
    stack_last_neighbor[0] = -1;
    stack_last_metric[0] = -1.0;
    visited[0] = start_source_poly_index;

    while (depth >= 0) {
        int source_poly_index = stack_poly[depth];
        if (!repair_source_poly_valid(source_poly_index)) {
            depth -= 1;
            continue;
        }

        repair_source_poly poly = repair_source_polys[source_poly_index];
        if (stack_stage[depth] == 0) {
            vec3 projected = project_point_onto_plane(stack_seed[depth], poly.plane_point, poly.normal);
            vec2 projected_uv = vec2(dot(projected, poly.axis_u), dot(projected, poly.axis_v));
            stack_projected[depth] = projected;
            stack_projected_uv[depth] = projected_uv;
            stack_face_seed[depth] = projected + safe_normalize(poly.normal, normal) * surface_sample_offset;
            stack_inside_face[depth] = inside_repair_source_poly(source_poly_index, projected_uv) ? 1 : 0;
            stack_best_edge[depth] = (stack_inside_face[depth] != 0)
                ? -1
                : find_best_repair_edge_index(source_poly_index, projected_uv);
            stack_last_neighbor[depth] = -1;
            stack_last_metric[depth] = -1.0;

            if (stack_inside_face[depth] != 0) {
                vec3 repaired_point;
                if (try_repair_candidate(projected, poly.normal, repaired_point)) {
                    result.plane_point = projected;
                    result.sample_point = repaired_point;
                    result.source_poly_index = source_poly_index;
                    return true;
                }
            }

            stack_stage[depth] = 1;
            continue;
        }

        if (stack_stage[depth] == 1) {
            bool pushed_child = false;
            if (depth < REPAIR_RECURSION_MAX && poly.neighbor_count > 0) {
                int best_neighbor_buffer_index = -1;
                int best_neighbor_source_poly = -1;
                float best_metric = 3.402823e38;
                float edge_limit = max(1.0, luxel_size * 1.5);
                for (int i = 0; i < poly.neighbor_count; ++i) {
                    int neighbor_buffer_index = poly.first_neighbor + i;
                    if ((neighbor_buffer_index < 0) || (neighbor_buffer_index >= repair_link_count)) {
                        continue;
                    }
                    repair_source_neighbor neighbor = repair_source_neighbors[neighbor_buffer_index];
                    if ((neighbor.source_poly_index < 0) ||
                        !repair_source_poly_valid(neighbor.source_poly_index) ||
                        source_poly_visited(visited, depth + 1, neighbor.source_poly_index))
                    {
                        continue;
                    }

                    float metric = 0.0;
                    if (stack_inside_face[depth] != 0) {
                        metric = repair_edge_distance(source_poly_index, neighbor.edge_index, stack_projected_uv[depth]);
                        if (metric > edge_limit) {
                            continue;
                        }
                    } else {
                        if (neighbor.edge_index != stack_best_edge[depth]) {
                            continue;
                        }
                    }

                    if ((metric < stack_last_metric[depth] - 1e-4) ||
                        ((abs(metric - stack_last_metric[depth]) <= 1e-4) && (neighbor_buffer_index <= stack_last_neighbor[depth])))
                    {
                        continue;
                    }

                    if ((best_neighbor_buffer_index < 0) ||
                        (metric < best_metric - 1e-4) ||
                        ((abs(metric - best_metric) <= 1e-4) && (neighbor_buffer_index < best_neighbor_buffer_index)))
                    {
                        best_neighbor_buffer_index = neighbor_buffer_index;
                        best_neighbor_source_poly = neighbor.source_poly_index;
                        best_metric = metric;
                    }
                }

                if (best_neighbor_buffer_index >= 0) {
                    stack_last_neighbor[depth] = best_neighbor_buffer_index;
                    stack_last_metric[depth] = best_metric;
                    depth += 1;
                    stack_poly[depth] = best_neighbor_source_poly;
                    stack_seed[depth] = stack_face_seed[depth - 1];
                    stack_stage[depth] = 0;
                    stack_inside_face[depth] = 0;
                    stack_best_edge[depth] = -1;
                    stack_last_neighbor[depth] = -1;
                    stack_last_metric[depth] = -1.0;
                    visited[depth] = best_neighbor_source_poly;
                    pushed_child = true;
                }
            }

            if (pushed_child) {
                continue;
            }

            stack_stage[depth] = 2;
            continue;
        }

        vec2 snapped_uv = closest_point_on_repair_source_poly_boundary_2d(stack_projected_uv[depth], source_poly_index);
        vec3 snapped_point = stack_projected[depth]
            + poly.axis_u * (snapped_uv.x - stack_projected_uv[depth].x)
            + poly.axis_v * (snapped_uv.y - stack_projected_uv[depth].y);
        if (distance(snapped_point, stack_projected[depth]) <= luxel_size) {
            vec3 repaired_point;
            if (try_repair_candidate(snapped_point, poly.normal, repaired_point)) {
                result.plane_point = snapped_point;
                result.sample_point = repaired_point;
                result.source_poly_index = source_poly_index;
                return true;
            }
        }

        depth -= 1;
    }

    return false;
}

repaired_sample repair_sample_point(vec3 plane_point) {
    repaired_sample result;
    result.plane_point = plane_point;
    result.sample_point = plane_point + safe_normalize(normal, vec3(0.0, 1.0, 0.0)) * surface_sample_offset;
    result.source_poly_index = repair_source_poly_index;
    if (!point_inside_any_solid(result.sample_point)) {
        return result;
    }

    if (try_repair_candidate(plane_point, normal, result.sample_point)) {
        return result;
    }

    if (repair_source_poly_valid(repair_source_poly_index)) {
        repaired_sample repaired;
        if (try_recursive_repair_walk(repair_source_poly_index, result.sample_point, repaired)) {
            return repaired;
        }
    }

    return result;
}

vec3 owner_face_normal(int source_poly_index, vec3 fallback_normal) {
    if (repair_source_poly_valid(source_poly_index)) {
        return safe_normalize(repair_source_polys[source_poly_index].normal, fallback_normal);
    }
    return safe_normalize(fallback_normal, vec3(0.0, 1.0, 0.0));
}

vec3 evaluate_phong_normal_for_source(int source_poly_index, vec3 sample_point, vec3 fallback_normal) {
    if ((source_poly_index < 0) || (source_poly_index >= phong_poly_count)) {
        return safe_normalize(fallback_normal, normal);
    }

    phong_source_poly source = phong_source_polys[source_poly_index];
    vec3 base_normal = safe_normalize(source.normal, fallback_normal);
    if ((source.enabled == 0) || (source.neighbor_count <= 0)) {
        return base_normal;
    }

    vec3 accum = base_normal * (source.area_weight * 2.0);
    float distance_bias = max(0.25, luxel_size);
    for (int i = 0; i < source.neighbor_count; ++i) {
        int neighbor_index = source.first_neighbor + i;
        if ((neighbor_index < 0) || (neighbor_index >= phong_neighbor_count_total)) {
            continue;
        }
        phong_neighbor neighbor = phong_neighbors[neighbor_index];
        float dist = dist_point_segment_3d(sample_point, neighbor.edge_a, neighbor.edge_b);
        float weight = neighbor.area_weight / max(distance_bias, dist);
        accum += neighbor.normal * weight;
    }

    if (dot(accum, accum) <= 1e-8) {
        return base_normal;
    }
    return safe_normalize(accum, base_normal);
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

vec3 ericw_skydome_direction(bool upper_hemisphere, int sample_index, float rotation_angle) {
    if (sample_index >= (SKYDOME_SAMPLE_COUNT - 1)) {
        return upper_hemisphere ? vec3(0.0, 1.0, 0.0) : vec3(0.0, -1.0, 0.0);
    }

    int ring_index = sample_index / SKYDOME_ANGLE_STEPS;
    int angle_index = sample_index % SKYDOME_ANGLE_STEPS;
    float elevation_step = radians(90.0 / float(SKYDOME_ELEVATION_STEPS + 1));
    float angle_step = radians(360.0 / float(SKYDOME_ANGLE_STEPS));
    float elevation = elevation_step * (0.5 + float(ring_index));
    float angle = angle_step * (float(angle_index) + (float(ring_index) / float(SKYDOME_ELEVATION_STEPS))) + rotation_angle;
    float radial_scale = cos(elevation);
    float vertical_scale = sin(elevation) * (upper_hemisphere ? 1.0 : -1.0);
    return safe_normalize(vec3(cos(angle) * radial_scale, vertical_scale, sin(angle) * radial_scale),
                          upper_hemisphere ? vec3(0.0, 1.0, 0.0) : vec3(0.0, -1.0, 0.0));
}

vec3 compute_skydome_lighting(int owner_source_poly_index,
                              vec3 visibility_plane_point,
                              vec3 visibility_face_normal,
                              vec3 sample_point,
                              vec3 sample_normal,
                              float near_hit_t,
                              float dirt_occlusion) {
    vec3 result = vec3(0.0);
    float upper_per_sample = (sunlight2_intensity > 0.0) ? (sunlight2_intensity / (300.0 * float(SKYDOME_SAMPLE_COUNT))) : 0.0;
    float lower_per_sample = (sunlight3_intensity > 0.0) ? (sunlight3_intensity / (300.0 * float(SKYDOME_SAMPLE_COUNT))) : 0.0;
    if ((upper_per_sample <= 0.0) && (lower_per_sample <= 0.0)) {
        return result;
    }

    uint upper_seed = hash_point_seed(sample_point, 13007);
    uint lower_seed = hash_point_seed(sample_point, 17011);
    float upper_rotation = random_float_01(upper_seed) * 6.28318530718;
    float lower_rotation = random_float_01(lower_seed) * 6.28318530718;
    float dirt = (skylight_dirt != 0)
        ? compute_dirt_attenuation(dirt_occlusion, dirt_scale, dirt_gain)
        : 1.0;

    for (int i = 0; i < SKYDOME_SAMPLE_COUNT; ++i) {
        if (upper_per_sample > 0.0) {
            vec3 dir = ericw_skydome_direction(true, i, upper_rotation);
            vec3 ro = build_face_local_shadow_ray_origin(visibility_plane_point, visibility_face_normal);
            bool visible = (sunlight_nosky != 0)
                ? !occluded(ro, dir, near_hit_t, sky_trace_distance, -1, owner_source_poly_index)
                : (closest_hit_distance(ro, dir, near_hit_t, sky_trace_distance, -1, owner_source_poly_index) >= sky_trace_distance);
            if (visible) {
                float incidence = evaluate_angle_scale(skylight_angle_scale, dot(sample_normal, dir));
                result += sunlight2_color * (upper_per_sample * incidence * dirt);
            }
        }
        if (lower_per_sample > 0.0) {
            vec3 dir = ericw_skydome_direction(false, i, lower_rotation);
            vec3 ro = build_face_local_shadow_ray_origin(visibility_plane_point, visibility_face_normal);
            bool visible = (sunlight_nosky != 0)
                ? !occluded(ro, dir, near_hit_t, sky_trace_distance, -1, owner_source_poly_index)
                : (closest_hit_distance(ro, dir, near_hit_t, sky_trace_distance, -1, owner_source_poly_index) >= sky_trace_distance);
            if (visible) {
                float incidence = evaluate_angle_scale(skylight_angle_scale, dot(sample_normal, dir));
                result += sunlight3_color * (lower_per_sample * incidence * dirt);
            }
        }
    }

    return result;
}

float compute_edge_aware_near_hit_t(float ju, float jv) {
    return ray_eps;
}

float compute_dirt_occlusion_ratio(vec3 sample_point, vec3 sample_normal) {
    vec3 tangent;
    vec3 bitangent;
    face_basis(sample_normal, tangent, bitangent);
    uint seed = hash_point_seed(sample_point, 7919);
    float accumulated_distance = 0.0;
    for (int i = 0; i < DIRT_RAY_COUNT; ++i) {
        vec3 local_dir = build_ericw_dirt_vector(i, seed);
        vec3 dir = safe_normalize(transform_to_tangent_space(sample_normal, tangent, bitangent, local_dir), sample_normal);
        float hit_distance = closest_hit_distance(sample_point, dir, ray_eps, max(1.0, dirt_depth), -1, -1);
        accumulated_distance += min(max(1.0, dirt_depth), hit_distance);
    }
    float avg_hit_distance = accumulated_distance / float(DIRT_RAY_COUNT);
    return clamp(1.0 - (avg_hit_distance / max(1.0, dirt_depth)), 0.0, 1.0);
}

vec3 compute_surface_emitter_contribution(surface_emitter emitter,
                                          vec3 emitter_sample_point,
                                          int owner_source_poly_index,
                                          vec3 visibility_plane_point,
                                          vec3 visibility_face_normal,
                                          vec3 sample_point,
                                          vec3 sample_normal,
                                          float near_hit_t,
                                          float dirt_occlusion)
{
    vec3 to_light = emitter_sample_point - sample_point;
    float dist = length(to_light);
    if ((dist > emitter.intensity) || (dist < 1e-3)) {
        return vec3(0.0);
    }

    vec3 dir_to_light = to_light / dist;
    vec3 light_to_surface = -dir_to_light;

    float receiver_dot = dot(sample_normal, dir_to_light);
    float geometric = 1.0;
    if (emitter.omnidirectional != 0) {
        geometric = max(0.0, receiver_dot * 0.5);
    } else {
        float emitter_dot = dot(emitter.surface_normal, light_to_surface);
        if ((emitter_dot < -0.01) || (receiver_dot < -0.01)) {
            return vec3(0.0);
        }
        if (emitter.rescale != 0) {
            emitter_dot = 0.5 + emitter_dot * 0.5;
            float rescaled_receiver_dot = 0.5 + receiver_dot * 0.5;
            geometric = max(0.0, emitter_dot * rescaled_receiver_dot);
        } else {
            geometric = max(0.0, emitter_dot * receiver_dot);
        }
    }
    if (geometric <= 0.0) {
        return vec3(0.0);
    }

    float spotlight = evaluate_surface_emitter_spotlight_factor(emitter, dir_to_light);
    if (spotlight <= 0.0) {
        return vec3(0.0);
    }

    float falloff = evaluate_surface_emitter_distance_falloff(emitter, dist);
    if (falloff <= 0.0) {
        return vec3(0.0);
    }

    float unshadowed_scale = geometric * spotlight * falloff;
    float peak_unshadowed = max(emitter.color.x, max(emitter.color.y, emitter.color.z)) * unshadowed_scale;
    if (peak_unshadowed <= (1.0 / 255.0)) {
        return vec3(0.0);
    }

    vec3 ro = build_face_local_shadow_ray_origin(visibility_plane_point, visibility_face_normal);
    if (occluded(ro,
                 dir_to_light,
                 near_hit_t,
                 max(0.0, dist - (shadow_bias * 2.0)),
                 emitter.ignore_occluder_group,
                 owner_source_poly_index)) {
        return vec3(0.0);
    }

    float dirt = surface_emitter_uses_dirt(emitter)
        ? compute_dirt_attenuation(dirt_occlusion,
                                   effective_surface_emitter_dirt_scale(emitter),
                                   effective_surface_emitter_dirt_gain(emitter))
        : 1.0;
    return emitter.color * (unshadowed_scale * dirt);
}

bool shade_sample(float ju, float jv, out vec3 sample_rgb) {
    if (!inside_poly_2d(ju, jv)) {
        sample_rgb = vec3(0.0);
        return false;
    }

    vec3 plane_point = origin + axis_u * (ju * luxel_size) + axis_v * (jv * luxel_size);
    repaired_sample repaired = repair_sample_point(plane_point);
    vec3 face_sample_point = plane_point + safe_normalize(normal, vec3(0.0, 1.0, 0.0)) * surface_sample_offset;
    bool face_sample_inside_solid = point_inside_any_solid(face_sample_point);
    int owner_source_poly_index = face_sample_inside_solid ? repaired.source_poly_index : repair_source_poly_index;
    vec3 owner_plane_point = face_sample_inside_solid ? repaired.plane_point : plane_point;
    vec3 owner_normal = face_sample_inside_solid
        ? owner_face_normal(repaired.source_poly_index, normal)
        : safe_normalize(normal, vec3(0.0, 1.0, 0.0));
    vec3 sample_point = face_sample_inside_solid ? repaired.sample_point : face_sample_point;
    vec3 sample_normal = evaluate_phong_normal_for_source(owner_source_poly_index, owner_plane_point, owner_normal);
    if (dot(sample_normal, sample_normal) <= 1e-8) {
        sample_normal = owner_normal;
    }
    float near_hit_t = compute_edge_aware_near_hit_t(ju, jv);

    sample_rgb = ambient_color;
    bool uses_dirt = skylight_dirt != 0;
    for (int li = 0; li < light_count; ++li) {
        if (light_uses_dirt(lights[li])) {
            uses_dirt = true;
            break;
        }
    }
    if (!uses_dirt) {
        for (int ei = 0; ei < rect_surface_emitter_index_count; ++ei) {
            int flat_index = rect_surface_emitter_index_first + ei;
            if ((flat_index < 0) || (flat_index >= rect_surface_emitter_index_count_total)) {
                continue;
            }
            uint emitter_index = rect_surface_emitter_indices[flat_index].emitter_index;
            if (emitter_index >= uint(surface_emitter_count_total)) {
                continue;
            }
            if (surface_emitter_uses_dirt(surface_emitters[emitter_index])) {
                uses_dirt = true;
                break;
            }
        }
    }
    float dirt_occlusion = uses_dirt ? compute_dirt_occlusion_ratio(sample_point, sample_normal) : 0.0;
    for (int li = 0; li < light_count; ++li) {
        point_light light = lights[li];
        vec3 dir;
        float dist;
        float attenuation = 1.0;
        if (light.parallel != 0) {
            dir = safe_normalize(light.parallel_direction, vec3(0.0, 1.0, 0.0));
            dist = max(1.0, light.intensity);
        } else {
            vec3 to_light = light.position - sample_point;
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
        vec3 ro = build_face_local_shadow_ray_origin(owner_plane_point, owner_normal);
        if (occluded(ro, dir, near_hit_t, max(0.0, dist - (shadow_bias * 2.0)), light.ignore_occluder_group, owner_source_poly_index)) {
            continue;
        }
        float dirt = light_uses_dirt(light)
            ? compute_dirt_attenuation(dirt_occlusion, effective_light_dirt_scale(light), effective_light_dirt_gain(light))
            : 1.0;
        sample_rgb += light.color * (emit * spotlight * incidence * attenuation * dirt);
    }
    for (int ei = 0; ei < rect_surface_emitter_index_count; ++ei) {
        int flat_index = rect_surface_emitter_index_first + ei;
        if ((flat_index < 0) || (flat_index >= rect_surface_emitter_index_count_total)) {
            continue;
        }
        uint emitter_index = rect_surface_emitter_indices[flat_index].emitter_index;
        if (emitter_index >= uint(surface_emitter_count_total)) {
            continue;
        }
        surface_emitter emitter = surface_emitters[emitter_index];
        for (int si = 0; si < emitter.sample_point_count; ++si) {
            int sample_index = emitter.first_sample_point + si;
            if ((sample_index < 0) || (sample_index >= surface_emitter_sample_count_total)) {
                continue;
            }
            sample_rgb += compute_surface_emitter_contribution(emitter,
                                                               surface_emitter_samples[sample_index].point,
                                                               owner_source_poly_index,
                                                               owner_plane_point,
                                                               owner_normal,
                                                               sample_point,
                                                               sample_normal,
                                                               near_hit_t,
                                                               dirt_occlusion);
        }
    }
    sample_rgb += compute_skydome_lighting(owner_source_poly_index, owner_plane_point, owner_normal, sample_point, sample_normal, near_hit_t, dirt_occlusion);
    return true;
}

void main() {
    ivec2 local_xy = ivec2(gl_GlobalInvocationID.xy);
    if ((local_xy.x >= rect_w) || (local_xy.y >= rect_h)) {
        return;
    }

    vec3 accum = vec3(0.0);
    int used_samples = 0;
    // Runtime grid size from the worldspawn `_extra_samples` key. 0 -> 1x1
    // (off), 2 -> 2x2, 4 -> 4x4 (historical default). Any unexpected value
    // falls back to 4 via the ternary below so older compiled maps still bake
    // at full quality.
    int aa_grid = (extra_samples <= 0) ? 1
               : (extra_samples <= 2) ? 2 : 4;
    float inv_grid = 1.0 / float(aa_grid);
    ivec2 atlas_xy = ivec2(rect_x + local_xy.x, rect_y + local_xy.y);
    uint out_index = uint(atlas_xy.y * atlas_width + atlas_xy.x);
    if (oversampled_output != 0) {
        vec3 sample_rgb;
        float ju = (float(local_xy.x) + 0.5) * inv_grid;
        float jv = (float(local_xy.y) + 0.5) * inv_grid;
        bool valid = shade_sample(ju, jv, sample_rgb);
        pixels[out_index].value = valid ? vec4(max(sample_rgb, vec3(0.0)), 1.0) : vec4(0.0);
        return;
    }
    for (int sy = 0; sy < aa_grid; ++sy) {
        for (int sx = 0; sx < aa_grid; ++sx) {
            float ju = float(local_xy.x - 2) + (float(sx) + 0.5) * inv_grid;
            float jv = float(local_xy.y - 2) + (float(sy) + 0.5) * inv_grid;
            vec3 sample_rgb;
            if (!shade_sample(ju, jv, sample_rgb)) {
                continue;
            }
            accum += sample_rgb;
            used_samples += 1;
        }
    }

    if (used_samples > 0) {
        accum /= float(used_samples);
    }
    pixels[out_index].value = vec4(max(accum, vec3(0.0)), 1.0);
}
@end

@program bake cs_bake
