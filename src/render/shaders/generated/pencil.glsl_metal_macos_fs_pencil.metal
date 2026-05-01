#pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct fs_params
{
    float4 u_resolution_time;
    float4 u_effect_params;
};

struct main0_out
{
    float4 frag_color [[color(0)]];
};

struct main0_in
{
    float2 v_uv [[user(locn0)]];
};

static inline __attribute__((always_inline))
float4 normal_sample(thread const float2& uv, texture2d<float> u_normals, sampler u_normal_smp)
{
    float4 _59 = u_normals.sample(u_normal_smp, uv);
    return float4((_59.xyz * 2.0) - float3(1.0), _59.w);
}

static inline __attribute__((always_inline))
float depth_sample(thread const float2& uv, texture2d<float> u_depth, sampler u_depth_smp)
{
    return u_depth.sample(u_depth_smp, uv).x;
}

static inline __attribute__((always_inline))
float luminance(thread const float3& c)
{
    return dot(c, float3(0.2989999949932098388671875, 0.58700001239776611328125, 0.114000000059604644775390625));
}

static inline __attribute__((always_inline))
float normal_delta(thread const float4& center, thread const float4& sample_n)
{
    float _92 = step(9.9999997473787516355514526367188e-05, center.w);
    float _96 = step(9.9999997473787516355514526367188e-05, sample_n.w);
    return fast::max(abs(_92 - _96), (length(center.xyz - sample_n.xyz) * _92) * _96);
}

static inline __attribute__((always_inline))
float geometry_edge(thread const float2& uv, thread const float2& texel, thread const float& radius, thread const float4& center_n, thread const float& center_d, texture2d<float> u_normals, sampler u_normal_smp, texture2d<float> u_depth, sampler u_depth_smp)
{
    float2 param = uv + (texel * float2(radius, 0.0));
    float2 param_1 = uv + (texel * float2(-radius, 0.0));
    float2 param_2 = uv + (texel * float2(0.0, radius));
    float2 param_3 = uv + (texel * float2(0.0, -radius));
    float4 param_4 = center_n;
    float4 param_5 = normal_sample(param, u_normals, u_normal_smp);
    float _166 = normal_delta(param_4, param_5);
    float4 param_6 = center_n;
    float4 param_7 = normal_sample(param_1, u_normals, u_normal_smp);
    float _172 = normal_delta(param_6, param_7);
    float4 param_8 = center_n;
    float4 param_9 = normal_sample(param_2, u_normals, u_normal_smp);
    float _178 = normal_delta(param_8, param_9);
    float4 param_10 = center_n;
    float4 param_11 = normal_sample(param_3, u_normals, u_normal_smp);
    float _184 = normal_delta(param_10, param_11);
    float _205 = fast::max(fast::max(_166, _172), fast::max(_178, _184));
    float2 param_12 = uv + (texel * float2(radius, 0.0));
    float _217 = depth_sample(param_12, u_depth, u_depth_smp);
    float2 param_13 = uv + (texel * float2(-radius, 0.0));
    float _227 = depth_sample(param_13, u_depth, u_depth_smp);
    float2 param_14 = uv + (texel * float2(0.0, radius));
    float _236 = depth_sample(param_14, u_depth, u_depth_smp);
    float2 param_15 = uv + (texel * float2(0.0, -radius));
    float _246 = depth_sample(param_15, u_depth, u_depth_smp);
    return fast::max(_205 * step(1.0, ((step(0.119999997317790985107421875, _166) + step(0.119999997317790985107421875, _172)) + step(0.119999997317790985107421875, _178)) + step(0.119999997317790985107421875, _184)), smoothstep(0.008000000379979610443115234375, 0.02999999932944774627685546875, fast::max(abs((_217 + _227) - (2.0 * center_d)), abs((_236 + _246) - (2.0 * center_d))) / fast::max(center_d, 9.9999997473787516355514526367188e-05)) * (1.0 - ((1.0 - step(0.07999999821186065673828125, _205)) * (1.0 - smoothstep(0.014999999664723873138427734375, 0.07999999821186065673828125, fast::max(fast::max(abs(_217 - center_d), abs(_227 - center_d)), fast::max(abs(_236 - center_d), abs(_246 - center_d))) / fast::max(center_d, 9.9999997473787516355514526367188e-05)))))) * step(9.9999997473787516355514526367188e-05, center_n.w);
}

fragment main0_out main0(main0_in in [[stage_in]], constant fs_params& _328 [[buffer(0)]], texture2d<float> u_scene [[texture(0)]], texture2d<float> u_normals [[texture(1)]], texture2d<float> u_depth [[texture(2)]], sampler u_scene_smp [[sampler(0)]], sampler u_normal_smp [[sampler(1)]], sampler u_depth_smp [[sampler(2)]])
{
    main0_out out = {};
    float2 _340 = float2(1.0) / fast::max(_328.u_resolution_time.xy, float2(1.0));
    float2 _349 = fast::clamp(in.v_uv, _340, float2(1.0) - _340);
    float4 _357 = u_scene.sample(u_scene_smp, _349);
    float2 param = _349;
    float4 _361 = normal_sample(param, u_normals, u_normal_smp);
    float2 param_1 = _349;
    float _365 = depth_sample(param_1, u_depth, u_depth_smp);
    float3 _369 = _357.xyz;
    float3 param_2 = _369;
    float _383 = fast::max(_328.u_effect_params.y, 0.25);
    float2 param_3 = _349;
    float2 param_4 = _340;
    float param_5 = mix(_383, fast::max(_328.u_effect_params.z, _383), fast::clamp(1.0 - smoothstep(96.0, 896.0, _365 * 4096.0), 0.0, 1.0));
    float4 param_6 = _361;
    float param_7 = _365;
    out.frag_color = float4(mix(mix(float3(0.964999973773956298828125, 0.935000002384185791015625, 0.8650000095367431640625), mix(float3(luminance(param_2)), _369, float3(0.3400000035762786865234375)), float3(fast::clamp(_328.u_effect_params.w, 0.0, 1.0))), float3(0.104999996721744537353515625, 0.092000000178813934326171875, 0.08200000226497650146484375), float3(fast::clamp(smoothstep(0.1599999964237213134765625, 0.4199999868869781494140625, geometry_edge(param_3, param_4, param_5, param_6, param_7, u_normals, u_normal_smp, u_depth, u_depth_smp) * fast::max(_328.u_effect_params.x, 0.00999999977648258209228515625)) * 0.949999988079071044921875, 0.0, 1.0) * step(9.9999997473787516355514526367188e-05, _361.w))), _357.w);
    return out;
}

