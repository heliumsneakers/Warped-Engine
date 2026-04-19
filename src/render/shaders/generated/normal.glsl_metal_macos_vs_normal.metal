#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct vs_params
{
    float4x4 u_mvp;
    float4x4 u_normal_model;
};

struct main0_out
{
    float3 v_nrm [[user(locn0)]];
    float v_view_dist [[user(locn1)]];
    float4 gl_Position [[position]];
};

struct main0_in
{
    float3 a_pos [[attribute(0)]];
    float3 a_nrm [[attribute(1)]];
};

vertex main0_out main0(main0_in in [[stage_in]], constant vs_params& _14 [[buffer(0)]])
{
    main0_out out = {};
    out.v_nrm = fast::normalize(float3x3(_14.u_normal_model[0].xyz, _14.u_normal_model[1].xyz, _14.u_normal_model[2].xyz) * in.a_nrm);
    float4 _43 = float4(in.a_pos, 1.0);
    out.v_view_dist = fast::max(-(_14.u_normal_model * _43).z, 0.0);
    out.gl_Position = _14.u_mvp * _43;
    return out;
}

