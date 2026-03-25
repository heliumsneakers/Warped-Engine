#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct vs_params
{
    float4x4 u_mvp;
    float4x4 u_model;
};

struct main0_out
{
    float3 v_nrm [[user(locn0)]];
    float2 v_uv [[user(locn1)]];
    float2 v_lmuv [[user(locn2)]];
    float4 gl_Position [[position]];
};

struct main0_in
{
    float3 a_pos [[attribute(0)]];
    float3 a_nrm [[attribute(1)]];
    float2 a_uv [[attribute(2)]];
    float2 a_lmuv [[attribute(3)]];
};

vertex main0_out main0(main0_in in [[stage_in]], constant vs_params& _14 [[buffer(0)]])
{
    main0_out out = {};
    out.v_nrm = float3x3(_14.u_model[0].xyz, _14.u_model[1].xyz, _14.u_model[2].xyz) * in.a_nrm;
    out.v_uv = in.a_uv;
    out.v_lmuv = in.a_lmuv;
    out.gl_Position = _14.u_mvp * float4(in.a_pos, 1.0);
    return out;
}

