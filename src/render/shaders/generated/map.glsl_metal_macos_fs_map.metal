#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct main0_out
{
    float4 frag_color [[color(0)]];
};

struct main0_in
{
    float2 v_uv [[user(locn1)]];
    float2 v_lmuv [[user(locn2)]];
};

fragment main0_out main0(main0_in in [[stage_in]], texture2d<float> u_tex [[texture(0)]], texture2d<float> u_lm [[texture(1)]], sampler u_tex_smp [[sampler(0)]], sampler u_lm_smp [[sampler(1)]])
{
    main0_out out = {};
    float4 _24 = u_tex.sample(u_tex_smp, in.v_uv);
    out.frag_color = float4((_24.xyz * u_lm.sample(u_lm_smp, in.v_lmuv).xyz) * 2.0, _24.w);
    return out;
}

