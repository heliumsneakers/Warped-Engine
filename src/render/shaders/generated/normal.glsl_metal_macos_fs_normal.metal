#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct main0_out
{
    float4 frag_color [[color(0)]];
    float4 frag_depth_out [[color(1)]];
};

struct main0_in
{
    float3 v_nrm [[user(locn0)]];
    float v_view_dist [[user(locn1)]];
};

fragment main0_out main0(main0_in in [[stage_in]])
{
    main0_out out = {};
    out.frag_color = float4((fast::normalize(in.v_nrm) * 0.5) + float3(0.5), 1.0);
    out.frag_depth_out = float4(in.v_view_dist * 0.000244140625, 0.0, 0.0, 0.0);
    return out;
}

