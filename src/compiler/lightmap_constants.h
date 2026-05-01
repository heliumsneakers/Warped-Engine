#pragma once

static constexpr int LIGHTMAP_LM_PAD = 2;
static constexpr float LIGHTMAP_EDGE_SEAM_GUARD_LUXELS = 0.75f;

static inline int ComputeLightmapAAGridSize(int extraSamples)
{
    return (extraSamples <= 0) ? 1 : (extraSamples <= 2) ? 2 : 4;
}
