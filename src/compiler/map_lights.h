#pragma once

#include "../utils/map_types.h"

#include <cstdint>
#include <vector>

std::vector<PlayerStart> GetPlayerStarts(const Map& map);
std::vector<PointLight> GetPointLights(const Map& map);
std::vector<SurfaceLightTemplate> GetSurfaceLightTemplates(const Map& map);
LightBakeSettings GetLightBakeSettings(const Map& map);
std::vector<Vector3> GenerateFaceEmitterSamples(const MapPolygon& poly,
                                                float sampleSpacing,
                                                float offsetAlongNormal,
                                                int maxSamplesPerAxis);
std::vector<Vector3> GenerateFaceEmitterSamples(const MapPolygon& poly);
uint32_t HashLightSeed(const Vector3& position, int extra);
float RandomFloat01(uint32_t& state);
Vector3 RandomPointInUnitSphere(uint32_t& state);
void AppendDeviatedLights(const PointLight& baseLight, float deviance, int samples, int seedSalt, std::vector<PointLight>& out);
