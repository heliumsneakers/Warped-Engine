#include "map_lights.h"
#include "map_entity_props.h"
#include "map_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

std::vector<Vector3> GenerateFaceEmitterSamples(const MapPolygon& poly,
                                                       float sampleSpacing,
                                                       float offsetAlongNormal,
                                                       int maxSamplesPerAxis) {
    Vector3 axisU{};
    Vector3 axisV{};
    FaceBasis(poly.normal, axisU, axisV);

    float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
    std::vector<Vector2> poly2d;
    poly2d.reserve(poly.verts.size());
    const Vector3 planeAnchor = poly.verts[0];
    const float anchorU = Vector3DotProduct(planeAnchor, axisU);
    const float anchorV = Vector3DotProduct(planeAnchor, axisV);
    for (const Vector3& v : poly.verts) {
        const float u = Vector3DotProduct(v, axisU);
        const float vv = Vector3DotProduct(v, axisV);
        minU = std::min(minU, u);
        maxU = std::max(maxU, u);
        minV = std::min(minV, vv);
        maxV = std::max(maxV, vv);
        poly2d.push_back({u, vv});
    }

    const float extentU = std::max(0.0f, maxU - minU);
    const float extentV = std::max(0.0f, maxV - minV);
    const float safeSpacing = std::max(1.0f, sampleSpacing);
    const int safeMaxPerAxis = std::max(1, maxSamplesPerAxis);
    const int samplesU = std::max(1, std::min(safeMaxPerAxis, (int)std::ceil(extentU / safeSpacing)));
    const int samplesV = std::max(1, std::min(safeMaxPerAxis, (int)std::ceil(extentV / safeSpacing)));
    const float stepU = (samplesU > 0) ? (extentU / (float)samplesU) : 0.0f;
    const float stepV = (samplesV > 0) ? (extentV / (float)samplesV) : 0.0f;

    std::vector<Vector3> samples;
    samples.reserve((size_t)samplesU * (size_t)samplesV);
    for (int y = 0; y < samplesV; ++y) {
        for (int x = 0; x < samplesU; ++x) {
            const float u = minU + ((float)x + 0.5f) * stepU;
            const float v = minV + ((float)y + 0.5f) * stepV;
            if (!InsidePoly2D(poly2d, u, v)) {
                continue;
            }
            Vector3 p = planeAnchor;
            p = Vector3Add(p, Vector3Scale(axisU, u - anchorU));
            p = Vector3Add(p, Vector3Scale(axisV, v - anchorV));
            p = Vector3Add(p, Vector3Scale(poly.normal, offsetAlongNormal));
            samples.push_back(p);
        }
    }

    if (samples.empty()) {
        samples.push_back(Vector3Add(PolygonCentroid(poly.verts), Vector3Scale(poly.normal, offsetAlongNormal)));
    }
    return samples;
}

std::vector<Vector3> GenerateFaceEmitterSamples(const MapPolygon& poly) {
    return GenerateFaceEmitterSamples(poly, 32.0f, 1.0f, 8);
}

uint32_t HashLightSeed(const Vector3& position, int extra) {
    auto h = [](float v) -> uint32_t {
        return (uint32_t)std::lround(v * 1000.0f);
    };
    uint32_t seed = 2166136261u;
    seed = (seed ^ h(position.x)) * 16777619u;
    seed = (seed ^ h(position.y)) * 16777619u;
    seed = (seed ^ h(position.z)) * 16777619u;
    seed = (seed ^ (uint32_t)extra) * 16777619u;
    return seed ? seed : 1u;
}

float RandomFloat01(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return (float)(state & 0x00FFFFFFu) / (float)0x01000000u;
}

Vector3 RandomPointInUnitSphere(uint32_t& state) {
    for (int attempt = 0; attempt < 16; ++attempt) {
        const Vector3 p = {
            RandomFloat01(state) * 2.0f - 1.0f,
            RandomFloat01(state) * 2.0f - 1.0f,
            RandomFloat01(state) * 2.0f - 1.0f
        };
        if (Vector3LengthSq(p) <= 1.0f) {
            return p;
        }
    }
    return {0.0f, 0.0f, 0.0f};
}

void AppendDeviatedLights(const PointLight& baseLight,
                                 float deviance,
                                 int samples,
                                 int seedSalt,
                                 std::vector<PointLight>& out)
{
    if (deviance <= 0.0f || samples <= 1) {
        out.push_back(baseLight);
        return;
    }

    const int safeSamples = std::max(1, samples);
    const Vector3 sampleColor = Vector3Scale(baseLight.color, 1.0f / (float)safeSamples);
    uint32_t seed = HashLightSeed(baseLight.position, seedSalt);
    for (int i = 0; i < safeSamples; ++i) {
        PointLight split = baseLight;
        split.position = Vector3Add(baseLight.position, Vector3Scale(RandomPointInUnitSphere(seed), deviance));
        split.color = sampleColor;
        out.push_back(split);
    }
}

LightBakeSettings GetLightBakeSettings(const Map &map) {
    LightBakeSettings settings;

    auto ConvertLegacyMaxLight = [](float value) {
        if (value <= 0.0f) {
            return 0.0f;
        }
        // Ericw-style maxlight lives in the classic Quake brightness domain.
        // Values above a small HDR-style threshold are treated as legacy values
        // and mapped into Warped's float lightmap space.
        if (value > 4.0f) {
            return value / 64.0f;
        }
        return value;
    };

    for (const Entity& entity : map.entities) {
        if (!EntityHasClass(entity, "worldspawn")) {
            continue;
        }

        Vector3 ambient255{};
        if (ParseColor255Prop(entity, "_ambient", ambient255)) {
            settings.ambientColor = Color255ToUnit(ambient255);
        }

        float luxelSize = settings.luxelSize;
        if (ParseFloatProp(entity, "_world_units_per_luxel", luxelSize) ||
            ParseFloatProp(entity, "_lmscale", luxelSize)) {
            settings.luxelSize = std::max(0.125f, luxelSize);
        }

        int bounceCount = settings.bounceCount;
        if (ParseIntProp(entity, "_bounce", bounceCount)) {
            settings.bounceCount = std::max(0, bounceCount);
        }

        float bounceScale = settings.bounceScale;
        if (ParseFloatProp(entity, "_bouncescale", bounceScale)) {
            settings.bounceScale = std::max(0.0f, bounceScale);
        }

        float bounceColorScale = settings.bounceColorScale;
        if (ParseFloatProp(entity, "_bouncecolorscale", bounceColorScale)) {
            settings.bounceColorScale = std::clamp(bounceColorScale, 0.0f, 1.0f);
        }

        float bounceLightSubdivision = settings.bounceLightSubdivision;
        if (ParseFloatProp(entity, "_bouncelightsubdivision", bounceLightSubdivision)) {
            settings.bounceLightSubdivision = std::max(1.0f, bounceLightSubdivision);
        }

        float rangeScale = settings.rangeScale;
        if (ParseFloatProp(entity, "_range", rangeScale)) {
            settings.rangeScale = std::max(0.0f, rangeScale);
        }

        float maxLight = settings.maxLight;
        if (ParseFloatProp(entity, "_maxlight", maxLight)) {
            settings.maxLight = ConvertLegacyMaxLight(maxLight);
        }

        float lightmapGamma = settings.lightmapGamma;
        if (ParseFloatProp(entity, "_gamma", lightmapGamma) ||
            ParseFloatProp(entity, "gamma", lightmapGamma)) {
            settings.lightmapGamma = std::max(0.01f, lightmapGamma);
        }

        float surfLightScale = settings.surfLightScale;
        if (ParseFloatProp(entity, "_surflightscale", surfLightScale)) {
            settings.surfLightScale = std::max(0.0f, surfLightScale);
        }

        float surfLightAttenuation = settings.surfLightAttenuation;
        if (ParseFloatProp(entity, "_surflight_atten", surfLightAttenuation)) {
            settings.surfLightAttenuation = std::max(0.0f, surfLightAttenuation);
        }

        float surfLightSubdivision = settings.surfLightSubdivision;
        if (ParseFloatProp(entity, "_surflightsubdivision", surfLightSubdivision) ||
            ParseFloatProp(entity, "_choplight", surfLightSubdivision)) {
            settings.surfLightSubdivision = std::max(1.0f, surfLightSubdivision);
        }

        float surfaceSampleOffset = settings.surfaceSampleOffset;
        if (ParseFloatProp(entity, "_sampleoffset", surfaceSampleOffset) ||
            ParseFloatProp(entity, "_sample_offset", surfaceSampleOffset)) {
            settings.surfaceSampleOffset = std::max(0.125f, surfaceSampleOffset);
        }

        ParseFloatProp(entity, "_sunlight", settings.sunlightIntensity);
        ParseFloatProp(entity, "_sunlight2", settings.sunlight2Intensity);
        ParseFloatProp(entity, "_sunlight3", settings.sunlight3Intensity);
        ParseIntProp(entity, "_sunlight_nosky", settings.sunlightNoSky);
        ParseFloatProp(entity, "_sunlight_penumbra", settings.sunlightPenumbra);
        settings.sunlightAngleScale = ParseAngleScaleProp(entity, settings.sunlightAngleScale);
        settings.dirt = ParseDirtOverrideProp(entity, "_dirt", settings.dirt);
        if (settings.dirt == -1) {
            settings.dirt = ParseDirtOverrideProp(entity, "_dirty", settings.dirt);
        }
        settings.sunlightDirt = ParseDirtOverrideProp(entity, "_sunlight_dirt", settings.sunlightDirt);
        settings.sunlight2Dirt = ParseDirtOverrideProp(entity, "_sunlight2_dirt", settings.sunlight2Dirt);
        ParseIntProp(entity, "_dirtmode", settings.dirtMode);
        ParseFloatProp(entity, "_dirtdepth", settings.dirtDepth);
        ParseFloatProp(entity, "_dirtscale", settings.dirtScale);
        ParseFloatProp(entity, "_dirtgain", settings.dirtGain);
        ParseFloatProp(entity, "_dirtangle", settings.dirtAngle);
        ParseIntProp(entity, "_lm_AA_scale", settings.lmAAScale);

        // _extra_samples: super-sampling grid size for the direct-light bake.
        // Allowed values are 0 (off -> 1x1), 2 (2x2), 4 (4x4). Anything else
        // clamps back to the nearest permitted value. The default is 4 so the
        // historical bake quality is preserved when the key is not set.
        int extraSamples = settings.extraSamples;
        if (ParseIntProp(entity, "_extra_samples", extraSamples)) {
            if (extraSamples <= 0) {
                extraSamples = 0;
            } else if (extraSamples <= 2) {
                extraSamples = 2;
            } else {
                extraSamples = 4;
            }
            settings.extraSamples = extraSamples;
        }

        // _soften: post-process box filter radius. Allowed values 0..4 → off,
        // 3x3, 5x5, 7x7, 9x9. Clamped into range.
        int soften = settings.soften;
        if (ParseIntProp(entity, "_soften", soften)) {
            if (soften < 0) soften = 0;
            if (soften > 4) soften = 4;
            settings.soften = soften;
        }

        Vector3 parsedColor{};
        if (ParseUnitOr255ColorProp(entity, "_sunlight_color", parsedColor) ||
            ParseUnitOr255ColorProp(entity, "_sun_color", parsedColor)) {
            settings.sunlightColor = parsedColor;
        }
        if (ParseUnitOr255ColorProp(entity, "_sunlight_color2", parsedColor) ||
            ParseUnitOr255ColorProp(entity, "_sunlight2_color", parsedColor)) {
            settings.sunlight2Color = parsedColor;
        }
        if (ParseUnitOr255ColorProp(entity, "_sunlight_color3", parsedColor) ||
            ParseUnitOr255ColorProp(entity, "_sunlight3_color", parsedColor)) {
            settings.sunlight3Color = parsedColor;
        }

        Vector3 sunMangle{};
        if (ParseVec3Prop(entity, "_sunlight_mangle", sunMangle) ||
            ParseVec3Prop(entity, "_sun_mangle", sunMangle) ||
            ParseVec3Prop(entity, "_sun_angle", sunMangle)) {
            settings.sunlightDirection = Vector3Scale(MangleToWorldLightDirection(sunMangle), -1.0f);
        }

        break;
    }

    printf("[LightSettings] ambient=(%.2f,%.2f,%.2f) luxel=%.3f bounces=%d bounceScale=%.2f bounceColorScale=%.2f bounceSubdiv=%.1f range=%.2f maxLight=%.3f gamma=%.2f surfScale=%.2f surfAtten=%.2f surfSubdiv=%.1f sampleOffset=%.3f sun=%.1f sun2=%.1f sun3=%.1f sunNoSky=%d dirt=%d lmAA=%d extraSamples=%d soften=%d\n",
           settings.ambientColor.x, settings.ambientColor.y, settings.ambientColor.z,
           settings.luxelSize, settings.bounceCount, settings.bounceScale, settings.bounceColorScale,
           settings.bounceLightSubdivision, settings.rangeScale, settings.maxLight, settings.lightmapGamma,
           settings.surfLightScale, settings.surfLightAttenuation, settings.surfLightSubdivision,
           settings.surfaceSampleOffset,
           settings.sunlightIntensity, settings.sunlight2Intensity, settings.sunlight3Intensity,
           settings.sunlightNoSky,
           settings.dirt, settings.lmAAScale, settings.extraSamples, settings.soften);
    return settings;
}

std::vector<PointLight> GetPointLights(const Map &map) {
    std::vector<PointLight> lights;
    int nextLightSeedSalt = 1;
    for (const Entity& e : map.entities) {
        if (EntityHasClass(e, "light_point")) {
            Vector3 originTB{0, 0, 0};
            Vector3 color255{255, 255, 255};
            ParseVec3Prop(e, "origin", originTB);
            ParseColor255Prop(e, "_color", color255);

            float intensity = 300.0f;
            ParseFloatProp(e, "intensity", intensity);

            PointLight pl{};
            pl.position = ConvertTBPointEntityToWorld(originTB);
            pl.color = Color255ToUnit(color255);
            pl.intensity = std::max(1.0f, intensity);
            pl.emissionNormal = { 0.0f, 0.0f, 0.0f };
            pl.directional = 0;
            pl.ignoreOccluderGroup = -1;
            pl.attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
            pl.angleScale = ParseAngleScaleProp(e, 1.0f);
            pl.dirt = ParseDirtOverrideProp(e, "_dirt", pl.dirt);
            ParseFloatProp(e, "_dirtscale", pl.dirtScale);
            ParseFloatProp(e, "_dirtgain", pl.dirtGain);
            float deviance = 0.0f;
            ParseFloatProp(e, "_deviance", deviance);
            int samples = 16;
            ParseIntProp(e, "_samples", samples);
            AppendDeviatedLights(pl, deviance, samples, nextLightSeedSalt++, lights);
            printf("LightPoint at (%.1f,%.1f,%.1f) radius=%.1f\n",
                   pl.position.x, pl.position.y, pl.position.z, pl.intensity);
            continue;
        }

        if (!EntityClassStartsWith(e, "light") || EntityHasClass(e, "light_brush")) {
            continue;
        }
        if (e.properties.find("_surface") != e.properties.end()) {
            continue;
        }

        Vector3 originTB{0, 0, 0};
        ParseVec3Prop(e, "origin", originTB);
        const Vector3 origin = ConvertTBPointEntityToWorld(originTB);

        Vector3 color255{255, 255, 255};
        float brightness = 300.0f;
        bool hasLightColor = ParseLightColor255AndBrightness(e, color255, &brightness);
        if (!hasLightColor) {
            ParseColor255Prop(e, "_color", color255);
        }
        ParseFloatProp(e, "light", brightness);

        int delay = 0;
        ParseIntProp(e, "delay", delay);

        float wait = 1.0f;
        ParseFloatProp(e, "wait", wait);

        PointLight pl{};
        pl.position = origin;
        pl.color = Color255ToUnit(color255);
        pl.intensity = std::max(1.0f, brightness * std::max(0.01f, wait));
        pl.emissionNormal = { 0.0f, 0.0f, 0.0f };
        pl.directional = 0;
        pl.ignoreOccluderGroup = -1;
        pl.attenuationMode = DelayToAttenuationMode(delay);
        pl.angleScale = ParseAngleScaleProp(e, 0.5f);
        pl.dirt = ParseDirtOverrideProp(e, "_dirt", pl.dirt);
        ParseFloatProp(e, "_dirtscale", pl.dirtScale);
        ParseFloatProp(e, "_dirtgain", pl.dirtGain);

        Vector3 lightDirection{};
        if (ParseLightDirection(map, e, origin, lightDirection)) {
            float outerAngle = 40.0f;
            ParseFloatProp(e, "angle", outerAngle);
            float softAngle = 0.0f;
            ParseFloatProp(e, "_softangle", softAngle);
            const float outerHalf = std::clamp(outerAngle, 1.0f, 179.0f) * 0.5f * DEG2RAD;
            const float innerHalf = std::clamp((softAngle > 0.0f) ? softAngle : outerAngle, 1.0f, std::clamp(outerAngle, 1.0f, 179.0f)) * 0.5f * DEG2RAD;
            pl.spotDirection = lightDirection;
            pl.spotOuterCos = cosf(outerHalf);
            pl.spotInnerCos = cosf(innerHalf);
        }

        float deviance = 0.0f;
        ParseFloatProp(e, "_deviance", deviance);
        int samples = 16;
        ParseIntProp(e, "_samples", samples);
        AppendDeviatedLights(pl, deviance, samples, nextLightSeedSalt++, lights);
        printf("Light at (%.1f,%.1f,%.1f) radius=%.1f delay=%d wait=%.2f\n",
               pl.position.x, pl.position.y, pl.position.z, pl.intensity, delay, wait);
    }

    int nextLightBrushGroup = 0;
    for (const Entity& entity : map.entities) {
        if (!EntityHasClass(entity, "light_brush")) continue;

        const Vector3 color255 = ReadLightBrushColor255(entity);
        const float intensity = ReadLightBrushIntensity(entity);

        for (const Brush& brush : entity.brushes) {
            const int lightBrushGroup = nextLightBrushGroup++;
            std::vector<MapPolygon> polys;
            const int nF = (int)brush.faces.size();
            if (nF < 3) continue;

            std::vector<Plane> planes(nF);
            for (int i = 0; i < nF; ++i) {
                planes[i] = brush.faces[i].plane;
            }

            std::vector<std::vector<Vector3>> facePolys(nF);
            for (int i = 0; i < nF - 2; ++i)
              for (int j = i + 1; j < nF - 1; ++j)
                for (int k = j + 1; k < nF; ++k) {
                    Vector3 ip;
                    if (!GetIntersection(planes[i], planes[j], planes[k], ip)) continue;
                    bool inside = true;
                    for (int m = 0; m < nF; ++m) {
                        float d = Vector3DotProduct(brush.faces[m].normal, ip) + planes[m].d;
                        if (d > (float)epsilon) { inside = false; break; }
                    }
                    if (inside) {
                        facePolys[i].push_back(ip);
                        facePolys[j].push_back(ip);
                        facePolys[k].push_back(ip);
                    }
                }

            for (int i = 0; i < nF; ++i) {
                RemoveDuplicatePoints(facePolys[i], (float)epsilon);
                if (facePolys[i].size() < 3) continue;

                Vector3 nTB = brush.faces[i].normal;
                SortPolygonVertices(facePolys[i], nTB);
                for (auto& p : facePolys[i]) p = ConvertTBtoRaylib(p);

                Vector3 nRL = CalculateNormal(facePolys[i][0], facePolys[i][1], facePolys[i][2]);
                if (Vector3DotProduct(nRL, ConvertTBtoRaylib(nTB)) < 0.f) {
                    std::reverse(facePolys[i].begin(), facePolys[i].end());
                    nRL = CalculateNormal(facePolys[i][0], facePolys[i][1], facePolys[i][2]);
                }

                MapPolygon mp;
                mp.verts = std::move(facePolys[i]);
                mp.normal = nRL;
                mp.occluderGroup = lightBrushGroup;
                polys.push_back(std::move(mp));
            }

            for (const MapPolygon& poly : polys) {
                const std::vector<Vector3> samples = GenerateFaceEmitterSamples(poly);
                const float invSampleCount = 1.0f / (float)samples.size();
                const Vector3 sampleColor = {
                    (color255.x / 255.0f) * invSampleCount,
                    (color255.y / 255.0f) * invSampleCount,
                    (color255.z / 255.0f) * invSampleCount
                };
                for (const Vector3& samplePos : samples) {
                    PointLight pl{};
                    pl.position = samplePos;
                    pl.color = sampleColor;
                    pl.intensity = intensity;
                    pl.emissionNormal = poly.normal;
                    pl.directional = 1;
                    pl.ignoreOccluderGroup = -1;
                    pl.attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
                    pl.angleScale = 1.0f;
                    lights.push_back(pl);
                }
                printf("LightBrush face emitters: %zu samples radius=%.1f normal=(%.2f,%.2f,%.2f)\n",
                       samples.size(), intensity, poly.normal.x, poly.normal.y, poly.normal.z);
            }
        }
    }
    return lights;
}

std::vector<SurfaceLightTemplate> GetSurfaceLightTemplates(const Map& map) {
    std::vector<SurfaceLightTemplate> templates;
    for (const Entity& e : map.entities) {
        if (!EntityClassStartsWith(e, "light")) {
            continue;
        }

        auto surfaceIt = e.properties.find("_surface");
        if (surfaceIt == e.properties.end() || surfaceIt->second.empty()) {
            continue;
        }

        Vector3 color255{255, 255, 255};
        float brightness = 300.0f;
        bool hasLightColor = ParseLightColor255AndBrightness(e, color255, &brightness);
        if (!hasLightColor) {
            ParseColor255Prop(e, "_color", color255);
        }
        ParseFloatProp(e, "light", brightness);

        int delay = 0;
        ParseIntProp(e, "delay", delay);

        float wait = 1.0f;
        ParseFloatProp(e, "wait", wait);

        SurfaceLightTemplate templ{};
        templ.texture = surfaceIt->second;
        ParseIntProp(e, "_surflight_group", templ.surfaceLightGroup);
        ParseFloatProp(e, "_surface_offset", templ.surfaceOffset);
        ParseIntProp(e, "_surface_spotlight", templ.surfaceSpotlight);
        ParseFloatProp(e, "_deviance", templ.deviance);
        ParseIntProp(e, "_samples", templ.devianceSamples);
        templ.light.color = Vector3Scale(Color255ToUnit(color255), std::max(0.0f, brightness) / 300.0f);
        templ.light.intensity = std::max(1.0f, brightness * std::max(0.01f, wait));
        templ.light.attenuationMode = DelayToAttenuationMode(delay);
        templ.light.angleScale = ParseAngleScaleProp(e, 0.5f);
        templ.light.dirt = ParseDirtOverrideProp(e, "_dirt", templ.light.dirt);
        ParseFloatProp(e, "_dirtscale", templ.light.dirtScale);
        ParseFloatProp(e, "_dirtgain", templ.light.dirtGain);

        Vector3 originTB{};
        ParseVec3Prop(e, "origin", originTB);
        const Vector3 origin = ConvertTBPointEntityToWorld(originTB);
        Vector3 lightDirection{};
        if (ParseLightDirection(map, e, origin, lightDirection)) {
            float outerAngle = 40.0f;
            ParseFloatProp(e, "angle", outerAngle);
            float softAngle = 0.0f;
            ParseFloatProp(e, "_softangle", softAngle);
            templ.light.spotDirection = lightDirection;
            templ.light.spotOuterCos = cosf(std::clamp(outerAngle, 1.0f, 179.0f) * 0.5f * DEG2RAD);
            templ.light.spotInnerCos = cosf(std::clamp((softAngle > 0.0f) ? softAngle : outerAngle, 1.0f, std::clamp(outerAngle, 1.0f, 179.0f)) * 0.5f * DEG2RAD);
        }

        templates.push_back(std::move(templ));
    }
    return templates;
}

std::vector<PlayerStart> GetPlayerStarts(const Map &map) {
    std::vector<PlayerStart> starts;
    for (auto &entity : map.entities) {
        auto it = entity.properties.find("classname");
        if (it != entity.properties.end() && it->second == "info_player_start") {
            auto orgIt = entity.properties.find("origin");
            if (orgIt != entity.properties.end()) {
                auto coords = SplitBySpace(orgIt->second);
                if (coords.size() == 3) {
                    try {
                        Vector3 posTB = {
                            std::stof(coords[0]),
                            std::stof(coords[1]),
                            std::stof(coords[2])
                        };
                        Vector3 posRL = ConvertTBPointEntityToWorld(posTB);

                        PlayerStart ps;
                        ps.position = posRL;
                        ParsePointEntityFacing(entity, ps.yaw, ps.pitch);
                        starts.push_back(ps);

                        printf("PlayerStart TB:(%.1f, %.1f, %.1f)  RL:(%.1f, %.1f, %.1f) yaw=%.1f pitch=%.1f\n",
                               posTB.x, posTB.y, posTB.z, posRL.x, posRL.y, posRL.z, ps.yaw, ps.pitch);
                    } catch (...) {}
                }
            }
        }
    }
    return starts;
}
