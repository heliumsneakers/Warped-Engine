#include "map_entity_props.h"
#include "map_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

std::vector<std::string> SplitBySpace(const std::string& s) {
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string token;
    while (iss >> token) tokens.push_back(token);
    return tokens;
}

bool ParseVec3Prop(const Entity& e, const char* key, Vector3& out) {
    auto it = e.properties.find(key);
    if (it == e.properties.end()) return false;
    auto t = SplitBySpace(it->second);
    if (t.size() != 3) return false;
    try { out = { std::stof(t[0]), std::stof(t[1]), std::stof(t[2]) }; }
    catch (...) { return false; }
    return true;
}

bool ParseFloatProp(const Entity& e, const char* key, float& out) {
    auto it = e.properties.find(key);
    if (it == e.properties.end()) return false;
    try { out = std::stof(it->second); }
    catch (...) {
        printf("  [warn] entity '%s' has non-numeric %s='%s', using default\n",
               e.properties.count("classname") ? e.properties.at("classname").c_str() : "?",
               key, it->second.c_str());
        return false;
    }
    return true;
}

bool ParseIntProp(const Entity& e, const char* key, int& out) {
    float parsed = 0.0f;
    if (!ParseFloatProp(e, key, parsed)) {
        return false;
    }
    out = (int)std::lround(parsed);
    return true;
}

bool EntityHasClass(const Entity& e, const char* classname) {
    auto it = e.properties.find("classname");
    return (it != e.properties.end()) && (it->second == classname);
}

bool EntityClassStartsWith(const Entity& e, const char* prefix) {
    auto it = e.properties.find("classname");
    if (it == e.properties.end()) {
        return false;
    }
    const std::string& name = it->second;
    const size_t len = strlen(prefix);
    return name.size() >= len && name.compare(0, len, prefix) == 0;
}

bool ShouldRenderBrushEntity(const Entity& entity, bool devMode) {
    if ((EntityHasClass(entity, "trigger_once") ||
         EntityHasClass(entity, "trigger_multiple")) && !devMode) {
        return false;
    }

    if (EntityHasClass(entity, "func_clip")) {
        return false;
    }

    return true;
}

int ClampColor255Component(float value) {
    return std::max(0, std::min(255, (int)std::lround(value)));
}

Vector3 ClampColor255(const Vector3& color255) {
    return {
        (float)ClampColor255Component(color255.x),
        (float)ClampColor255Component(color255.y),
        (float)ClampColor255Component(color255.z)
    };
}

Vector3 Color255ToUnit(const Vector3& color255) {
    const Vector3 clamped = ClampColor255(color255);
    return {
        clamped.x / 255.0f,
        clamped.y / 255.0f,
        clamped.z / 255.0f
    };
}

bool ParseColor255Prop(const Entity& e, const char* key, Vector3& out) {
    Vector3 parsed{};
    if (!ParseVec3Prop(e, key, parsed)) {
        return false;
    }
    out = ClampColor255(parsed);
    return true;
}

bool ParseUnitOr255ColorProp(const Entity& e, const char* key, Vector3& outUnit) {
    Vector3 parsed{};
    if (!ParseVec3Prop(e, key, parsed)) {
        return false;
    }
    const float maxComponent = std::max(parsed.x, std::max(parsed.y, parsed.z));
    if (maxComponent <= 1.0f) {
        outUnit = {
            std::clamp(parsed.x, 0.0f, 1.0f),
            std::clamp(parsed.y, 0.0f, 1.0f),
            std::clamp(parsed.z, 0.0f, 1.0f)
        };
        return true;
    }
    outUnit = Color255ToUnit(parsed);
    return true;
}

bool ParseLightColor255AndBrightness(const Entity& e,
                                            Vector3& outColor255,
                                            float* outBrightness)
{
    auto it = e.properties.find("_light");
    if (it == e.properties.end()) {
        return false;
    }

    const std::vector<std::string> toks = SplitBySpace(it->second);
    if (toks.size() != 3 && toks.size() != 4) {
        return false;
    }

    try {
        outColor255 = ClampColor255({
            std::stof(toks[0]),
            std::stof(toks[1]),
            std::stof(toks[2])
        });
        if (outBrightness && toks.size() == 4) {
            *outBrightness = std::stof(toks[3]);
        }
    } catch (...) {
        return false;
    }

    return true;
}

PointLightAttenuationMode DelayToAttenuationMode(int delay) {
    switch (delay) {
        case 0: return POINT_LIGHT_ATTEN_LINEAR;
        case 1: return POINT_LIGHT_ATTEN_INVERSE;
        case 2: return POINT_LIGHT_ATTEN_INVERSE_SQUARE;
        case 3: return POINT_LIGHT_ATTEN_NONE;
        case 4: return POINT_LIGHT_ATTEN_LOCAL_MINLIGHT;
        case 5: return POINT_LIGHT_ATTEN_INVERSE_SQUARE_B;
        default: return POINT_LIGHT_ATTEN_LINEAR;
    }
}

Vector3 MangleToTBDirection(const Vector3& mangle) {
    const float yaw = mangle.x * DEG2RAD;
    const float pitch = mangle.y * DEG2RAD;
    return Vector3Normalize({
        cosf(pitch) * cosf(yaw),
        cosf(pitch) * sinf(yaw),
        sinf(pitch)
    });
}

Vector3 MangleToWorldLightDirection(const Vector3& mangle) {
    return Vector3Normalize(ConvertTBPointEntityToWorld(MangleToTBDirection(mangle)));
}

Vector3 PointEntityAnglesToTBDirection(const Vector3& angles) {
    const float pitch = angles.x * DEG2RAD;
    const float yaw = angles.y * DEG2RAD;
    return Vector3Normalize({
        cosf(pitch) * cosf(yaw),
        cosf(pitch) * sinf(yaw),
        sinf(pitch)
    });
}

bool ParsePointEntityFacing(const Entity& entity, float& outYaw, float& outPitch) {
    outYaw = 0.0f;
    outPitch = 0.0f;

    Vector3 angles{};
    if (!ParseVec3Prop(entity, "angles", angles) && !ParseVec3Prop(entity, "mangle", angles)) {
        return false;
    }

    const Vector3 directionWorld = ConvertTBPointEntityToWorld(PointEntityAnglesToTBDirection(angles));
    if (Vector3LengthSq(directionWorld) <= 1.0e-6f) {
        return false;
    }

    const Vector3 normalized = Vector3Normalize(directionWorld);
    outYaw = atan2f(normalized.z, normalized.x) * RAD2DEG;
    outPitch = asinf(std::clamp(normalized.y, -1.0f, 1.0f)) * RAD2DEG;
    return true;
}

bool FindEntityOriginByTargetname(const Map& map, const std::string& targetname, Vector3& outWorldOrigin) {
    if (targetname.empty()) {
        return false;
    }
    for (const Entity& entity : map.entities) {
        auto it = entity.properties.find("targetname");
        if (it == entity.properties.end() || it->second != targetname) {
            continue;
        }
        Vector3 originTB{};
        if (!ParseVec3Prop(entity, "origin", originTB)) {
            continue;
        }
        outWorldOrigin = ConvertTBPointEntityToWorld(originTB);
        return true;
    }
    return false;
}

bool ParseLightDirection(const Map& map, const Entity& e, const Vector3& lightOrigin, Vector3& outDirection) {
    auto targetIt = e.properties.find("target");
    if (targetIt != e.properties.end()) {
        Vector3 targetOrigin{};
        if (FindEntityOriginByTargetname(map, targetIt->second, targetOrigin)) {
            const Vector3 toTarget = Vector3Subtract(targetOrigin, lightOrigin);
            if (Vector3LengthSq(toTarget) > 1e-6f) {
                outDirection = Vector3Normalize(toTarget);
                return true;
            }
        }
    }

    Vector3 mangle{};
    if (ParseVec3Prop(e, "mangle", mangle) || ParseVec3Prop(e, "angles", mangle)) {
        outDirection = MangleToWorldLightDirection(mangle);
        return true;
    }

    return false;
}

float ParseAngleScaleProp(const Entity& e, float defaultValue) {
    float value = defaultValue;
    ParseFloatProp(e, "_anglescale", value) || ParseFloatProp(e, "_anglesense", value);
    return std::clamp(value, 0.0f, 1.0f);
}

int ParseDirtOverrideProp(const Entity& e, const char* key, int defaultValue) {
    int value = defaultValue;
    ParseIntProp(e, key, value);
    return value;
}

Vector3 ReadLightBrushColor255(const Entity& e) {
    Vector3 color255{255, 255, 255};
    ParseColor255Prop(e, "_color", color255);
    return ClampColor255(color255);
}

float ReadLightBrushIntensity(const Entity& e) {
    float intensity = 300.0f;
    ParseFloatProp(e, "intensity", intensity);
    return intensity;
}

std::string EncodeLightBrushTextureName(const Vector3& color255) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "__light_brush_%d_%d_%d",
             ClampColor255Component(color255.x),
             ClampColor255Component(color255.y),
             ClampColor255Component(color255.z));
    return std::string(buffer);
}
