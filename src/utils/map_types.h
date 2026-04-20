#pragma once

#include "../math/wmath.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

inline constexpr double epsilon = 1e-4;

struct Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 texcoord;
};

struct Plane {
    Vector3 normal;
    double d;
};

struct Face {
    std::vector<Vector3> vertices;
    Plane plane{};
    std::string texture;
    Vector3 textureAxes1;
    Vector3 textureAxes2;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
    float rotation = 0.0f;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    Vector3 normal{};
};

struct Brush {
    std::vector<Face> faces;
};

struct Entity {
    std::unordered_map<std::string, std::string> properties;
    std::vector<Brush> brushes;
};

struct Map {
    std::vector<Entity> entities;
};

struct PlayerStart {
    Vector3 position;
    float yaw = 0.0f;
    float pitch = 0.0f;
};

enum PointLightAttenuationMode : uint8_t {
    POINT_LIGHT_ATTEN_QUADRATIC = 0,
    POINT_LIGHT_ATTEN_LINEAR,
    POINT_LIGHT_ATTEN_INVERSE,
    POINT_LIGHT_ATTEN_INVERSE_SQUARE,
    POINT_LIGHT_ATTEN_NONE,
    POINT_LIGHT_ATTEN_LOCAL_MINLIGHT,
    POINT_LIGHT_ATTEN_INVERSE_SQUARE_B,
};

struct PointLight {
    Vector3 position;
    Vector3 color;
    float intensity;
    Vector3 emissionNormal{0.0f, 0.0f, 0.0f};
    int directional = 0;
    Vector3 parallelDirection{0.0f, 0.0f, 0.0f};
    int parallel = 0;
    int requiresSkyVisibility = 0;
    int ignoreOccluderGroup = -1;
    uint8_t attenuationMode = POINT_LIGHT_ATTEN_QUADRATIC;
    float angleScale = 1.0f;
    int8_t dirt = -2;
    float dirtScale = -1.0f;
    float dirtGain = -1.0f;
    Vector3 spotDirection{0.0f, 0.0f, 0.0f};
    float spotOuterCos = -2.0f;
    float spotInnerCos = -2.0f;
};

struct SurfaceLightTemplate {
    std::string texture;
    int surfaceLightGroup = 0;
    float surfaceOffset = 2.0f;
    int surfaceSpotlight = 0;
    float deviance = 0.0f;
    int devianceSamples = 16;
    PointLight light;
};

struct LightBakeSettings {
    Vector3 ambientColor{0.12f, 0.12f, 0.12f};
    float luxelSize = 1.0f;
    int bounceCount = 1;
    float bounceScale = 1.0f;
    float bounceColorScale = 0.0f;
    float bounceLightSubdivision = 64.0f;
    float rangeScale = 0.5f;
    float maxLight = 0.0f;
    float lightmapGamma = 1.0f;
    float surfLightScale = 1.0f;
    float surfLightAttenuation = 1.0f;
    float surfLightSubdivision = 16.0f;
    float surfaceSampleOffset = 1.0f;
    float sunlightIntensity = 0.0f;
    Vector3 sunlightColor{1.0f, 1.0f, 1.0f};
    Vector3 sunlightDirection{0.0f, 1.0f, 0.0f};
    float sunlightPenumbra = 0.0f;
    float sunlightAngleScale = 0.5f;
    int sunlightNoSky = 0;
    float sunlight2Intensity = 0.0f;
    Vector3 sunlight2Color{1.0f, 1.0f, 1.0f};
    float sunlight3Intensity = 0.0f;
    Vector3 sunlight3Color{1.0f, 1.0f, 1.0f};
    int dirt = -1;
    int sunlightDirt = -2;
    int sunlight2Dirt = -2;
    int dirtMode = 0;
    float dirtDepth = 128.0f;
    float dirtScale = 1.0f;
    float dirtGain = 1.0f;
    float dirtAngle = 88.0f;
    int lmAAScale = 0;
    int extraSamples = 0;
    int soften = 0;
};

struct MapVertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
    float lu, lv;
};

struct MapMeshBucket {
    std::string texture;
    uint32_t lightmapPage = 0;
    std::vector<MapVertex> vertices;
    std::vector<uint32_t> indices;
};

struct MapPolygon {
    std::vector<Vector3> verts;
    Vector3 normal;
    std::string texture;
    int occluderGroup = -1;
    int sourceBrushId = -1;
    int sourceEntityId = -1;
    int sourceFaceIndex = -1;
    int surfaceLightGroup = 0;
    uint8_t noBounce = 0;
    float surfLightAttenuation = -1.0f;
    int8_t surfLightRescale = -1;
    uint8_t phong = 0;
    float phongAngle = 89.0f;
    float phongAngleConcave = 0.0f;
    int phongGroup = 0;
    Vector3 texAxisU, texAxisV;
    float offU, offV;
    float rot, scaleU, scaleV;
    float facePlaneD = 0.0f;
};
