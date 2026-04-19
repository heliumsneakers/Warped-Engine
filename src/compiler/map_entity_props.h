#pragma once

#include "../utils/map_types.h"

#include <string>
#include <vector>

std::vector<std::string> SplitBySpace(const std::string& s);
bool ParseVec3Prop(const Entity& e, const char* key, Vector3& out);
bool ParseFloatProp(const Entity& e, const char* key, float& out);
bool ParseIntProp(const Entity& e, const char* key, int& out);
bool EntityHasClass(const Entity& e, const char* classname);
bool EntityClassStartsWith(const Entity& e, const char* prefix);
bool ShouldRenderBrushEntity(const Entity& entity, bool devMode);
int ClampColor255Component(float value);
Vector3 ClampColor255(const Vector3& color255);
Vector3 Color255ToUnit(const Vector3& color255);
bool ParseColor255Prop(const Entity& e, const char* key, Vector3& out);
bool ParseUnitOr255ColorProp(const Entity& e, const char* key, Vector3& outUnit);
bool ParseLightColor255AndBrightness(const Entity& e, Vector3& outColor255, float* outBrightness);
PointLightAttenuationMode DelayToAttenuationMode(int delay);
Vector3 MangleToTBDirection(const Vector3& mangle);
Vector3 MangleToWorldLightDirection(const Vector3& mangle);
Vector3 PointEntityAnglesToTBDirection(const Vector3& angles);
bool ParsePointEntityFacing(const Entity& entity, float& outYaw, float& outPitch);
bool FindEntityOriginByTargetname(const Map& map, const std::string& targetname, Vector3& outWorldOrigin);
bool ParseLightDirection(const Map& map, const Entity& e, const Vector3& lightOrigin, Vector3& outDirection);
float ParseAngleScaleProp(const Entity& e, float defaultValue);
int ParseDirtOverrideProp(const Entity& e, const char* key, int defaultValue);
Vector3 ReadLightBrushColor255(const Entity& e);
float ReadLightBrushIntensity(const Entity& e);
std::string EncodeLightBrushTextureName(const Vector3& color255);
