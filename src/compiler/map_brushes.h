#pragma once

#include "map_tokenizer.h"
#include "../utils/map_types.h"

bool ParseBrush(MapTokenizer& tokenizer, Brush& out);
bool ParseFace(MapTokenizer& tokenizer, Face& out);
bool ParseVector3(MapTokenizer& tokenizer, Vector3& out);
bool ParseTextureAxis(MapTokenizer& tokenizer, Vector3& axis, float& offset);
