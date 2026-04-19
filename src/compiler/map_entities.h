#pragma once

#include "map_tokenizer.h"
#include "../utils/map_types.h"

bool ParseEntity(MapTokenizer& tokenizer, Entity& out);
bool ParseProperty(MapTokenizer& tokenizer, std::string& key, std::string& value);
