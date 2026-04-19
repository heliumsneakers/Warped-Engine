#include "map_parser.h"
#include "map_entities.h"
#include "map_tokenizer.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <utility>

MapParseResult ParseMapFile(const std::string& filePath) {
    MapParseResult result;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        result.error = "failed to open map: " + filePath;
        return result;
    }

    MapTokenizer tokenizer(file);
    std::string token;
    while (NextToken(tokenizer, token)) {
        if (token != "{") {
            SetTokenizerError(tokenizer, "expected entity open");
            break;
        }

        Entity entity{};
        if (!ParseEntity(tokenizer, entity)) {
            break;
        }
        result.map.entities.push_back(std::move(entity));
    }

    if (!tokenizer.error.empty()) {
        result.error = tokenizer.error;
        result.map.entities.clear();
        return result;
    }

    bool hasValve220Version = false;
    for (const Entity& entity : result.map.entities) {
        auto versionIt = entity.properties.find("mapversion");
        if (versionIt != entity.properties.end() && versionIt->second == "220") {
            hasValve220Version = true;
            break;
        }
    }
    if (!hasValve220Version) {
        result.error = "missing required mapversion 220";
        result.map.entities.clear();
        return result;
    }

    size_t brushCount = 0;
    size_t faceCount = 0;
    for (const Entity& entity : result.map.entities) {
        brushCount += entity.brushes.size();
        for (const Brush& brush : entity.brushes) {
            faceCount += brush.faces.size();
        }
    }
    printf("Parsed map: Entities=%zu, Brushes=%zu, Faces=%zu\n",
           result.map.entities.size(), brushCount, faceCount);
    result.ok = true;
    return result;
}
