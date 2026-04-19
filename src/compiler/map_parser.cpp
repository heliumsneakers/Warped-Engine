#include "map_parser.h"
#include "map_entities.h"
#include "map_tokenizer.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <utility>

Map ParseMapFile(const std::string& filePath) {
    Map map;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        printf("Failed to open map: %s\n", filePath.c_str());
        return map;
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
        map.entities.push_back(std::move(entity));
    }

    if (!tokenizer.error.empty()) {
        printf("Failed to parse map %s: %s\n", filePath.c_str(), tokenizer.error.c_str());
        map.entities.clear();
        return map;
    }

    bool hasValve220Version = false;
    for (const Entity& entity : map.entities) {
        auto versionIt = entity.properties.find("mapversion");
        if (versionIt != entity.properties.end() && versionIt->second == "220") {
            hasValve220Version = true;
            break;
        }
    }
    if (!hasValve220Version) {
        printf("Failed to parse map %s: missing required mapversion 220\n", filePath.c_str());
        map.entities.clear();
        return map;
    }

    size_t brushCount = 0;
    size_t faceCount = 0;
    for (const Entity& entity : map.entities) {
        brushCount += entity.brushes.size();
        for (const Brush& brush : entity.brushes) {
            faceCount += brush.faces.size();
        }
    }
    printf("Parsed map: Entities=%zu, Brushes=%zu, Faces=%zu\n",
           map.entities.size(), brushCount, faceCount);
    return map;
}
