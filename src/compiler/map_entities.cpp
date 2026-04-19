#include "map_entities.h"
#include "map_brushes.h"

#include <utility>

bool ParseProperty(MapTokenizer& tokenizer, std::string& key, std::string& value) {
    if (!NextQuotedString(tokenizer, key) || !NextQuotedString(tokenizer, value)) {
        return false;
    }
    if (key == "mapversion" && value != "220") {
        SetTokenizerError(tokenizer, "unsupported mapversion '" + value + "'; expected 220");
        return false;
    }
    return true;
}

bool ParseEntity(MapTokenizer& tokenizer, Entity& out) {
    out = Entity{};
    while (true) {
        char c = 0;
        if (!PeekChar(tokenizer, c)) {
            SetTokenizerError(tokenizer, "unexpected end of file in entity");
            return false;
        }
        if (c == '}') {
            std::string token;
            NextToken(tokenizer, token);
            return true;
        }
        if (c == '"') {
            std::string key;
            std::string value;
            if (!ParseProperty(tokenizer, key, value)) {
                return false;
            }
            out.properties[key] = value;
            continue;
        }
        if (c == '{') {
            std::string token;
            NextToken(tokenizer, token);
            Brush brush{};
            if (!ParseBrush(tokenizer, brush)) {
                return false;
            }
            out.brushes.push_back(std::move(brush));
            continue;
        }

        SetTokenizerError(tokenizer, "expected property, brush, or entity close");
        return false;
    }
}
