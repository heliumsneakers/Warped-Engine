#pragma once

#include <istream>
#include <string>

inline constexpr size_t MAP_MAX_TOKEN_LENGTH = 512;

struct MapTokenizer {
    explicit MapTokenizer(std::istream& input) : in(input) {}

    std::istream& in;
    int line = 1;
    std::string error;
};

bool PeekChar(MapTokenizer& tokenizer, char& out);
bool NextToken(MapTokenizer& tokenizer, std::string& out);
bool NextQuotedString(MapTokenizer& tokenizer, std::string& out);
void SetTokenizerError(MapTokenizer& tokenizer, const std::string& message);
