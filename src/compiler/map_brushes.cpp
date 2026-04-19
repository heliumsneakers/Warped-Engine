#include "map_brushes.h"
#include "map_geometry.h"

#include <cmath>
#include <cstdlib>
#include <utility>

static bool ExpectToken(MapTokenizer& tokenizer, const char* expected) {
    std::string token;
    if (!NextToken(tokenizer, token) || token != expected) {
        SetTokenizerError(tokenizer, std::string("expected '") + expected + "'");
        return false;
    }
    return true;
}

static bool ParseFloatToken(MapTokenizer& tokenizer, float& out) {
    std::string token;
    if (!NextToken(tokenizer, token)) {
        SetTokenizerError(tokenizer, "expected number");
        return false;
    }
    char* end = nullptr;
    out = std::strtof(token.c_str(), &end);
    if (!end || *end != '\0') {
        SetTokenizerError(tokenizer, "expected number, got '" + token + "'");
        return false;
    }
    return true;
}

bool ParseVector3(MapTokenizer& tokenizer, Vector3& out) {
    if (!ExpectToken(tokenizer, "(")) {
        return false;
    }
    if (!ParseFloatToken(tokenizer, out.x) ||
        !ParseFloatToken(tokenizer, out.y) ||
        !ParseFloatToken(tokenizer, out.z)) {
        return false;
    }
    return ExpectToken(tokenizer, ")");
}

bool ParseTextureAxis(MapTokenizer& tokenizer, Vector3& axis, float& offset) {
    if (!ExpectToken(tokenizer, "[")) {
        return false;
    }
    if (!ParseFloatToken(tokenizer, axis.x) ||
        !ParseFloatToken(tokenizer, axis.y) ||
        !ParseFloatToken(tokenizer, axis.z) ||
        !ParseFloatToken(tokenizer, offset)) {
        return false;
    }
    axis = ConvertTBtoRaylib(axis);
    return ExpectToken(tokenizer, "]");
}

bool ParseFace(MapTokenizer& tokenizer, Face& out) {
    out = Face{};
    for (int i = 0; i < 3; ++i) {
        Vector3 point{};
        if (!ParseVector3(tokenizer, point)) {
            return false;
        }
        out.vertices.push_back(point);
    }

    if (!NextToken(tokenizer, out.texture)) {
        SetTokenizerError(tokenizer, "expected texture name");
        return false;
    }
    if (!ParseTextureAxis(tokenizer, out.textureAxes1, out.offsetX) ||
        !ParseTextureAxis(tokenizer, out.textureAxes2, out.offsetY) ||
        !ParseFloatToken(tokenizer, out.rotation) ||
        !ParseFloatToken(tokenizer, out.scaleX) ||
        !ParseFloatToken(tokenizer, out.scaleY)) {
        return false;
    }

    out.normal = CalculateNormal(out.vertices[0], out.vertices[1], out.vertices[2]);
    if (Vector3LengthSq(out.normal) <= 1.0e-8f) {
        SetTokenizerError(tokenizer, "degenerate face plane");
        return false;
    }
    out.plane.normal = out.normal;
    out.plane.d = Vector3DotProduct(out.normal, out.vertices[0]);
    return true;
}

bool ParseBrush(MapTokenizer& tokenizer, Brush& out) {
    out = Brush{};
    while (true) {
        char c = 0;
        if (!PeekChar(tokenizer, c)) {
            SetTokenizerError(tokenizer, "unexpected end of file in brush");
            return false;
        }
        if (c == '}') {
            std::string token;
            NextToken(tokenizer, token);
            return true;
        }
        if (c != '(') {
            SetTokenizerError(tokenizer, "expected face or brush close");
            return false;
        }

        Face face{};
        if (!ParseFace(tokenizer, face)) {
            return false;
        }
        out.faces.push_back(std::move(face));
    }
}
