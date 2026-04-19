#include "map_tokenizer.h"

#include <cctype>

void SetTokenizerError(MapTokenizer& tokenizer, const std::string& message) {
    if (tokenizer.error.empty()) {
        tokenizer.error = "line " + std::to_string(tokenizer.line) + ": " + message;
    }
}

static int GetChar(MapTokenizer& tokenizer) {
    const int c = tokenizer.in.get();
    if (c == '\n') {
        ++tokenizer.line;
    }
    return c;
}

static void UngetChar(MapTokenizer& tokenizer, int c) {
    if (c == EOF) {
        return;
    }
    if (c == '\n') {
        --tokenizer.line;
    }
    tokenizer.in.unget();
}

static bool SkipWhitespaceAndComments(MapTokenizer& tokenizer) {
    while (true) {
        int c = GetChar(tokenizer);
        while (c != EOF && std::isspace((unsigned char)c)) {
            c = GetChar(tokenizer);
        }
        if (c != '/') {
            UngetChar(tokenizer, c);
            return c != EOF;
        }

        const int next = GetChar(tokenizer);
        if (next == '/') {
            while ((c = GetChar(tokenizer)) != EOF && c != '\n') {}
            continue;
        }
        if (next == '*') {
            int prev = 0;
            bool closed = false;
            while ((c = GetChar(tokenizer)) != EOF) {
                if (prev == '*' && c == '/') {
                    closed = true;
                    break;
                }
                prev = c;
            }
            if (!closed) {
                SetTokenizerError(tokenizer, "unterminated block comment");
                return false;
            }
            continue;
        }

        UngetChar(tokenizer, next);
        UngetChar(tokenizer, c);
        return true;
    }
}

bool PeekChar(MapTokenizer& tokenizer, char& out) {
    if (!SkipWhitespaceAndComments(tokenizer)) {
        return false;
    }
    const int c = GetChar(tokenizer);
    if (c == EOF) {
        return false;
    }
    out = (char)c;
    UngetChar(tokenizer, c);
    return true;
}

bool NextQuotedString(MapTokenizer& tokenizer, std::string& out) {
    out.clear();
    if (!SkipWhitespaceAndComments(tokenizer)) {
        SetTokenizerError(tokenizer, "expected quoted string");
        return false;
    }
    int c = GetChar(tokenizer);
    if (c != '"') {
        SetTokenizerError(tokenizer, "expected quoted string");
        return false;
    }

    while ((c = GetChar(tokenizer)) != EOF) {
        if (c == '"') {
            return true;
        }
        if (c == '\\') {
            const int escaped = GetChar(tokenizer);
            if (escaped == EOF) {
                break;
            }
            if (escaped == '"' || escaped == '\\') {
                out.push_back((char)escaped);
            } else {
                out.push_back('\\');
                out.push_back((char)escaped);
            }
        } else {
            out.push_back((char)c);
        }

        if (out.size() > MAP_MAX_TOKEN_LENGTH) {
            SetTokenizerError(tokenizer, "quoted string exceeds 512 characters");
            return false;
        }
    }

    SetTokenizerError(tokenizer, "unterminated quoted string");
    return false;
}

bool NextToken(MapTokenizer& tokenizer, std::string& out) {
    out.clear();
    if (!SkipWhitespaceAndComments(tokenizer)) {
        return false;
    }

    int c = GetChar(tokenizer);
    if (c == EOF) {
        return false;
    }

    if (c == '"') {
        UngetChar(tokenizer, c);
        return NextQuotedString(tokenizer, out);
    }

    if (c == '{' || c == '}' || c == '(' || c == ')' || c == '[' || c == ']') {
        out.push_back((char)c);
        return true;
    }

    while (c != EOF) {
        if (std::isspace((unsigned char)c) || c == '{' || c == '}' ||
            c == '(' || c == ')' || c == '[' || c == ']' || c == '"') {
            UngetChar(tokenizer, c);
            break;
        }
        out.push_back((char)c);
        if (out.size() > MAP_MAX_TOKEN_LENGTH) {
            SetTokenizerError(tokenizer, "token exceeds 512 characters");
            return false;
        }
        c = GetChar(tokenizer);
    }

    return !out.empty();
}
