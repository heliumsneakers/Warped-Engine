#include "asset_pack.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_set>

#include "stb_image.h"

#define RRES_IMPLEMENTATION
#include "../../lib/rres/src/rres.h"

namespace {

static std::string ReplaceExtension(const std::string &path, const char *ext)
{
    size_t slash = path.find_last_of("/\\");
    size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return path + ext;
    return path.substr(0, dot) + ext;
}

static bool ReadFileBytes(const std::string &path, std::vector<unsigned char> &bytes)
{
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size < 0) {
        fclose(f);
        return false;
    }

    bytes.resize((size_t)size);
    bool ok = size == 0 || fread(bytes.data(), 1, (size_t)size, f) == (size_t)size;
    fclose(f);
    return ok;
}

static unsigned int EncodeExtPart(const char *ext, int start)
{
    unsigned int value = 0;
    for (int i = 0; i < 4; ++i)
    {
        value <<= 8;
        unsigned char c = 0;
        size_t idx = (size_t)(start + i);
        if (idx < std::strlen(ext)) c = (unsigned char) ext[idx];
        value |= c;
    }
    return value;
}

static std::string DecodeExt(unsigned int ext1, unsigned int ext2)
{
    char buf[9] = { 0 };
    buf[0] = (char)((ext1 >> 24) & 0xFF);
    buf[1] = (char)((ext1 >> 16) & 0xFF);
    buf[2] = (char)((ext1 >> 8) & 0xFF);
    buf[3] = (char)(ext1 & 0xFF);
    buf[4] = (char)((ext2 >> 24) & 0xFF);
    buf[5] = (char)((ext2 >> 16) & 0xFF);
    buf[6] = (char)((ext2 >> 8) & 0xFF);
    buf[7] = (char)(ext2 & 0xFF);

    std::string ext;
    for (int i = 0; i < 8 && buf[i] != 0; ++i) ext.push_back(buf[i]);
    return ext;
}

static std::vector<unsigned char> BuildRawChunkBuffer(const std::vector<unsigned char> &rawBytes,
                                                      const std::string &sourcePath)
{
    const char *ext = std::strrchr(sourcePath.c_str(), '.');
    if (!ext) ext = "";

    const unsigned int propCount = 4;
    const unsigned int props[4] = {
        (unsigned int)rawBytes.size(),
        EncodeExtPart(ext, 0),
        EncodeExtPart(ext, 4),
        0
    };

    std::vector<unsigned char> packed(sizeof(unsigned int) + sizeof(props) + rawBytes.size());
    unsigned char *ptr = packed.data();

    std::memcpy(ptr, &propCount, sizeof(propCount));
    ptr += sizeof(propCount);
    std::memcpy(ptr, props, sizeof(props));
    ptr += sizeof(props);
    if (!rawBytes.empty()) std::memcpy(ptr, rawBytes.data(), rawBytes.size());

    return packed;
}

} // namespace

std::string GetCompanionRresPath(const std::string &path)
{
    return ReplaceExtension(path, ".rres");
}

uint32_t AssetPackResourceId(const std::string &logicalPath)
{
    return rresComputeCRC32((const unsigned char *)logicalPath.c_str(), (int)logicalPath.size());
}

bool WriteAssetPackRres(const std::string &outputPath,
                        const std::vector<PackagedAssetEntry> &entries,
                        std::string *errorMessage)
{
    std::unordered_set<std::string> seen;
    std::vector<PackagedAssetEntry> uniqueEntries;
    uniqueEntries.reserve(entries.size());

    for (const PackagedAssetEntry &entry : entries)
    {
        if (entry.logicalPath.empty() || entry.sourcePath.empty()) continue;
        if (!seen.insert(entry.logicalPath).second) continue;
        uniqueEntries.push_back(entry);
    }

    FILE *f = fopen(outputPath.c_str(), "wb");
    if (!f)
    {
        if (errorMessage) *errorMessage = "Failed to open output pack for write: " + outputPath;
        return false;
    }

    rresFileHeader header = { 0 };
    header.id[0] = 'r';
    header.id[1] = 'r';
    header.id[2] = 'e';
    header.id[3] = 's';
    header.version = 100;
    header.chunkCount = (unsigned short)uniqueEntries.size();
    fwrite(&header, sizeof(header), 1, f);

    for (const PackagedAssetEntry &entry : uniqueEntries)
    {
        std::vector<unsigned char> rawBytes;
        if (!ReadFileBytes(entry.sourcePath, rawBytes))
        {
            fclose(f);
            if (errorMessage) *errorMessage = "Failed to read source asset: " + entry.sourcePath;
            return false;
        }

        std::vector<unsigned char> packed = BuildRawChunkBuffer(rawBytes, entry.sourcePath);

        rresResourceChunkInfo info = { 0 };
        info.type[0] = 'R';
        info.type[1] = 'A';
        info.type[2] = 'W';
        info.type[3] = 'D';
        info.id = AssetPackResourceId(entry.logicalPath);
        info.compType = RRES_COMP_NONE;
        info.cipherType = RRES_CIPHER_NONE;
        info.flags = 0;
        info.packedSize = (unsigned int)packed.size();
        info.baseSize = (unsigned int)packed.size();
        info.nextOffset = 0;
        info.reserved = 0;
        info.crc32 = rresComputeCRC32(packed.data(), info.packedSize);

        fwrite(&info, sizeof(info), 1, f);
        fwrite(packed.data(), 1, packed.size(), f);
    }

    fclose(f);
    return true;
}

bool LoadRawAssetFromPack(const std::string &packPath,
                          const std::string &logicalPath,
                          std::vector<unsigned char> &bytes,
                          std::string *extension)
{
    bytes.clear();
    if (extension) extension->clear();

    rresResourceChunk chunk = rresLoadResourceChunk(packPath.c_str(), AssetPackResourceId(logicalPath));
    if (chunk.data.raw == nullptr) return false;

    bool ok = std::memcmp(chunk.info.type, "RAWD", 4) == 0 &&
              chunk.data.propCount >= 4 &&
              chunk.data.props != nullptr;

    if (ok)
    {
        size_t rawSize = (size_t)chunk.data.props[0];
        unsigned char *raw = (unsigned char *)chunk.data.raw;
        bytes.assign(raw, raw + rawSize);
        if (extension)
            *extension = DecodeExt(chunk.data.props[1], chunk.data.props[2]);
    }

    rresUnloadResourceChunk(chunk);
    return ok;
}

// ---------------------------------------------------------------------------
//  Mipmap generation + multi-chunk .rres writing
// ---------------------------------------------------------------------------

static int ComputeMipCount(int w, int h)
{
    return (int)std::floor(std::log2((double)std::max(w, h))) + 1;
}

static MipmapChain GenerateMipmaps(const unsigned char *rgba, int w, int h)
{
    MipmapChain chain;

    // Level 0: base image
    MipmapLevel base;
    base.width = w;
    base.height = h;
    base.pixels.assign(rgba, rgba + (size_t)w * h * 4);
    chain.levels.push_back(std::move(base));

    int mw = w, mh = h;
    while (mw > 1 || mh > 1) {
        const MipmapLevel &prev = chain.levels.back();
        const int pw = prev.width;
        const int ph = prev.height;
        const int nw = std::max(1, pw / 2);
        const int nh = std::max(1, ph / 2);

        MipmapLevel level;
        level.width = nw;
        level.height = nh;
        level.pixels.resize((size_t)nw * nh * 4);

        for (int y = 0; y < nh; ++y) {
            for (int x = 0; x < nw; ++x) {
                const int sx = x * 2;
                const int sy = y * 2;
                // Clamp second sample to edge for non-power-of-two
                const int sx1 = std::min(sx + 1, pw - 1);
                const int sy1 = std::min(sy + 1, ph - 1);

                const unsigned char *p00 = &prev.pixels[((size_t)sy * pw + sx) * 4];
                const unsigned char *p10 = &prev.pixels[((size_t)sy * pw + sx1) * 4];
                const unsigned char *p01 = &prev.pixels[((size_t)sy1 * pw + sx) * 4];
                const unsigned char *p11 = &prev.pixels[((size_t)sy1 * pw + sx1) * 4];

                unsigned char *dst = &level.pixels[((size_t)y * nw + x) * 4];
                for (int c = 0; c < 4; ++c) {
                    dst[c] = (unsigned char)((p00[c] + p10[c] + p01[c] + p11[c] + 2) / 4);
                }
            }
        }

        mw = nw;
        mh = nh;
        chain.levels.push_back(std::move(level));
    }

    return chain;
}

static std::vector<unsigned char> BuildMipChunkBuffer(int width, int height, int mipCount,
                                                       const unsigned char *pixels, size_t pixelBytes)
{
    const unsigned int propCount = 4;
    const unsigned int props[4] = {
        (unsigned int)width,
        (unsigned int)height,
        (unsigned int)mipCount,
        0
    };

    std::vector<unsigned char> packed(sizeof(unsigned int) + sizeof(props) + pixelBytes);
    unsigned char *ptr = packed.data();
    std::memcpy(ptr, &propCount, sizeof(propCount));
    ptr += sizeof(propCount);
    std::memcpy(ptr, props, sizeof(props));
    ptr += sizeof(props);
    if (pixelBytes > 0) std::memcpy(ptr, pixels, pixelBytes);

    return packed;
}

bool WriteAssetPackRresWithMipmaps(const std::string &outputPath,
                                   const std::vector<PackagedAssetEntry> &entries,
                                   std::string *errorMessage)
{
    std::unordered_set<std::string> seen;
    std::vector<PackagedAssetEntry> uniqueEntries;
    uniqueEntries.reserve(entries.size());

    for (const PackagedAssetEntry &entry : entries)
    {
        if (entry.logicalPath.empty() || entry.sourcePath.empty()) continue;
        if (!seen.insert(entry.logicalPath).second) continue;
        uniqueEntries.push_back(entry);
    }

    // Pre-generate all mipmap chains and count total chunks
    struct TextureMips {
        MipmapChain chain;
        std::string logicalPath;
    };
    std::vector<TextureMips> allMips;
    allMips.reserve(uniqueEntries.size());
    unsigned int totalChunks = 0;

    for (const PackagedAssetEntry &entry : uniqueEntries) {
        std::vector<unsigned char> rawBytes;
        if (!ReadFileBytes(entry.sourcePath, rawBytes)) {
            if (errorMessage) *errorMessage = "Failed to read source asset: " + entry.sourcePath;
            return false;
        }

        int w = 0, h = 0, comp = 0;
        unsigned char *pixels = stbi_load_from_memory(rawBytes.data(), (int)rawBytes.size(), &w, &h, &comp, 4);
        if (!pixels) {
            if (errorMessage) *errorMessage = "Failed to decode PNG: " + entry.sourcePath;
            return false;
        }

        TextureMips tm;
        tm.logicalPath = entry.logicalPath;
        tm.chain = GenerateMipmaps(pixels, w, h);
        stbi_image_free(pixels);

        totalChunks += (unsigned int)tm.chain.levels.size();
        printf("[AssetPack] %s: %dx%d, %zu mip levels\n",
               entry.logicalPath.c_str(), w, h, tm.chain.levels.size());

        allMips.push_back(std::move(tm));
    }

    FILE *f = fopen(outputPath.c_str(), "wb");
    if (!f) {
        if (errorMessage) *errorMessage = "Failed to open output pack for write: " + outputPath;
        return false;
    }

    rresFileHeader header = { 0 };
    header.id[0] = 'r'; header.id[1] = 'r'; header.id[2] = 'e'; header.id[3] = 's';
    header.version = 100;
    header.chunkCount = (unsigned short)totalChunks;
    fwrite(&header, sizeof(header), 1, f);

    for (const TextureMips &tm : allMips) {
        const uint32_t resId = AssetPackResourceId(tm.logicalPath);
        const int mipCount = (int)tm.chain.levels.size();

        // We need to write all mip chunks with nextOffset linking.
        // Strategy: write each chunk, then go back and patch nextOffset of the previous chunk.
        long prevInfoPos = -1;

        for (int m = 0; m < mipCount; ++m) {
            const MipmapLevel &level = tm.chain.levels[m];
            const size_t pixelBytes = (size_t)level.width * level.height * 4;

            // Base chunk stores total mipCount; subsequent chunks store 1
            std::vector<unsigned char> packed = BuildMipChunkBuffer(
                level.width, level.height, (m == 0) ? mipCount : 1,
                level.pixels.data(), pixelBytes);

            // Patch previous chunk's nextOffset to point here
            long chunkInfoPos = ftell(f);
            if (prevInfoPos >= 0) {
                long savedPos = ftell(f);
                fseek(f, prevInfoPos + offsetof(rresResourceChunkInfo, nextOffset), SEEK_SET);
                unsigned int offset = (unsigned int)chunkInfoPos;
                fwrite(&offset, sizeof(offset), 1, f);
                fseek(f, savedPos, SEEK_SET);
            }

            rresResourceChunkInfo info = { 0 };
            info.type[0] = 'R'; info.type[1] = 'A'; info.type[2] = 'W'; info.type[3] = 'D';
            info.id = resId;
            info.compType = RRES_COMP_NONE;
            info.cipherType = RRES_CIPHER_NONE;
            info.flags = 0;
            info.packedSize = (unsigned int)packed.size();
            info.baseSize = (unsigned int)packed.size();
            info.nextOffset = 0; // Will be patched by next iteration if not last
            info.reserved = 0;
            info.crc32 = rresComputeCRC32(packed.data(), info.packedSize);

            fwrite(&info, sizeof(info), 1, f);
            fwrite(packed.data(), 1, packed.size(), f);

            prevInfoPos = chunkInfoPos;
        }
    }

    fclose(f);
    return true;
}

bool LoadMipmappedAssetFromPack(const std::string &packPath,
                                const std::string &logicalPath,
                                MipmapChain &chain)
{
    chain.levels.clear();

    rresResourceMulti multi = rresLoadResourceMulti(packPath.c_str(), AssetPackResourceId(logicalPath));
    if (multi.count == 0 || multi.chunks == nullptr) return false;

    // Check if this is a new-style mipmapped resource (props[2] = mipCount > 1 or multi.count > 1)
    // or a legacy raw PNG resource
    const rresResourceChunk &first = multi.chunks[0];
    const bool isRawD = std::memcmp(first.info.type, "RAWD", 4) == 0;

    if (!isRawD || first.data.propCount < 4 || first.data.props == nullptr) {
        rresUnloadResourceMulti(multi);
        return false;
    }

    // Heuristic: new-style mipmap chunks have width/height in props[0]/[1] and mipCount in props[2].
    // Legacy chunks have fileSize in props[0] and encoded extension in props[1]/[2].
    // For new-style, multi.count should match mipCount in the base chunk.
    const unsigned int prop0 = first.data.props[0];
    const unsigned int prop2 = first.data.props[2];

    if (multi.count > 1 && prop2 > 1) {
        // New-style mipmapped resource
        for (unsigned int i = 0; i < multi.count; ++i) {
            const rresResourceChunk &chunk = multi.chunks[i];
            if (chunk.data.propCount < 4 || chunk.data.props == nullptr || chunk.data.raw == nullptr) continue;

            MipmapLevel level;
            level.width = (int)chunk.data.props[0];
            level.height = (int)chunk.data.props[1];
            const size_t pixelBytes = (size_t)level.width * level.height * 4;

            // The raw data starts after propCount + props in the packed buffer,
            // but rresLoadResourceChunkData already parses this for us.
            const unsigned char *raw = (const unsigned char *)chunk.data.raw;
            level.pixels.assign(raw, raw + pixelBytes);
            chain.levels.push_back(std::move(level));
        }

        rresUnloadResourceMulti(multi);
        return !chain.levels.empty();
    }

    // Legacy single-chunk: raw PNG bytes — decode and return as single mip level
    const size_t rawSize = (size_t)prop0;
    const unsigned char *raw = (const unsigned char *)first.data.raw;

    int w = 0, h = 0, comp = 0;
    unsigned char *pixels = stbi_load_from_memory(raw, (int)rawSize, &w, &h, &comp, 4);
    rresUnloadResourceMulti(multi);

    if (!pixels) return false;

    MipmapLevel level;
    level.width = w;
    level.height = h;
    level.pixels.assign(pixels, pixels + (size_t)w * h * 4);
    stbi_image_free(pixels);

    chain.levels.push_back(std::move(level));
    return true;
}
