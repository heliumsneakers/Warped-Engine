#include "asset_pack.h"

#include <cstdio>
#include <cstring>
#include <unordered_set>

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
