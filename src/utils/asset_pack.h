#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct PackagedAssetEntry {
    std::string logicalPath;
    std::string sourcePath;
};

std::string GetCompanionRresPath(const std::string &path);
uint32_t AssetPackResourceId(const std::string &logicalPath);

bool WriteAssetPackRres(const std::string &outputPath,
                        const std::vector<PackagedAssetEntry> &entries,
                        std::string *errorMessage = nullptr);

bool LoadRawAssetFromPack(const std::string &packPath,
                          const std::string &logicalPath,
                          std::vector<unsigned char> &bytes,
                          std::string *extension = nullptr);

struct MipmapLevel {
    int width = 0;
    int height = 0;
    std::vector<unsigned char> pixels; // RGBA8
};

struct MipmapChain {
    std::vector<MipmapLevel> levels;
};

bool WriteAssetPackRresWithMipmaps(const std::string &outputPath,
                                   const std::vector<PackagedAssetEntry> &entries,
                                   std::string *errorMessage = nullptr);

bool LoadMipmappedAssetFromPack(const std::string &packPath,
                                const std::string &logicalPath,
                                MipmapChain &chain);
