#pragma once

#include "../utils/bsp_format.h"
#include "../utils/map_parser.h"

#include <cstdint>
#include <functional>
#include <vector>

struct StructuralBSPData {
    BSPTreeHeader         tree{};
    std::vector<BSPPlane> planes;
    std::vector<BSPFace>  faces;
    std::vector<BSPVec3>  faceVerts;
    std::vector<BSPNode>  nodes;
    std::vector<BSPLeaf>  leaves;
    std::vector<uint32_t> faceRefs;
    std::vector<MapPolygon> splitPolygons;
};

StructuralBSPData BuildStructuralBSP(const std::vector<MapPolygon>& polys,
                                     const std::function<uint32_t(const std::string&)>& resolveTextureIndex);
