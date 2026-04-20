#include "structural_bsp.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <limits>
#include <unordered_map>

namespace {

static constexpr float kPlaneEpsilon = 0.05f;
static constexpr int kMaxDepth = 64;

struct BuildFace {
    MapPolygon poly;
    int planeIndex = -1;
};

struct BuildBounds {
    Vector3 min{
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    };
    Vector3 max{
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max()
    };
};

enum FaceSide {
    FACE_COPLANAR = 0,
    FACE_FRONT,
    FACE_BACK,
    FACE_SPANNING
};

struct PlaneClassification {
    int frontCount = 0;
    int backCount = 0;
    int coplanarCount = 0;
    int spanningCount = 0;
};

static void ExtendBounds(BuildBounds* bounds, const Vector3& p) {
    bounds->min.x = std::min(bounds->min.x, p.x);
    bounds->min.y = std::min(bounds->min.y, p.y);
    bounds->min.z = std::min(bounds->min.z, p.z);
    bounds->max.x = std::max(bounds->max.x, p.x);
    bounds->max.y = std::max(bounds->max.y, p.y);
    bounds->max.z = std::max(bounds->max.z, p.z);
}

static BuildBounds ComputeFaceBounds(const std::vector<BuildFace>& faces, const std::vector<uint32_t>& faceIds) {
    BuildBounds bounds;
    if (faceIds.empty()) {
        bounds.min = Vector3Zero();
        bounds.max = Vector3Zero();
        return bounds;
    }
    for (uint32_t faceId : faceIds) {
        const BuildFace& face = faces[faceId];
        for (const Vector3& v : face.poly.verts) {
            ExtendBounds(&bounds, v);
        }
    }
    return bounds;
}

static float PlaneDistance(const BSPPlane& plane, const Vector3& p) {
    return plane.nx * p.x + plane.ny * p.y + plane.nz * p.z + plane.d;
}

static FaceSide ClassifyFaceAgainstPlane(const BuildFace& face, const BSPPlane& plane) {
    bool hasFront = false;
    bool hasBack = false;
    for (const Vector3& v : face.poly.verts) {
        const float dist = PlaneDistance(plane, v);
        if (dist > kPlaneEpsilon) {
            hasFront = true;
        } else if (dist < -kPlaneEpsilon) {
            hasBack = true;
        }
        if (hasFront && hasBack) {
            return FACE_SPANNING;
        }
    }
    if (hasFront) {
        return FACE_FRONT;
    }
    if (hasBack) {
        return FACE_BACK;
    }
    return FACE_COPLANAR;
}

class Builder {
public:
    explicit Builder(const std::function<uint32_t(const std::string&)>& resolveTextureIndexIn)
        : resolveTextureIndex(resolveTextureIndexIn) {
    }

    StructuralBSPData Build(const std::vector<MapPolygon>& polys) {
        InitializeFaces(polys);
        std::vector<uint32_t> faceIds(facePool.size());
        for (size_t i = 0; i < faceIds.size(); ++i) {
            faceIds[i] = (uint32_t)i;
        }

        out.tree.rootChild = BuildNode(faceIds, 0);
        out.tree.outsideLeaf = -1;
        out.tree.reserved0 = 0;
        out.tree.reserved1 = 0;
        SerializeFaces();
        return out;
    }

private:
    const std::function<uint32_t(const std::string&)>& resolveTextureIndex;
    StructuralBSPData out;
    std::vector<BuildFace> facePool;

    void InitializeFaces(const std::vector<MapPolygon>& polys) {
        facePool.reserve(polys.size());
        for (const MapPolygon& poly : polys) {
            if (poly.verts.size() < 3 || Vector3LengthSq(poly.normal) <= 1e-8f) {
                continue;
            }
            BuildFace face;
            face.poly = poly;
            face.planeIndex = FindOrAddPlane(poly.normal, -Vector3DotProduct(poly.normal, poly.verts[0]));
            facePool.push_back(std::move(face));
        }
    }

    int FindOrAddPlane(const Vector3& normal, float d) {
        for (size_t i = 0; i < out.planes.size(); ++i) {
            const BSPPlane& plane = out.planes[i];
            if (fabsf(plane.nx - normal.x) <= kPlaneEpsilon &&
                fabsf(plane.ny - normal.y) <= kPlaneEpsilon &&
                fabsf(plane.nz - normal.z) <= kPlaneEpsilon &&
                fabsf(plane.d - d) <= kPlaneEpsilon)
            {
                return (int)i;
            }
        }
        BSPPlane plane{};
        plane.nx = normal.x;
        plane.ny = normal.y;
        plane.nz = normal.z;
        plane.d = d;
        out.planes.push_back(plane);
        return (int)out.planes.size() - 1;
    }

    int32_t EncodeLeafIndex(uint32_t leafIndex) const {
        return -1 - (int32_t)leafIndex;
    }

    uint32_t AppendFaceRefs(const std::vector<uint32_t>& refs) {
        const uint32_t first = (uint32_t)out.faceRefs.size();
        out.faceRefs.insert(out.faceRefs.end(), refs.begin(), refs.end());
        return first;
    }

    uint32_t BuildLeaf(const std::vector<uint32_t>& faceIds) {
        const BuildBounds bounds = ComputeFaceBounds(facePool, faceIds);
        BSPLeaf leaf{};
        leaf.contents = 0;
        leaf.minX = bounds.min.x;
        leaf.minY = bounds.min.y;
        leaf.minZ = bounds.min.z;
        leaf.maxX = bounds.max.x;
        leaf.maxY = bounds.max.y;
        leaf.maxZ = bounds.max.z;
        leaf.firstFaceRef = AppendFaceRefs(faceIds);
        leaf.faceRefCount = (uint32_t)faceIds.size();
        out.leaves.push_back(leaf);
        return (uint32_t)out.leaves.size() - 1;
    }

    bool ChooseSplitPlane(const std::vector<uint32_t>& faceIds, int* outPlaneIndex) const {
        float bestScore = FLT_MAX;
        int bestPlane = -1;
        for (uint32_t faceId : faceIds) {
            const BuildFace& candidateFace = facePool[faceId];
            const BSPPlane& plane = out.planes[(size_t)candidateFace.planeIndex];
            PlaneClassification stats;
            for (uint32_t otherId : faceIds) {
                switch (ClassifyFaceAgainstPlane(facePool[otherId], plane)) {
                    case FACE_FRONT: ++stats.frontCount; break;
                    case FACE_BACK: ++stats.backCount; break;
                    case FACE_COPLANAR: ++stats.coplanarCount; break;
                    case FACE_SPANNING: ++stats.spanningCount; break;
                }
            }
            if (stats.frontCount == 0 || stats.backCount == 0) {
                continue;
            }
            const float score = (float)stats.spanningCount * 8.0f +
                                (float)abs(stats.frontCount - stats.backCount) +
                                (float)stats.coplanarCount * 0.25f;
            if (score < bestScore) {
                bestScore = score;
                bestPlane = candidateFace.planeIndex;
            }
        }
        if (bestPlane < 0) {
            return false;
        }
        *outPlaneIndex = bestPlane;
        return true;
    }

    int32_t BuildNode(const std::vector<uint32_t>& faceIds, int depth) {
        if (faceIds.empty() || depth >= kMaxDepth || faceIds.size() <= 2) {
            return EncodeLeafIndex(BuildLeaf(faceIds));
        }

        int splitPlaneIndex = -1;
        if (!ChooseSplitPlane(faceIds, &splitPlaneIndex)) {
            return EncodeLeafIndex(BuildLeaf(faceIds));
        }

        const BSPPlane& plane = out.planes[(size_t)splitPlaneIndex];
        std::vector<uint32_t> nodeFaces;
        std::vector<uint32_t> frontFaces;
        std::vector<uint32_t> backFaces;

        for (uint32_t faceId : faceIds) {
            const BuildFace& face = facePool[faceId];
            switch (ClassifyFaceAgainstPlane(face, plane)) {
                case FACE_COPLANAR:
                    nodeFaces.push_back(faceId);
                    break;
                case FACE_FRONT:
                    frontFaces.push_back(faceId);
                    break;
                case FACE_BACK:
                    backFaces.push_back(faceId);
                    break;
                case FACE_SPANNING:
                    // The map parser owns CSG and polygon clipping. The BSP
                    // tree is only a spatial index, so spanning polygons stay
                    // intact and are referenced from this node.
                    nodeFaces.push_back(faceId);
                    break;
            }
        }

        if (frontFaces.empty() || backFaces.empty()) {
            return EncodeLeafIndex(BuildLeaf(faceIds));
        }

        const BuildBounds bounds = ComputeFaceBounds(facePool, faceIds);
        BSPNode node{};
        node.planeIndex = splitPlaneIndex;
        node.minX = bounds.min.x;
        node.minY = bounds.min.y;
        node.minZ = bounds.min.z;
        node.maxX = bounds.max.x;
        node.maxY = bounds.max.y;
        node.maxZ = bounds.max.z;
        node.firstFaceRef = AppendFaceRefs(nodeFaces);
        node.faceRefCount = (uint32_t)nodeFaces.size();
        out.nodes.push_back(node);
        const uint32_t nodeIndex = (uint32_t)out.nodes.size() - 1;

        out.nodes[nodeIndex].frontChild = BuildNode(frontFaces, depth + 1);
        out.nodes[nodeIndex].backChild = BuildNode(backFaces, depth + 1);
        return (int32_t)nodeIndex;
    }

    void SerializeFaces() {
        std::vector<int32_t> remap(facePool.size(), -1);
        out.faces.reserve(out.faceRefs.size());
        for (uint32_t& ref : out.faceRefs) {
            if ((size_t)ref >= facePool.size()) {
                ref = 0;
                continue;
            }
            if (remap[ref] < 0) {
                const BuildFace& face = facePool[ref];
                const BSPPlane& plane = out.planes[(size_t)std::max(face.planeIndex, 0)];
                remap[ref] = (int32_t)out.faces.size();

                MapPolygon sourcePoly = face.poly;
                sourcePoly.normal = { plane.nx, plane.ny, plane.nz };

                BSPFace outFace{};
                outFace.planeIndex = (uint32_t)std::max(0, face.planeIndex);
                outFace.textureIndex = resolveTextureIndex(sourcePoly.texture);
                outFace.firstVertex = (uint32_t)out.faceVerts.size();
                outFace.vertexCount = (uint32_t)sourcePoly.verts.size();
                outFace.sourceEntityId = sourcePoly.sourceEntityId;
                outFace.sourceBrushId = sourcePoly.sourceBrushId;
                outFace.sourceFaceIndex = sourcePoly.sourceFaceIndex;
                outFace.flags = 0;
                out.faces.push_back(outFace);
                for (const Vector3& v : sourcePoly.verts) {
                    out.faceVerts.push_back({ v.x, v.y, v.z });
                }
            }
            ref = (uint32_t)remap[ref];
        }
    }
};

} // namespace

StructuralBSPData BuildStructuralBSP(const std::vector<MapPolygon>& polys,
                                     const std::function<uint32_t(const std::string&)>& resolveTextureIndex)
{
    Builder builder(resolveTextureIndex);
    return builder.Build(polys);
}
