#pragma once

#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include <cstdint>
#include <ctime>
#include <vector>

// ---------------------------------------------------------------------------
// Node layout: 32 bytes exactly — two nodes share one 64-byte cache line.
//
// Children are always stored adjacently:
//   left  child = nodes[first_child]
//   right child = nodes[first_child + 1]
//
// Leaf:     primCount() > 0
//           first_child = index into orderedPrims[]
// Interior: primCount() == 0
//           first_child = index of left child in nodes[]
//
// `meta` packs the primitive count (low 24 bits) and the split axis (high 8
// bits). maxPrimsInNode is capped at 255, so 24 bits is far more than enough
// for the count. Per-node surface area (needed only by light sampling, never
// by ray traversal) lives in a separate parallel array so it does not bloat
// the hot traversal node — that is what gets us back to 32 bytes.
//
// Bounds are stored interleaved as [minX,maxX, minY,maxY, minZ,maxZ].
// This lets the ray-octant trick select the near/far slab per axis with
// a single index flip rather than a branch.
// ---------------------------------------------------------------------------
struct BVHNode {
    float bounds[6];
    uint32_t first_child;
    uint32_t meta; // [31:24] = splitAxis, [23:0] = primitive count

    static constexpr uint32_t COUNT_MASK = 0x00FFFFFFu;

    uint32_t primitive_count() const {
        return meta & COUNT_MASK;
    }
    uint8_t splitAxis() const {
        return (uint8_t) (meta >> 24);
    }
    void setLeaf(uint32_t count) {
        meta = count & COUNT_MASK; // splitAxis bits left zero; unused in leaves
    }
    void setInterior(uint8_t axis) {
        meta = ((uint32_t) axis << 24); // count == 0 marks interior
    }

    bool isLeaf() const {
        return (meta & COUNT_MASK) > 0;
    }

    void setBounds(const Bounds3 &b) {
        bounds[0] = b.pMin.x;
        bounds[1] = b.pMax.x;
        bounds[2] = b.pMin.y;
        bounds[3] = b.pMax.y;
        bounds[4] = b.pMin.z;
        bounds[5] = b.pMax.z;
    }
};
static_assert(sizeof(BVHNode) == 32, "BVHNode must be 32 bytes");

// ---------------------------------------------------------------------------
// Temporary build node — only alive during construction, then discarded.
// All allocated from a pool so there is no heap fragmentation.
// ---------------------------------------------------------------------------
struct BuildNode {
    Bounds3 bounds;
    BuildNode *children[2] = {nullptr, nullptr};
    Object *object = nullptr;
    float area = 0.f;
    int splitAxis = 0;
    int primOffset = 0;
    int primCount = 0;
};

// ---------------------------------------------------------------------------
// BVHAccel
// ---------------------------------------------------------------------------
class BVHAccel {
  public:
    enum class SplitMethod { NAIVE, SAH };

    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::SAH);
    ~BVHAccel() = default;

    Intersection Intersect(const Ray &ray) const;
    bool IntersectP(const Ray &ray, float tMax = std::numeric_limits<float>::infinity()) const;
    void Sample(Intersection &pos, float &pdf);

    // Legacy pointer — into buildPool, valid until destructor.
    BuildNode *root = nullptr;

  private:
    // Per-axis layout so nodeIntersect reads contiguous memory.
    struct RayData {
        float org[3];
        float invDir[3];
        int octant[3]; // 0 = +dir, 1 = -dir; selects near/far slab index
    };

    BuildNode *recursiveBuild(std::vector<Object *> &prims, int start, int end);
    uint32_t flatten(BuildNode *node, uint32_t *offset);
    RayData precompute(const Ray &ray) const;

    // Slab test. bounds layout: [minX,maxX, minY,maxY, minZ,maxZ].
    // octant[a]==0 means ray goes +axis so bounds[a*2] is the near slab.
    inline float nodeIntersect(const BVHNode &node, const RayData &rd) const {
        float tMin = 0.f, tMax = std::numeric_limits<float>::infinity();
        for (int a = 0; a < 3; a++) {
            float t0 = (node.bounds[a * 2 + rd.octant[a]] - rd.org[a]) * rd.invDir[a];
            float t1 = (node.bounds[a * 2 + 1 - rd.octant[a]] - rd.org[a]) * rd.invDir[a];
            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);
        }
        return tMin <= tMax ? tMin : std::numeric_limits<float>::infinity();
    }

    const int maxPrimsInNode;
    const SplitMethod splitMethod;

    std::vector<Object *> primitives;
    std::vector<Object *> orderedPrims;
    std::vector<BVHNode> nodes;
    // Per-node surface area, parallel to nodes[]. Only the light-sampling
    // traversal (getSample) reads this; keeping it out of BVHNode keeps the
    // hot ray-traversal node at 32 bytes.
    std::vector<float> nodeAreas;
    std::vector<BuildNode> buildPool;

    void getSample(BuildNode *node, float p, Intersection &pos, float &pdf);
    float totalArea = 0.f;

    // True when every primitive in this BVH is a leaf Triangle, enabling the
    // deferred-attribute fast path in Intersect(). False for the scene-level
    // BVH whose primitives are MeshTriangle aggregates, which must go through
    // the general getIntersection() path.
    bool leavesAreTriangles = false;
};