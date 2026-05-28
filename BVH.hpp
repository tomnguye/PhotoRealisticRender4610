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
// Leaf:     primitive_count > 0
//           first_child = index into orderedPrims[]
// Interior: primitive_count == 0
//           first_child = index of left child in nodes[]
//
// Bounds are stored interleaved as [minX,maxX, minY,maxY, minZ,maxZ].
// This lets the ray-octant trick select the near/far slab per axis with
// a single index flip rather than a branch.
// ---------------------------------------------------------------------------
struct BVHNode
{
    float bounds[6];          // minX maxX minY maxY minZ maxZ
    uint32_t first_child;     // leaf: prim offset,  interior: left child idx
    uint32_t primitive_count; // 0 = interior node,  >0 = leaf

    bool isLeaf() const { return primitive_count > 0; }

    void setBounds(const Bounds3 &b)
    {
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
struct BuildNode
{
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
class BVHAccel
{
public:
    enum class SplitMethod
    {
        NAIVE,
        SAH
    };

    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::SAH);
    ~BVHAccel() = default;

    Intersection Intersect(const Ray &ray) const;
    bool IntersectP(const Ray &ray) const;
    void Sample(Intersection &pos, float &pdf);

    // Legacy pointer — into buildPool, valid until destructor.
    BuildNode *root = nullptr;

private:
    struct RayData
    {
        float invDir[3];
        float org[3];
        int octant[3]; // 0 or 1: selects near/far bound per axis
    };

    BuildNode *recursiveBuild(std::vector<Object *> &prims, int start, int end);
    uint32_t flatten(BuildNode *node, uint32_t *offset);
    RayData precompute(const Ray &ray) const;

    // Returns tMin of slab intersection, or +inf on miss.
    inline float nodeIntersect(const BVHNode &node, const RayData &rd) const
    {
        float tMin = 0.f, tMax = std::numeric_limits<float>::infinity();
        for (int a = 0; a < 3; a++)
        {
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
    std::vector<BuildNode> buildPool;

    void getSample(BuildNode *node, float p, Intersection &pos, float &pdf);
    float totalArea = 0.f;
};