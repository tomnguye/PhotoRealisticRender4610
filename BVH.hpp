#pragma once

#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>
class Triangle;
struct BVHNode {
    float bounds[6];
    uint32_t first_child;

    // Combines primitive count and split axis into one field to save space so we can fit the node
    // Into 32 bytes.
    uint32_t meta; // [31:24] = splitAxis, [23:0] = primitive count

    static constexpr uint32_t COUNT_MASK = 0x00FFFFFFu;

    uint32_t primitive_count() const {
        return meta & COUNT_MASK;
    }
    uint8_t splitAxis() const {
        return (uint8_t) (meta >> 24);
    }
    void setLeaf(uint32_t count) {
        meta = count & COUNT_MASK; // splitAxis bits left zero
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

// Build node is only used when building the tree, its discarded later.
struct BuildNode {
    Bounds3 bounds;
    BuildNode *children[2] = {nullptr, nullptr};
    Object *object = nullptr;
    float area = 0.f;
    int splitAxis = 0;
    int primOffset = 0;
    int primCount = 0;
};

class BVHAccel {
  public:
    enum class SplitMethod { NAIVE, SAH };

    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::SAH);
    ~BVHAccel() = default;

    // Regular ray interesction
    Intersection Intersect(const Ray &ray) const;

    // Ray intersection for shadow rays. Checks if there is any occlusion not nearest occlusion.
    bool IntersectP(const Ray &ray, float tMax = std::numeric_limits<float>::infinity()) const;

    // Sample a light source
    void Sample(Intersection &pos, float &pdf);

    BuildNode *root = nullptr;

  private:
    // Uses struct of arrays for better memory acess
    struct RayData {
        float org[3];
        float invDir[3];
        int octant[3];
    };
    BuildNode *makeLeaf(BuildNode *node, std::vector<Object *> &primitives, int start, int end);

    bool partitionSAH(std::vector<Object *> &primitives,
                      const std::vector<Bounds3> &primitiveBounds,
                      const std::vector<Vector3f> &primitiveCentroids,
                      const Bounds3 &centroidBounds, const Bounds3 &nodeBounds, int start,
                      int count, int &mid);

    BuildNode *recursiveBuild(std::vector<Object *> &prims, int start, int end);
    uint32_t flatten(BuildNode *node, uint32_t *offset);
    RayData precompute(const Ray &ray) const;

    void intersectTriangleLeaf(const BVHNode &node, const Ray &ray, float &closestT,
                               const Triangle *&hitTriangle, float &hitU, float &hitV) const;

    void intersectObjectLeaf(const BVHNode &node, const Ray &ray, float &closestT,
                             Intersection &result, const Triangle *&hitTriangle) const;

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

    // Node areas is an array for area. each index matches the nodes in nodes[].
    // We use this for light sampling so that BVHNode can stay at 32 bytes.
    std::vector<float> nodeAreas;
    std::vector<BuildNode> buildPool;

    void getSample(BuildNode *node, float p, Intersection &pos, float &pdf);
    float totalArea = 0.f;

    bool leavesAreTriangles = false;
};