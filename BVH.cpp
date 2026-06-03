#include "BVH.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <limits>
#include <utility>
#include <vector>

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p)) {
    if (primitives.empty())
        return;

    leavesAreTriangles = (dynamic_cast<Triangle *>(primitives[0]) != nullptr);

    time_t start, stop;
    time(&start);

    buildPool.reserve(2 * primitives.size());
    orderedPrims.reserve(primitives.size());

    std::vector<Object *> work = primitives;
    root = recursiveBuild(work, 0, (int) work.size());

    nodes.resize(buildPool.size());
    nodeAreas.resize(buildPool.size());
    uint32_t offset = 0;
    flatten(root, &offset);

    totalArea = root->area;

    buildPool.clear();
    buildPool.shrink_to_fit();

    time(&stop);
    double diff = difftime(stop, start);
    printf("BVH built: %d nodes, %zu prims — %ih %im %is\n", (int) nodes.size(), primitives.size(),
           (int) diff / 3600, ((int) diff / 60) % 60, (int) diff % 60);
}

BuildNode *BVHAccel::makeLeaf(BuildNode *node, std::vector<Object *> &primitives, int start,
                              int end) {
    node->primOffset = (int) orderedPrims.size();
    node->primCount = end - start;
    node->area = 0.f;
    for (int i = start; i < end; i++) {
        node->area += primitives[i]->getArea();
        orderedPrims.push_back(primitives[i]);
    }
    if (node->primCount == 1)
        node->object = primitives[start];
    return node;
}

bool BVHAccel::partitionSAH(std::vector<Object *> &primitives,
                            const std::vector<Bounds3> &primitiveBounds,
                            const std::vector<Vector3f> &primitiveCentroids,
                            const Bounds3 &centroidBounds, const Bounds3 &nodeBounds, int start,
                            int count, int &mid) {
    constexpr int NUM_BUCKETS = 12;
    struct Bucket {
        int count = 0;
        Bounds3 bounds;
    };

    float bestCost = std::numeric_limits<float>::infinity();
    int bestAxis = -1;
    int bestBucketSplit = NUM_BUCKETS / 2;
    const float parentSurfaceArea = nodeBounds.SurfaceArea();

    for (int testAxis = 0; testAxis < 3; testAxis++) {
        const float centroidMin = centroidBounds.pMin[testAxis];
        const float centroidSpan = centroidBounds.pMax[testAxis] - centroidMin;
        if (centroidSpan == 0.f)
            continue;

        Bucket buckets[NUM_BUCKETS];
        for (int i = 0; i < count; i++) {
            float relativePosition = (primitiveCentroids[i][testAxis] - centroidMin) / centroidSpan;
            int bucketIndex =
                std::clamp((int) (NUM_BUCKETS * relativePosition), 0, NUM_BUCKETS - 1);
            buckets[bucketIndex].count++;
            buckets[bucketIndex].bounds = Union(buckets[bucketIndex].bounds, primitiveBounds[i]);
        }

        float leftSurfaceArea[NUM_BUCKETS - 1];
        float rightSurfaceArea[NUM_BUCKETS - 1];
        int leftCount[NUM_BUCKETS - 1];
        int rightCount[NUM_BUCKETS - 1];

        Bounds3 leftBounds;
        int leftRunning = 0;
        for (int i = 0; i < NUM_BUCKETS - 1; i++) {
            leftBounds = Union(leftBounds, buckets[i].bounds);
            leftRunning += buckets[i].count;
            leftSurfaceArea[i] = leftBounds.SurfaceArea();
            leftCount[i] = leftRunning;
        }

        Bounds3 rightBounds;
        int rightRunning = 0;
        for (int i = NUM_BUCKETS - 2; i >= 0; i--) {
            rightBounds = Union(rightBounds, buckets[i + 1].bounds);
            rightRunning += buckets[i + 1].count;
            rightSurfaceArea[i] = rightBounds.SurfaceArea();
            rightCount[i] = rightRunning;
        }

        for (int i = 0; i < NUM_BUCKETS - 1; i++) {
            if (leftCount[i] == 0 || rightCount[i] == 0)
                continue;
            float cost =
                1.f + (leftCount[i] * leftSurfaceArea[i] + rightCount[i] * rightSurfaceArea[i]) /
                          parentSurfaceArea;
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = testAxis;
                bestBucketSplit = i;
            }
        }
    }

    if (bestAxis == -1)
        return false;

    const float centroidMin = centroidBounds.pMin[bestAxis];
    const float centroidSpan = centroidBounds.pMax[bestAxis] - centroidMin;
    const float splitEdge = centroidMin + (bestBucketSplit + 1) * centroidSpan / NUM_BUCKETS;

    Object **first = primitives.data() + start;
    Object **middle = std::partition(first, first + count, [&](Object *obj) {
        return obj->getBounds().Centroid()[bestAxis] < splitEdge;
    });

    int leftSize = (int) (middle - first);
    if (leftSize == 0 || leftSize == count)
        return false;

    mid = start + leftSize;
    return true;
}

BuildNode *BVHAccel::recursiveBuild(std::vector<Object *> &primitives, int start, int end) {
    buildPool.emplace_back();
    BuildNode *node = &buildPool.back();
    const int count = end - start;

    std::vector<Bounds3> primitiveBounds(count);
    std::vector<Vector3f> primitiveCentroids(count);
    Bounds3 centroidBounds;
    for (int i = 0; i < count; i++) {
        primitiveBounds[i] = primitives[start + i]->getBounds();
        primitiveCentroids[i] = primitiveBounds[i].Centroid();
        node->bounds = Union(node->bounds, primitiveBounds[i]);
        centroidBounds = Union(centroidBounds, primitiveCentroids[i]);
    }

    const int splitAxis = centroidBounds.maxExtent();
    if (count <= maxPrimsInNode || centroidBounds.pMax[splitAxis] == centroidBounds.pMin[splitAxis])
        return makeLeaf(node, primitives, start, end);

    node->splitAxis = splitAxis;
    int mid = (start + end) / 2;

    bool partitioned = false;
    if (splitMethod == SplitMethod::SAH && count > 4)
        partitioned = partitionSAH(primitives, primitiveBounds, primitiveCentroids, centroidBounds,
                                   node->bounds, start, count, mid);

    if (!partitioned) {
        auto compareOnAxis = [splitAxis](Object *lhs, Object *rhs) {
            return lhs->getBounds().Centroid()[splitAxis] < rhs->getBounds().Centroid()[splitAxis];
        };
        std::nth_element(primitives.data() + start, primitives.data() + mid,
                         primitives.data() + end, compareOnAxis);
    }

    node->children[0] = recursiveBuild(primitives, start, mid);
    node->children[1] = recursiveBuild(primitives, mid, end);
    node->area = node->children[0]->area + node->children[1]->area;
    return node;
}

uint32_t BVHAccel::flatten(BuildNode *node, uint32_t *offset) {
    BVHNode &ln = nodes[*offset];
    ln.setBounds(node->bounds);
    nodeAreas[*offset] = node->area;

    uint32_t myOffset = (*offset)++;

    if (node->primCount > 0) {
        ln.first_child = (uint32_t) node->primOffset;
        ln.setLeaf((uint32_t) node->primCount);
    } else {
        ln.setInterior((uint8_t) node->splitAxis);
        flatten(node->children[0], offset);
        ln.first_child = flatten(node->children[1], offset);
    }

    return myOffset;
}

BVHAccel::RayData BVHAccel::precompute(const Ray &ray) const {
    RayData rayData;
    for (int a = 0; a < 3; a++) {
        float d = ray.direction[a];
        rayData.org[a] = ray.origin[a];
        rayData.invDir[a] = (d != 0.f) ? (1.f / d) : std::numeric_limits<float>::infinity();
        rayData.octant[a] = (d < 0.f) ? 1 : 0;
    }
    return rayData;
}

void BVHAccel::intersectTriangleLeaf(const BVHNode &node, const Ray &ray, float &closestT,
                                     const Triangle *&hitTriangle, float &hitU, float &hitV) const {
    for (uint32_t i = 0; i < node.primitive_count(); i++) {
        const Triangle *tri = static_cast<const Triangle *>(orderedPrims[node.first_child + i]);
        Triangle::TriangleHit h = tri->hitTest(ray);
        if (h.happened && h.t < closestT) {
            closestT = h.t;
            hitTriangle = tri;
            hitU = h.u;
            hitV = h.v;
        }
    }
}

void BVHAccel::intersectObjectLeaf(const BVHNode &node, const Ray &ray, float &closestT,
                                   Intersection &result, const Triangle *&hitTriangle) const {
    for (uint32_t i = 0; i < node.primitive_count(); i++) {
        Intersection tmp = orderedPrims[node.first_child + i]->getIntersection(ray);
        if (tmp.happened && tmp.tnear < closestT) {
            closestT = (float) tmp.tnear;
            result = tmp;
            hitTriangle = nullptr;
        }
    }
}

Intersection BVHAccel::Intersect(const Ray &ray) const {
    Intersection result;
    if (nodes.empty())
        return result;

    const RayData rayData = precompute(ray);

    struct StackEntry {
        uint32_t idx;
        float tBox;
    };
    StackEntry stack[64];
    int stackPtr = 0;

    uint32_t nodeIdx = 0;
    float closestT = std::numeric_limits<float>::infinity();
    const Triangle *hitTriangle = nullptr;
    float hitU = 0.f, hitV = 0.f;

    while (true) {
        const BVHNode &node = nodes[nodeIdx];

        if (node.isLeaf()) {
            if (leavesAreTriangles)
                intersectTriangleLeaf(node, ray, closestT, hitTriangle, hitU, hitV);
            else
                intersectObjectLeaf(node, ray, closestT, result, hitTriangle);
        } else {
            uint32_t left = nodeIdx + 1;
            uint32_t right = node.first_child;
            float tLeft = nodeIntersect(nodes[left], rayData);
            float tRight = nodeIntersect(nodes[right], rayData);

            uint32_t nearChild = left;
            uint32_t farChild = right;
            float tNear = tLeft;
            float tFar = tRight;
            if (tRight < tLeft) {
                nearChild = right;
                farChild = left;
                tNear = tRight;
                tFar = tLeft;
            }

            if (tNear < closestT) {
                if (tFar < closestT)
                    stack[stackPtr++] = {farChild, tFar};
                nodeIdx = nearChild;
                continue;
            }
        }

        nodeIdx = 0;
        bool foundNext = false;
        while (stackPtr > 0) {
            StackEntry entry = stack[--stackPtr];
            if (entry.tBox < closestT) {
                nodeIdx = entry.idx;
                foundNext = true;
                break;
            }
        }
        if (!foundNext)
            break;
    }

    if (hitTriangle)
        result = hitTriangle->finalize(ray, closestT, hitU, hitV);

    return result;
}

bool BVHAccel::IntersectP(const Ray &ray, float tMaxDist) const {
    if (nodes.empty())
        return false;

    const RayData rayData = precompute(ray);

    uint32_t stack[64];
    int stackPtr = 0;
    uint32_t nodeIdx = 0;

    while (true) {
        const BVHNode &node = nodes[nodeIdx];

        float tBox = nodeIntersect(node, rayData);
        if (tBox < tMaxDist) {
            if (node.isLeaf()) {
                for (uint32_t i = 0; i < node.primitive_count(); i++) {
                    float t = orderedPrims[node.first_child + i]->intersectT(ray, tMaxDist);
                    if (t >= 0.f && t < tMaxDist)
                        return true;
                }
            } else {
                uint32_t left = nodeIdx + 1;
                uint32_t right = node.first_child;
                uint32_t near, far;
                if (rayData.octant[node.splitAxis()] == 0) {
                    near = left;
                    far = right;
                } else {
                    near = right;
                    far = left;
                }
                if (nodeIntersect(nodes[far], rayData) < tMaxDist)
                    stack[stackPtr++] = far;
                nodeIdx = near;
                continue;
            }
        }

        if (stackPtr == 0)
            break;
        nodeIdx = stack[--stackPtr];
    }

    return false;
}

void BVHAccel::getSample(BuildNode *node, float p, Intersection &pos, float &pdf) {
    uint32_t nodeIdx = 0;

    while (true) {
        const BVHNode &node = nodes[nodeIdx];

        if (node.isLeaf()) {
            for (uint32_t i = 0; i < node.primitive_count(); i++) {
                Object *obj = orderedPrims[node.first_child + i];
                p -= obj->getArea();
                if (p <= 0.f || i == node.primitive_count() - 1) {
                    obj->Sample(pos, pdf);
                    return;
                }
            }
            return;
        }

        const float leftArea = nodeAreas[nodeIdx + 1];
        if (p < leftArea)
            nodeIdx = nodeIdx + 1;
        else {
            p -= leftArea;
            nodeIdx = node.first_child;
        }
    }
}

void BVHAccel::Sample(Intersection &pos, float &pdf) {
    float p = get_random_float() * totalArea;
    getSample(nullptr, p, pos, pdf);
    pdf /= totalArea;
}