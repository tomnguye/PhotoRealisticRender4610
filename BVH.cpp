#include "BVH.hpp"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <numeric>
#include <limits>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode))
    , splitMethod(splitMethod)
    , primitives(std::move(p))
{
    if (primitives.empty()) return;

    time_t start, stop;
    time(&start);

    // Reserve the pool up front so pointers into it never get invalidated
    // by a realloc. Worst case is 2*N-1 nodes for N primitives.
    buildPool.reserve(2 * primitives.size());
    orderedPrims.reserve(primitives.size());

    // Work on a mutable copy so we can nth_element / partition in place.
    std::vector<Object*> work = primitives;
    root = recursiveBuild(work, 0, (int)work.size());

    // Flatten the pointer tree into the cache-friendly flat array.
    // Two children are always adjacent so they share a cache line.
    nodes.resize(buildPool.size());
    uint32_t offset = 0;
    flatten(root, &offset);

    totalArea = root->area;

    // Build data no longer needed.
    buildPool.clear();
    buildPool.shrink_to_fit();

    time(&stop);
    double diff = difftime(stop, start);
    printf("BVH built: %d nodes, %zu prims — %ih %im %is\n",
        (int)nodes.size(), primitives.size(),
        (int)diff / 3600, ((int)diff / 60) % 60, (int)diff % 60);
}

// ---------------------------------------------------------------------------
// recursiveBuild
//
// Key improvements over the original:
//   - All allocations come from buildPool (no per-node `new`)
//   - Longest axis chosen from centroid bounds, not a cycling counter
//   - SAH evaluated over all 3 axes, picks the globally cheapest split
//   - nth_element (O(n)) replaces full sort (O(n log n)) for naive split
//   - orderedPrims filled in leaf order for contiguous leaf prim ranges
// ---------------------------------------------------------------------------
BuildNode* BVHAccel::recursiveBuild(std::vector<Object*>& prims, int start, int end)
{
    buildPool.emplace_back();
    BuildNode* node = &buildPool.back();

    const int count = end - start;

    // Union of all primitive bounds in [start, end).
    Bounds3 bounds;
    for (int i = start; i < end; i++)
        bounds = Union(bounds, prims[i]->getBounds());
    node->bounds = bounds;

    // --- Leaf ---
    if (count == 1) {
        node->primOffset = (int)orderedPrims.size();
        node->primCount = 1;
        node->object = prims[start];
        node->area = prims[start]->getArea();
        orderedPrims.push_back(prims[start]);
        return node;
    }

    // --- Centroid bounds → longest axis ---
    Bounds3 centBounds;
    for (int i = start; i < end; i++)
        centBounds = Union(centBounds, prims[i]->getBounds().Centroid());
    const int axis = centBounds.maxExtent();
    node->splitAxis = axis;

    int mid = (start + end) / 2;

    // Degenerate: all centroids coincide — can't split meaningfully.
    if (centBounds.pMax[axis] == centBounds.pMin[axis]) {
        // Force a leaf if small enough, otherwise just median-split.
        if (count <= maxPrimsInNode) {
            node->primOffset = (int)orderedPrims.size();
            node->primCount = count;
            node->area = 0.f;
            for (int i = start; i < end; i++) {
                node->area += prims[i]->getArea();
                orderedPrims.push_back(prims[i]);
            }
            return node;
        }
        // else fall through with mid already set to median
    }
    else if (splitMethod == SplitMethod::SAH && count > 4) {
        // -----------------------------------------------------------------
        // Binned SAH: try all 3 axes, pick the globally best split.
        //
        // For each axis we partition primitives into N_BUCKETS buckets
        // based on centroid position, then sweep to find the split that
        // minimises:  C = 1 + (nL * areaL + nR * areaR) / areaParent
        // -----------------------------------------------------------------
        constexpr int N_BUCKETS = 12;

        struct Bucket { int count = 0; Bounds3 bounds; };

        float bestCost = std::numeric_limits<float>::infinity();
        int   bestAxis = axis;
        int   bestSplit = N_BUCKETS / 2;
        const float parentArea = bounds.SurfaceArea();

        for (int a = 0; a < 3; a++) {
            float lo = centBounds.pMin[a];
            float hi = centBounds.pMax[a];
            if (lo == hi) continue;

            Bucket buckets[N_BUCKETS];
            const float span = hi - lo;

            for (int i = start; i < end; i++) {
                float c = prims[i]->getBounds().Centroid()[a];
                int b = std::clamp((int)(N_BUCKETS * (c - lo) / span), 0, N_BUCKETS - 1);
                buckets[b].count++;
                buckets[b].bounds = Union(buckets[b].bounds, prims[i]->getBounds());
            }

            // Prefix scan: left side cumulative area + count.
            float leftArea[N_BUCKETS - 1];
            int   leftCount[N_BUCKETS - 1];
            {
                Bounds3 bL; int cL = 0;
                for (int i = 0; i < N_BUCKETS - 1; i++) {
                    bL = Union(bL, buckets[i].bounds);
                    cL += buckets[i].count;
                    leftArea[i] = bL.SurfaceArea();
                    leftCount[i] = cL;
                }
            }

            // Suffix scan: right side.
            float rightArea[N_BUCKETS - 1];
            int   rightCount[N_BUCKETS - 1];
            {
                Bounds3 bR; int cR = 0;
                for (int i = N_BUCKETS - 2; i >= 0; i--) {
                    bR = Union(bR, buckets[i + 1].bounds);
                    cR += buckets[i + 1].count;
                    rightArea[i] = bR.SurfaceArea();
                    rightCount[i] = cR;
                }
            }

            for (int i = 0; i < N_BUCKETS - 1; i++) {
                if (leftCount[i] == 0 || rightCount[i] == 0) continue;
                float cost = 1.f + (leftCount[i] * leftArea[i] +
                    rightCount[i] * rightArea[i]) / parentArea;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = a;
                    bestSplit = i;
                }
            }
        }

        // Partition in-place on best axis/bucket.
        float lo = centBounds.pMin[bestAxis];
        float span = centBounds.pMax[bestAxis] - lo;
        float edge = lo + (bestSplit + 1) * span / N_BUCKETS;

        Object** midPtr = std::partition(
            prims.data() + start, prims.data() + end,
            [bestAxis, edge](Object* o) {
                return o->getBounds().Centroid()[bestAxis] < edge;
            });

        mid = (int)(midPtr - prims.data());

        // Guard: partition can be degenerate for very flat geometry.
        if (mid == start || mid == end)
            mid = (start + end) / 2;
    }
    else {
        // Naive: nth_element on longest axis — O(n), not O(n log n).
        std::nth_element(
            prims.data() + start, prims.data() + mid, prims.data() + end,
            [axis](Object* a, Object* b) {
                return a->getBounds().Centroid()[axis] < b->getBounds().Centroid()[axis];
            });
    }

    node->children[0] = recursiveBuild(prims, start, mid);
    node->children[1] = recursiveBuild(prims, mid, end);
    node->area = node->children[0]->area + node->children[1]->area;
    return node;
}

// ---------------------------------------------------------------------------
// flatten — depth-first left-first traversal.
// Left child is always at (parentIndex + 1) so it costs nothing to reach.
// Right child index is stored in first_child for interior nodes.
// ---------------------------------------------------------------------------
uint32_t BVHAccel::flatten(BuildNode* node, uint32_t* offset)
{
    BVHNode& ln = nodes[*offset];
    ln.setBounds(node->bounds);

    uint32_t myOffset = (*offset)++;

    if (node->primCount > 0) {
        // Leaf
        ln.first_child = (uint32_t)node->primOffset;
        ln.primitive_count = (uint32_t)node->primCount;
    }
    else {
        // Interior: left child immediately follows, store right child offset.
        ln.primitive_count = 0;
        flatten(node->children[0], offset);                         // left: myOffset+1
        ln.first_child = flatten(node->children[1], offset);        // right: stored here
    }

    return myOffset;
}

// ---------------------------------------------------------------------------
// Precompute per-ray data — called once per ray before traversal.
//
// octant[a] = 1 means ray goes in the negative direction on axis a,
// so bounds[a*2+1] (the max bound) is the near slab and bounds[a*2]
// (the min bound) is the far slab — which lets us avoid a branch inside
// the hot box test loop.
// ---------------------------------------------------------------------------
BVHAccel::RayData BVHAccel::precompute(const Ray& ray) const
{
    RayData rd;
    for (int a = 0; a < 3; a++) {
        float d = ray.direction[a];
        // Avoid division by exactly zero; IEEE ±inf is fine for slab test.
        rd.invDir[a] = (d != 0.f) ? (1.f / d) : std::numeric_limits<float>::infinity();
        rd.org[a] = ray.origin[a];
        rd.octant[a] = (d < 0.f) ? 1 : 0;
    }
    return rd;
}

// ---------------------------------------------------------------------------
// Intersect — iterative traversal, explicit fixed-size stack.
//
// Stack holds indices into nodes[]. 64 levels handles trees with up to
// ~10M well-balanced leaves — increase if you ever hit an assertion.
//
// Traversal order: always visit the child whose bounding box the ray
// enters first (near child), push the far child. This maximises early
// exits via the closestT guard.
// ---------------------------------------------------------------------------
Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection result;
    if (nodes.empty()) return result;

    const RayData rd = precompute(ray);

    uint32_t stack[64];
    int      stackPtr = 0;
    uint32_t nodeIdx = 0;
    float    closestT = std::numeric_limits<float>::infinity();

    while (true) {
        const BVHNode& node = nodes[nodeIdx];

        float tBox = nodeIntersect(node, rd);

        if (tBox < closestT) {
            if (node.isLeaf()) {
                // Test all primitives in the leaf.
                for (uint32_t i = 0; i < node.primitive_count; i++) {
                    Intersection tmp = orderedPrims[node.first_child + i]->getIntersection(ray);
                    if (tmp.happened && tmp.tnear < closestT) {
                        closestT = tmp.tnear;
                        result = tmp;
                    }
                }
            }
            else {
                // Determine near/far child by ray direction on split axis.
                // The split axis is encoded in the node bounds ordering —
                // we derive it from which axis had the largest extent at
                // build time. Here we approximate by checking octant on
                // the dominant axis of the bounding box.
                //
                // Near child: the one the ray enters first.
                // Push far child, continue with near child.
                uint32_t left = nodeIdx + 1;
                uint32_t right = node.first_child;

                // Simple heuristic: visit left first unless ray is going
                // negative and right box is hit earlier.
                // A tighter version would store the split axis in the node —
                // add a uint8_t splitAxis to BVHNode if you want that.
                float tLeft = nodeIntersect(nodes[left], rd);
                float tRight = nodeIntersect(nodes[right], rd);

                // Push far child first (so near is on top of stack).
                if (tLeft <= tRight) {
                    if (tRight < closestT) stack[stackPtr++] = right;
                    nodeIdx = left;
                    continue;
                }
                else {
                    if (tLeft < closestT) stack[stackPtr++] = left;
                    nodeIdx = right;
                    continue;
                }
            }
        }

        if (stackPtr == 0) break;
        nodeIdx = stack[--stackPtr];
    }

    return result;
}

// ---------------------------------------------------------------------------
// IntersectP — shadow ray. Returns true on first hit, skips the rest.
// Identical traversal loop but exits immediately on any primitive hit.
// ---------------------------------------------------------------------------
bool BVHAccel::IntersectP(const Ray& ray) const
{
    if (nodes.empty()) return false;

    const RayData rd = precompute(ray);

    uint32_t stack[64];
    int      stackPtr = 0;
    uint32_t nodeIdx = 0;

    while (true) {
        const BVHNode& node = nodes[nodeIdx];

        if (nodeIntersect(node, rd) < std::numeric_limits<float>::infinity()) {
            if (node.isLeaf()) {
                for (uint32_t i = 0; i < node.primitive_count; i++) {
                    if (orderedPrims[node.first_child + i]->getIntersection(ray).happened)
                        return true;  // early exit — don't care which hit
                }
            }
            else {
                // No need for ordered traversal for shadow rays — just push both.
                stack[stackPtr++] = node.first_child;       // right child
                nodeIdx = nodeIdx + 1;                      // left child
                continue;
            }
        }

        if (stackPtr == 0) break;
        nodeIdx = stack[--stackPtr];
    }

    return false;
}

// ---------------------------------------------------------------------------
// Light sampling — unchanged logic from original, uses build tree.
// ---------------------------------------------------------------------------
void BVHAccel::getSample(BuildNode* node, float p, Intersection& pos, float& pdf)
{
    if (!node->children[0] && !node->children[1]) {
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if (p < node->children[0]->area)
        getSample(node->children[0], p, pos, pdf);
    else
        getSample(node->children[1], p - node->children[0]->area, pos, pdf);
}

void BVHAccel::Sample(Intersection& pos, float& pdf)
{
    float p = std::sqrt(get_random_float()) * totalArea;
    getSample(root, p, pos, pdf);
    pdf /= totalArea;
}