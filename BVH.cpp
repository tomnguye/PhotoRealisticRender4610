#include "BVH.hpp"
#include "Triangle.hpp"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <numeric>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p)) {
    if (primitives.empty())
        return;

    // Decide once whether this BVH's primitives are leaf Triangles. If so,
    // Intersect() can use the deferred-attribute fast path. All primitives in a
    // given BVH are homogeneous in this renderer (a mesh's BVH holds Triangles;
    // the scene BVH holds MeshTriangles), so testing the first is sufficient.
    leavesAreTriangles = (dynamic_cast<Triangle *>(primitives[0]) != nullptr);

    time_t start, stop;
    time(&start);

    // Reserve the pool up front so pointers into it never get invalidated
    // by a realloc. Worst case is 2*N-1 nodes for N primitives.
    buildPool.reserve(2 * primitives.size());
    orderedPrims.reserve(primitives.size());

    // Work on a mutable copy so we can nth_element / partition in place.
    std::vector<Object *> work = primitives;
    root = recursiveBuild(work, 0, (int) work.size());

    // Flatten the pointer tree into the cache-friendly flat array.
    // Two children are always adjacent so they share a cache line.
    nodes.resize(buildPool.size());
    nodeAreas.resize(buildPool.size());
    uint32_t offset = 0;
    flatten(root, &offset);

    totalArea = root->area;

    // Build data no longer needed.
    buildPool.clear();
    buildPool.shrink_to_fit();

    time(&stop);
    double diff = difftime(stop, start);
    printf("BVH built: %d nodes, %zu prims — %ih %im %is\n", (int) nodes.size(), primitives.size(),
           (int) diff / 3600, ((int) diff / 60) % 60, (int) diff % 60);
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
BuildNode *BVHAccel::recursiveBuild(std::vector<Object *> &prims, int start, int end) {
    buildPool.emplace_back();
    BuildNode *node = &buildPool.back();

    const int count = end - start;

    // Cache bounds and centroids for [start, end) in one pass.
    // getBounds() may be a virtual call that walks a sub-BVH (e.g. MeshTriangle),
    // so calling it once per primitive instead of 3-5x per level saves a lot.
    std::vector<Bounds3> primBounds(count);
    std::vector<Vector3f> primCentroids(count);

    Bounds3 bounds, centBounds;
    for (int i = 0; i < count; i++) {
        primBounds[i] = prims[start + i]->getBounds();
        primCentroids[i] = primBounds[i].Centroid();
        bounds = Union(bounds, primBounds[i]);
        centBounds = Union(centBounds, primCentroids[i]);
    }
    node->bounds = bounds;

    // --- Leaf ---
    if (count == 1) {
        node->primOffset = (int) orderedPrims.size();
        node->primCount = 1;
        node->object = prims[start];
        node->area = prims[start]->getArea();
        orderedPrims.push_back(prims[start]);
        return node;
    }

    // Stop early and emit a multi-primitive leaf once the subset is small
    // enough. Trades a few extra primitive tests per leaf for far fewer box
    // tests and a shallower tree. Box tests dominate, so this is usually a win.
    if (count <= maxPrimsInNode) {
        node->primOffset = (int) orderedPrims.size();
        node->primCount = count;
        node->area = 0.f;
        for (int i = start; i < end; i++) {
            node->area += prims[i]->getArea();
            orderedPrims.push_back(prims[i]);
        }
        return node;
    }

    // --- Centroid bounds → longest axis ---
    const int axis = centBounds.maxExtent();
    node->splitAxis = axis;

    int mid = (start + end) / 2;

    // Degenerate: all centroids coincide — can't split meaningfully.
    if (centBounds.pMax[axis] == centBounds.pMin[axis]) {
        // Force a leaf if small enough, otherwise just median-split.
        if (count <= maxPrimsInNode) {
            node->primOffset = (int) orderedPrims.size();
            node->primCount = count;
            node->area = 0.f;
            for (int i = start; i < end; i++) {
                node->area += prims[i]->getArea();
                orderedPrims.push_back(prims[i]);
            }
            return node;
        }
        // else fall through with mid already set to median
    } else if (splitMethod == SplitMethod::SAH && count > 4) {
        // -----------------------------------------------------------------
        // Binned SAH: try all 3 axes, pick the globally best split.
        //
        // For each axis we partition primitives into N_BUCKETS buckets
        // based on centroid position, then sweep to find the split that
        // minimises:  C = 1 + (nL * areaL + nR * areaR) / areaParent
        // -----------------------------------------------------------------
        constexpr int N_BUCKETS = 12;

        struct Bucket {
            int count = 0;
            Bounds3 bounds;
        };

        float bestCost = std::numeric_limits<float>::infinity();
        int bestAxis = axis;
        int bestSplit = N_BUCKETS / 2;
        const float parentArea = bounds.SurfaceArea();

        for (int a = 0; a < 3; a++) {
            float lo = centBounds.pMin[a];
            float hi = centBounds.pMax[a];
            if (lo == hi)
                continue;

            Bucket buckets[N_BUCKETS];
            const float span = hi - lo;

            // Use cached bounds and centroids — no virtual calls here.
            for (int i = 0; i < count; i++) {
                int b = std::clamp((int) (N_BUCKETS * (primCentroids[i][a] - lo) / span), 0,
                                   N_BUCKETS - 1);
                buckets[b].count++;
                buckets[b].bounds = Union(buckets[b].bounds, primBounds[i]);
            }

            // Prefix scan: left side cumulative area + count.
            float leftArea[N_BUCKETS - 1];
            int leftCount[N_BUCKETS - 1];
            {
                Bounds3 bL;
                int cL = 0;
                for (int i = 0; i < N_BUCKETS - 1; i++) {
                    bL = Union(bL, buckets[i].bounds);
                    cL += buckets[i].count;
                    leftArea[i] = bL.SurfaceArea();
                    leftCount[i] = cL;
                }
            }

            // Suffix scan: right side.
            float rightArea[N_BUCKETS - 1];
            int rightCount[N_BUCKETS - 1];
            {
                Bounds3 bR;
                int cR = 0;
                for (int i = N_BUCKETS - 2; i >= 0; i--) {
                    bR = Union(bR, buckets[i + 1].bounds);
                    cR += buckets[i + 1].count;
                    rightArea[i] = bR.SurfaceArea();
                    rightCount[i] = cR;
                }
            }

            for (int i = 0; i < N_BUCKETS - 1; i++) {
                if (leftCount[i] == 0 || rightCount[i] == 0)
                    continue;
                float cost =
                    1.f + (leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i]) / parentArea;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = a;
                    bestSplit = i;
                }
            }
        }

        // Partition in-place using cached centroids — no getBounds() calls.
        float lo = centBounds.pMin[bestAxis];
        float span = centBounds.pMax[bestAxis] - lo;
        float edge = lo + (bestSplit + 1) * span / N_BUCKETS;

        // Stable partition: split prims[] at edge on bestAxis.
        // We also keep primBounds/primCentroids in sync so the naive
        // nth_element fallback below still has valid cache data.
        std::vector<int> leftIdx, rightIdx;
        leftIdx.reserve(count);
        rightIdx.reserve(count);
        for (int i = 0; i < count; i++) {
            if (primCentroids[i][bestAxis] < edge)
                leftIdx.push_back(i);
            else
                rightIdx.push_back(i);
        }

        if (leftIdx.empty() || rightIdx.empty()) {
            mid = (start + end) / 2; // degenerate — fall back to median
        } else {
            // Write sorted order back into prims[].
            std::vector<Object *> tmp(count);
            int wi = 0;
            for (int idx : leftIdx)
                tmp[wi++] = prims[start + idx];
            for (int idx : rightIdx)
                tmp[wi++] = prims[start + idx];
            for (int i = 0; i < count; i++)
                prims[start + i] = tmp[i];
            mid = start + (int) leftIdx.size();
        }
    } else {
        // Naive: nth_element on longest axis — O(n), not O(n log n).
        // This branch only runs for count <= 4 so the getBounds() calls
        // here are negligible; don't bother with the cache.
        std::nth_element(prims.data() + start, prims.data() + mid, prims.data() + end,
                         [axis](Object *a, Object *b) {
                             return a->getBounds().Centroid()[axis] <
                                    b->getBounds().Centroid()[axis];
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

// ---------------------------------------------------------------------------
// Precompute per-ray data — called once per ray before traversal.
//
// octant[a] = 1 means ray goes in the negative direction on axis a,
// so bounds[a*2+1] (the max bound) is the near slab and bounds[a*2]
// (the min bound) is the far slab — which lets us avoid a branch inside
// the hot box test loop.
// ---------------------------------------------------------------------------
BVHAccel::RayData BVHAccel::precompute(const Ray &ray) const {
    RayData rd;
    for (int a = 0; a < 3; a++) {
        float d = ray.direction[a];
        rd.org[a] = ray.origin[a];
        rd.invDir[a] = (d != 0.f) ? (1.f / d) : std::numeric_limits<float>::infinity();
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
Intersection BVHAccel::Intersect(const Ray &ray) const {
    Intersection result;
    if (nodes.empty())
        return result;

    const RayData rd = precompute(ray);

    // Stack entries carry the node's box-entry distance recorded at push time.
    // When a node is popped the ray may have been shortened by a closer hit
    // found in the meantime, so we re-test tBox < closestT and skip the node
    // for free — no slab-test recomputation. Each node's box is tested exactly
    // once (as a child), never again at the top of the loop.
    struct StackEntry {
        uint32_t idx;
        float tBox;
    };
    StackEntry stack[64];
    int stackPtr = 0;

    uint32_t nodeIdx = 0;
    float closestT = std::numeric_limits<float>::infinity();

    // Deferred winner: we record only (triangle, barycentrics, t) during
    // traversal and build the full Intersection once, after the loop, for the
    // single closest hit. Avoids interpolating normals/UVs/tangents for every
    // triangle the ray merely passes near.
    const Triangle *hitTri = nullptr;
    float hitU = 0.f, hitV = 0.f;

    while (true) {
        const BVHNode &node = nodes[nodeIdx];

        if (node.isLeaf()) {
            if (leavesAreTriangles) {
                // Fast path: leaves are Triangles. Defer attribute work —
                // record only the winning (triangle, barycentrics, t) and
                // finalize once, after traversal.
                for (uint32_t i = 0; i < node.primitive_count(); i++) {
                    const Triangle *tri =
                        static_cast<const Triangle *>(orderedPrims[node.first_child + i]);
                    Triangle::TriHit h = tri->hitTest(ray);
                    if (h.happened && h.t < closestT) {
                        closestT = h.t;
                        hitTri = tri;
                        hitU = h.u;
                        hitV = h.v;
                    }
                }
            } else {
                // General path (e.g. scene BVH over MeshTriangle aggregates):
                // each primitive returns an already-finalized Intersection.
                for (uint32_t i = 0; i < node.primitive_count(); i++) {
                    Intersection tmp = orderedPrims[node.first_child + i]->getIntersection(ray);
                    if (tmp.happened && tmp.tnear < closestT) {
                        closestT = (float) tmp.tnear;
                        result = tmp;
                        hitTri = nullptr; // result already holds the winner
                    }
                }
            }
        } else {
            uint32_t left = nodeIdx + 1;
            uint32_t right = node.first_child;
            float tLeft = nodeIntersect(nodes[left], rd);
            float tRight = nodeIntersect(nodes[right], rd);

            // Descend into the nearer child whose box is still in range; push
            // the farther one with its tBox so a shortened ray can cull it on
            // pop. If a child's box is already beyond closestT, drop it now.
            if (tLeft <= tRight) {
                if (tLeft < closestT) {
                    if (tRight < closestT)
                        stack[stackPtr++] = {right, tRight};
                    nodeIdx = left;
                    continue;
                }
                // both children culled (tLeft<=tRight and tLeft>=closestT)
            } else {
                if (tRight < closestT) {
                    if (tLeft < closestT)
                        stack[stackPtr++] = {left, tLeft};
                    nodeIdx = right;
                    continue;
                }
                // both children culled
            }
        }

        // Pop the next in-range node. Skip any whose box is now beyond the
        // (possibly shortened) closest hit — no slab test needed.
        do {
            if (stackPtr == 0)
                goto done;
            nodeIdx = stack[--stackPtr].idx;
            if (stack[stackPtr].tBox < closestT)
                break;
        } while (true);
    }

done:
    if (hitTri)
        result = hitTri->finalize(ray, closestT, hitU, hitV);

    return result;
}

// ---------------------------------------------------------------------------
// IntersectP — shadow ray. Returns true on first hit, skips the rest.
// Identical traversal loop but exits immediately on any primitive hit.
// ---------------------------------------------------------------------------
bool BVHAccel::IntersectP(const Ray &ray, float tMaxDist) const {
    if (nodes.empty())
        return false;

    const RayData rd = precompute(ray);

    // Shadow rays exit on the *first* hit so we don't need tBox on the stack —
    // we can't prune with closestT anyway. But we do still want ordered traversal
    // so we find occluders near the origin first, maximising early-exit chance.
    uint32_t stack[64];
    int stackPtr = 0;
    uint32_t nodeIdx = 0;

    while (true) {
        const BVHNode &node = nodes[nodeIdx];

        float tBox = nodeIntersect(node, rd);
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

                // Near child first — find occluders sooner, exit earlier.
                // Only push the far child if its box is within range; a subtree
                // entirely beyond tMaxDist can never hold a valid occluder.
                uint32_t near, far;
                if (rd.octant[node.splitAxis()] == 0) {
                    near = left;
                    far = right;
                } else {
                    near = right;
                    far = left;
                }
                if (nodeIntersect(nodes[far], rd) < tMaxDist)
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

// ---------------------------------------------------------------------------
// Light sampling — unchanged logic from original, uses build tree.
// ---------------------------------------------------------------------------
void BVHAccel::getSample(BuildNode * /*node*/, float p, Intersection &pos, float &pdf) {
    uint32_t nodeIdx = 0;

    while (true) {
        const BVHNode &node = nodes[nodeIdx];

        if (node.isLeaf()) {
            for (uint32_t i = 0; i < node.primitive_count(); i++) {
                Object *obj = orderedPrims[node.first_child + i];
                p -= obj->getArea();
                if (p <= 0.f || i == node.primitive_count() - 1) {
                    // Sample() sets pdf = 1/area (uniform over the primitive).
                    // We leave it as-is; BVHAccel::Sample() divides by totalArea
                    // to get the final area pdf = 1/totalArea, which is correct
                    // for uniform sampling over all emissive primitives.
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
    float p = std::sqrt(get_random_float()) * totalArea;
    getSample(nullptr, p, pos, pdf); // root param ignored now
    pdf /= totalArea;
}