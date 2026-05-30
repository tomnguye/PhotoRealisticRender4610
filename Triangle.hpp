#pragma once

#include "BVH.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include <array>
#include <cassert>
#include <cstring>
#include <vector>

class Triangle : public Object {
  public:
    Vector3f v0, v1, v2;       // vertices A, B, C counter-clockwise order
    Vector3f e1, e2;           // edges v1-v0, v2-v0
    Vector2f t0, t1, t2;       // per-vertex texture coords
    Vector3f tan0, tan1, tan2; // per-vertex tangents (xyz only)
    float tangentW0 = 1.f, tangentW1 = 1.f,
          tangentW2 = 1.f; // glTF tangent handedness (w component)
    bool hasTangents = false;
    Vector3f n0, n1, n2; // per-vertex smooth normals
    bool hasSmoothNormals = false;
    Vector3f normal;
    float area;
    Material *m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        area = crossProduct(e1, e2).norm() * 0.5f;
    }

    // Cheap closest-hit query for the hot BVH loop: returns barycentrics + t
    // with NO attribute interpolation and NO Intersection construction. The
    // winning hit is turned into a full Intersection exactly once, after
    // traversal, via finalize(). happened==false means miss.
    struct TriHit {
        float t;
        float u, v;
        bool happened;
    };

    inline TriHit hitTest(const Ray &ray) const {
        Vector3f P = crossProduct(ray.direction, e2);
        float PdotE1 = dotProduct(P, e1);
        if (fabs(PdotE1) < 1e-6f)
            return {0.f, 0.f, 0.f, false};
        float inv = 1.f / PdotE1;
        Vector3f T = ray.origin - v0;
        float u = dotProduct(P, T) * inv;
        if (u < 0.f)
            return {0.f, 0.f, 0.f, false};
        Vector3f Q = crossProduct(T, e1);
        float v = dotProduct(Q, ray.direction) * inv;
        if (v < 0.f || u + v > 1.f)
            return {0.f, 0.f, 0.f, false};
        float t = dotProduct(Q, e2) * inv;
        if (t <= 1e-3f)
            return {0.f, 0.f, 0.f, false};
        return {t, u, v, true};
    }

    // Build the full Intersection for a hit produced by hitTest(). Called once
    // per ray, on the closest triangle only.
    Intersection finalize(const Ray &ray, float t, float u, float v) const;

    Intersection getIntersection(Ray ray) override;

    // Shadow-ray fast path: same Möller–Trumbore as getIntersection but
    // returns only tnear. No normal/tangent/UV work at all. A hit beyond
    // tMax is reported as a miss so callers can early-exit.
    float intersectT(const Ray &ray,
                     float tMax = std::numeric_limits<float>::infinity()) const override {
        Vector3f P = crossProduct(ray.direction, e2);
        float PdotE1 = dotProduct(P, e1);
        if (fabs(PdotE1) < 1e-6f)
            return -1.f;
        float inv = 1.f / PdotE1;
        Vector3f T = ray.origin - v0;
        float u = dotProduct(P, T) * inv;
        if (u < 0.f || u > 1.f)
            return -1.f;
        Vector3f Q = crossProduct(T, e1);
        float v = dotProduct(Q, ray.direction) * inv;
        if (v < 0.f || u + v > 1.f)
            return -1.f;
        float t = dotProduct(Q, e2) * inv;
        return (t > 1e-3f && t < tMax) ? t : -1.f;
    }
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index,
                              const Vector2f &uv, Vector3f &N, Vector2f &st) const override {
        N = normal;
    }
    Bounds3 getBounds() override;
    void Sample(Intersection &pos, float &pdf) override {
        float x = std::sqrt(get_random_float()), y = get_random_float();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = this->normal;
        pdf = 1.0f / area;
    }
    float getArea() override {
        return area;
    }
    bool hasEmit() override {
        return m->hasEmission();
    }
};

class MeshTriangle : public Object {
  public:
    MeshTriangle(const std::string &filename, Vector3f offset, Material *mt);

    MeshTriangle(std::vector<Triangle> tris, Bounds3 bounds, Material *mt) {
        triangles = std::move(tris);
        bounding_box = bounds;
        m = mt;
        area = 0;
        for (auto &tri : triangles)
            area += tri.area;
        buildBVH();
    }

    Bounds3 getBounds() override {
        return bounding_box;
    }

    Intersection getIntersection(Ray ray) override {
        Intersection inter;
        if (bvh)
            inter = bvh->Intersect(ray);
        return inter;
    }

    // Shadow fast path: use the sub-BVH's IntersectP, which exits on the first
    // occluder within tMax and never builds a full Intersection record. Returns
    // a sentinel positive distance on hit (the caller only checks the sign /
    // the < tMax bound, which IntersectP has already enforced).
    float intersectT(const Ray &ray,
                     float tMax = std::numeric_limits<float>::infinity()) const override {
        if (!bvh)
            return -1.f;
        return bvh->IntersectP(ray, tMax) ? 1e-3f : -1.f;
    }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index,
                              const Vector2f &uv, Vector3f &N, Vector2f &st) const override {
        N = triangles[index].normal;
    }

    void Sample(Intersection &pos, float &pdf) override {
        bvh->Sample(pos, pdf);
        pos.obj = this;
        pos.material = this->m;
    }

    float getArea() override {
        return area;
    }
    bool hasEmit() override {
        return m->hasEmission();
    }

    Bounds3 bounding_box;
    std::vector<Triangle> triangles;
    BVHAccel *bvh = nullptr;
    float area = 0.f;
    Material *m = nullptr;

  private:
    void buildBVH() {
        std::vector<Object *> ptrs;
        ptrs.reserve(triangles.size());
        for (auto &tri : triangles)
            ptrs.push_back(&tri);
        bvh = new BVHAccel(ptrs, 4);
    }
};

inline Bounds3 Triangle::getBounds() {
    return Union(Bounds3(v0, v1), v2);
}