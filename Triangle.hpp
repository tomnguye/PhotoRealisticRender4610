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

    Intersection getIntersection(Ray ray) override;
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
    float getArea() override { return area; }
    bool hasEmit() override { return m->hasEmission(); }
};

class MeshTriangle : public Object {
public:
    MeshTriangle(const std::string &filename, Vector3f offset, Material *mt = new Material());

    MeshTriangle(std::vector<Triangle> tris, Bounds3 bounds, Material *mt) {
        triangles = std::move(tris);
        bounding_box = bounds;
        m = mt;
        area = 0;
        for (auto &tri : triangles)
            area += tri.area;
        buildBVH();
    }

    Bounds3 getBounds() override { return bounding_box; }

    Intersection getIntersection(Ray ray) override {
        Intersection inter;
        if (bvh)
            inter = bvh->Intersect(ray);
        return inter;
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

    float getArea() override { return area; }
    bool hasEmit() override { return m->hasEmission(); }

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
        bvh = new BVHAccel(ptrs);
    }
};

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }
