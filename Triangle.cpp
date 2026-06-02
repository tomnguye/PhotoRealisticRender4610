#include "Triangle.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <string>

Intersection Triangle::finalize(const Ray &ray, float t, float u, float v) const {
    Intersection inter;
    inter.happened = true;
    inter.coords = ray.origin + t * ray.direction;
    inter.tnear = t;
    inter.obj = const_cast<Triangle *>(this);
    inter.material = m;

    float w0 = 1.f - u - v;
    inter.tcoords = t0 * w0 + t1 * u + t2 * v;

    if (hasSmoothNormals)
        inter.normal = normalize(n0 * w0 + n1 * u + n2 * v);
    else
        inter.normal = this->normal;

    if (hasTangents) {
        Vector3f interpTan = normalize(tan0 * w0 + tan1 * u + tan2 * v);
        float w = tangentW0 * w0 + tangentW1 * u + tangentW2 * v;
        inter.tangent = interpTan;
        inter.tangentHandedness = w >= 0.f ? 1.f : -1.f;
        inter.hasTangent = true;
    }

    return inter;
}

Intersection Triangle::getIntersection(Ray ray) {
    TriangleHit h = hitTest(ray);
    if (!h.happened)
        return Intersection();
    return finalize(ray, h.t, h.u, h.v);
}