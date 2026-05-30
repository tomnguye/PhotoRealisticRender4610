//
// Created by LEI XU on 5/13/19.
//
#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include "global.hpp"

class Object {
  public:
    Object() {}
    virtual ~Object() {}
    virtual Intersection getIntersection(Ray _ray) = 0;
    // Lightweight shadow test — returns a hit distance in (0, tMax) if the ray
    // is occluded within tMax, or -1.f otherwise. Overrides in leaf primitives
    // avoid building a full Intersection; the tMax bound lets aggregate objects
    // (e.g. MeshTriangle) early-exit on the first occluder inside range instead
    // of running a full closest-hit search.
    virtual float intersectT(const Ray &ray,
                             float tMax = std::numeric_limits<float>::infinity()) const {
        // Safe default: fall back to full intersection, then honour the bound.
        Intersection i = const_cast<Object *>(this)->getIntersection(ray);
        return (i.happened && i.tnear < tMax) ? (float) i.tnear : -1.f;
    }
    virtual void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &,
                                      const Vector2f &, Vector3f &, Vector2f &) const = 0;
    virtual Bounds3 getBounds() = 0;
    virtual float getArea() = 0;
    virtual void Sample(Intersection &pos, float &pdf) = 0;
    virtual bool hasEmit() = 0;
};

#endif // RAYTRACING_OBJECT_H