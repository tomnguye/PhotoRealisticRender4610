#ifndef RAYTRACING_CYLINDER_H
#define RAYTRACING_CYLINDER_H

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"

class Cylinder : public Object {
public:
    Vector3f center;
    float radius, height, radius2;
    Material *m;
    float area;

    Cylinder(const Vector3f &c, const float &r, const float &h, Material* mt = new Material()) 
        : center(c), radius(r), height(h), radius2(r * r), m(mt), area(2 * M_PI * r * h + 2 * M_PI * r * r) {}

    Intersection getIntersection(Ray ray) {
        Intersection result;
        result.happened = false;

        // Move ray origin to cylinder's local space
        Vector3f oc = ray.origin - center;

        // Intersect with infinite cylinder (aligned along Y-axis)
        float a = ray.direction.x * ray.direction.x + ray.direction.z * ray.direction.z;
        float b = 2 * (oc.x * ray.direction.x + oc.z * ray.direction.z);
        float c = oc.x * oc.x + oc.z * oc.z - radius2;

        float t0, t1;
        bool hit_side = solveQuadratic(a, b, c, t0, t1);

        float t = std::numeric_limits<float>::infinity();
        Vector3f normal;
        Vector3f hit_point;

        // Side intersection check
        if (hit_side) {
            if (t0 > 0) {
                float y0 = oc.y + t0 * ray.direction.y;
                if (y0 >= 0 && y0 <= height) {
                    t = t0;
                    hit_point = ray.origin + ray.direction * t;
                    Vector3f p = hit_point - center;
                    normal = normalize(Vector3f(p.x, 0, p.z));
                }
            }
            if (t1 > 0 && t1 < t) {
                float y1 = oc.y + t1 * ray.direction.y;
                if (y1 >= 0 && y1 <= height) {
                    t = t1;
                    hit_point = ray.origin + ray.direction * t;
                    Vector3f p = hit_point - center;
                    normal = normalize(Vector3f(p.x, 0, p.z));
                }
            }
        }

        // Cap intersection checks
        if (std::abs(ray.direction.y) > 1e-6) {
            // Bottom cap at y = 0
            float t_cap = -oc.y / ray.direction.y;
            Vector3f p_cap = oc + ray.direction * t_cap;
            if (t_cap > 0 && p_cap.x * p_cap.x + p_cap.z * p_cap.z <= radius2 && t_cap < t) {
                t = t_cap;
                hit_point = ray.origin + ray.direction * t;
                normal = Vector3f(0, -1, 0);
            }

            // Top cap at y = height
            t_cap = (height - oc.y) / ray.direction.y;
            p_cap = oc + ray.direction * t_cap;
            if (t_cap > 0 && p_cap.x * p_cap.x + p_cap.z * p_cap.z <= radius2 && t_cap < t) {
                t = t_cap;
                hit_point = ray.origin + ray.direction * t;
                normal = Vector3f(0, 1, 0);
            }
        }

        if (t < std::numeric_limits<float>::infinity()) {
            result.happened = true;
            result.coords = hit_point;
            result.normal = normal;
            result.material = m;
            result.obj = this;
            result.tnear = t;
        }

        return result;
    }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index,
                              const Vector2f &uv, Vector3f &N, Vector2f &st) const {
        Vector3f local = P - center;
        if (std::abs(local.y) < 1e-3) N = Vector3f(0, -1, 0); // bottom
        else if (std::abs(local.y - height) < 1e-3) N = Vector3f(0, 1, 0); // top
        else N = normalize(Vector3f(local.x, 0, local.z)); // side
    }

    Vector3f evalDiffuseColor(const Vector2f &st) const {
        return m->getColor();
    }

    Bounds3 getBounds() {
        return Bounds3(Vector3f(center.x - radius, center.y, center.z - radius),
                       Vector3f(center.x + radius, center.y + height, center.z + radius));
    }

    void Sample(Intersection &pos, float &pdf) {
        // This only samples the side surface for simplicity
        float theta = 2.0 * M_PI * get_random_float();
        float y = get_random_float() * height;
        Vector3f dir(std::cos(theta), 0, std::sin(theta));
        pos.coords = center + Vector3f(radius * dir.x, y, radius * dir.z);
        pos.normal = dir;
        pos.obj = this;
        pos.material = m;
        pdf = 1.0f / area;
    }

    float getArea() {
        return area;
    }

    bool hasEmit() {
        return m->hasEmission();
    }
};

#endif // RAYTRACING_CYLINDER_H
