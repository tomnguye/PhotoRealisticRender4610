#pragma once
#include "BRDFUtils.hpp"
#include "Bounds3.hpp"
#include "Vector.hpp"
#include "global.hpp"

struct Medium {
    float sigma_a = 0.0f;
    float sigma_s = 0.0f;
    float sigma_t = 0.0f;
    float g = 0.0f;
    bool bounded = false;
    Bounds3 bounds;

    bool active() const { return sigma_t > 1e-6f; }

    bool intersect(const Ray &ray, float &t_enter, float &t_exit) const {
        if (!bounded) {
            t_enter = 0.0f;
            t_exit = std::numeric_limits<float>::infinity();
            return true;
        }

        Vector3f invDir(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);
        std::array<int, 3> dirIsNeg = {int(ray.direction.x > 0), int(ray.direction.y > 0),
                                       int(ray.direction.z > 0)};

        // recompute t_enter and t_exit manually since IntersectP doesn't return them
        float tEnter = -std::numeric_limits<float>::max();
        float tExit = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; i++) {
            float t_min = (bounds.pMin[i] - ray.origin[i]) * invDir[i];
            float t_max = (bounds.pMax[i] - ray.origin[i]) * invDir[i];
            if (dirIsNeg[i] == 0)
                std::swap(t_min, t_max);
            tEnter = std::max(t_min, tEnter);
            tExit = std::min(t_max, tExit);
        }

        if (tEnter > tExit || tExit < 0.0f)
            return false;

        t_enter = tEnter;
        t_exit = tExit;
        return true;
    }

    static Medium dust() {
        Medium m;
        m.sigma_s = 0.05f;
        m.sigma_a = 0.005f;
        m.sigma_t = m.sigma_s + m.sigma_a;
        m.g = 0.6f;
        return m;
    }

    static Medium fog() {
        Medium m;
        m.sigma_s = 0.1f;
        m.sigma_a = 0.01f;
        m.sigma_t = m.sigma_s + m.sigma_a;
        m.g = 0.0f;
        return m;
    }
};

float phaseHG(float cos_theta, float g);
Vector3f samplePhaseHG(const Vector3f &wo, float g);