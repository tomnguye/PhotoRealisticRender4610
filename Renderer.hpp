//
// Created by goksu on 2/25/20.
//
#include "Integrator.hpp"
#include "Scene.hpp"

#pragma once
struct hit_payload {
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object *hit_obj;
};

// ─── Welford online variance tracker ─────────────────────────────────────────

struct Welford {
    int n = 0;
    float mean = 0.f;
    float M2 = 0.f;

    void update(float x) {
        n++;
        float delta = x - mean;
        mean += delta / (float)n;
        M2 += delta * (x - mean);
    }

    float variance() const { return n < 2 ? 0.f : M2 / (float)(n - 1); }

    // 95% confidence interval as fraction of mean — stop when below threshold
    float ci() const {
        if (n < 2 || std::abs(mean) < 1e-6f)
            return 1.f;
        return 1.96f * std::sqrt(variance() / (float)n) / std::abs(mean);
    }
};

// ─── Tile ─────────────────────────────────────────────────────────────────────

struct Tile {
    int x, y; // top-left pixel
    int w, h; // tile dimensions (may be smaller at edges)
};

struct RenderSettings {
    int width = 1280;
    int height = 1720;
    int minSPP = 64;
    int maxSPP = 64;
    float russianRoulette = 0.95f;
    int maxDepth = 30;
    float varianceThreshold = 0.05f;
    float exposure = 0.18f;
};

class Renderer {
public:
    void Render(const Scene &scene, const Integrator &integrator, const RenderSettings &settings);

private:
};
