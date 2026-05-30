//
// Created by goksu on 2/25/20.
//
#pragma once

#include "Integrator.hpp"
#include "Scene.hpp"
#include "ToneMapping.hpp"

/**
 * @brief Welford online variance tracker.
 * This can be used to track the variance of each pixels' value based on SPP.
 */
struct Welford {
    int n = 0;
    float mean = 0.f;
    float M2 = 0.f;

    void update(float x) {
        n++;
        float delta = x - mean;
        mean += delta / (float) n;
        M2 += delta * (x - mean);
    }

    float variance() const {
        return n < 2 ? 0.f : M2 / (float) (n - 1);
    }

    /**
     * @brief Calculates 95% confidence interval that true mean is within estimated mean.
     * Can be used to determine whether a pixel's samples have converged with 95% confidence.
     * @return 95% confidence interval as a fraction of the  mean.
     */
    float ci() const {
        if (n < 2 || std::abs(mean) < 1e-6f)
            return 1.f;
        return 1.96f * std::sqrt(variance() / (float) n) / std::abs(mean);
    }
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

    // Tone mapping operator applied when forming the 8-bit output image.
    // See ToneMapping.hpp for the available operators.
    tonemap::ToneMapper toneMapper = tonemap::ToneMapper::AgX;
};

class Renderer {
  public:
    void Render(const Scene &scene, const Integrator &integrator, const RenderSettings &settings);
};