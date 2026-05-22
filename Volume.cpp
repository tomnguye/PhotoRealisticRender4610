#include "Volume.hpp"

float phaseHG(float cos_theta, float g) {
    float denom = 1.0f + g * g - 2.0f * g * cos_theta;
    return (1.0f - g * g) / (4.0f * M_PI * std::pow(denom, 1.5f));
}

Vector3f samplePhaseHG(const Vector3f &wo, float g) {
    float cos_theta;
    if (std::abs(g) < 1e-3f) {
        cos_theta = 1.0f - 2.0f * get_random_float();
    } else {
        float sqr = (1.0f - g * g) / (1.0f - g + 2.0f * g * get_random_float());
        cos_theta = (1.0f + g * g - sqr * sqr) / (2.0f * g);
    }

    float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));
    float phi = 2.0f * M_PI * get_random_float();

    Vector3f T, B;
    buildTBN(wo, T, B);

    return normalize(sin_theta * std::cos(phi) * T + sin_theta * std::sin(phi) * B +
                     cos_theta * wo);
}