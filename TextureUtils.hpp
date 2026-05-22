#pragma once

#include "Vector.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

struct Texture {
    std::vector<unsigned char> data;
    int width = 0;
    int height = 0;
    int channels = 4;

    bool empty() const {
        return data.empty();
    }
};

namespace TextureUtils {

inline void uvToTexel(float u, float v, int width, int height, int &x0, int &y0, int &x1, int &y1,
                      float &fx, float &fy) {
    u = u - std::floor(u);
    v = v - std::floor(v);

    float px = u * (width - 1);
    float py = v * (height - 1);

    x0 = (int) px;
    y0 = (int) py;
    x1 = std::min(x0 + 1, width - 1);
    y1 = std::min(y0 + 1, height - 1);
    fx = px - x0;
    fy = py - y0;
}

inline Vector3f fetchRGB(const Texture &tex, int x, int y) {
    int i = (y * tex.width + x) * tex.channels; // use actual channels
    return Vector3f(tex.data[i] / 255.f, tex.data[i + 1] / 255.f, tex.data[i + 2] / 255.f);
}

inline float fetchChannel(const Texture &tex, int x, int y, int channel) {
    int i = (y * tex.width + x) * 4;
    return tex.data[i + channel] / 255.f;
}

inline float srgbToLinear(float x) {
    x = std::max(0.f, x);
    return x <= 0.04045f ? x / 12.92f : std::pow((x + 0.055f) / 1.055f, 2.4f);
}

inline Vector3f srgbToLinear(const Vector3f &c) {
    return Vector3f(srgbToLinear(c.x), srgbToLinear(c.y), srgbToLinear(c.z));
}

inline Vector3f bilerp(const Vector3f &c00, const Vector3f &c10, const Vector3f &c01,
                       const Vector3f &c11, float fx, float fy) {
    Vector3f top = c00 + (c10 - c00) * fx;
    Vector3f bottom = c01 + (c11 - c01) * fx;
    return top + (bottom - top) * fy;
}

inline Vector2f bilerp2(const Vector2f &c00, const Vector2f &c10, const Vector2f &c01,
                        const Vector2f &c11, float fx, float fy) {
    auto lerp2 = [](Vector2f a, Vector2f b, float t) {
        return Vector2f(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    };
    return lerp2(lerp2(c00, c10, fx), lerp2(c01, c11, fx), fy);
}

inline Vector3f sampleBaseColor(const Texture &tex, const Vector2f &uv, const Vector3f &fallback) {
    if (tex.empty())
        return fallback;

    int x0, y0, x1, y1;
    float fx, fy;
    uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

    Vector3f c = bilerp(fetchRGB(tex, x0, y0), fetchRGB(tex, x1, y0), fetchRGB(tex, x0, y1),
                        fetchRGB(tex, x1, y1), fx, fy);
    return srgbToLinear(c);
}

inline Vector3f sampleNormalMap(const Texture &tex, const Vector2f &uv) {
    if (tex.empty())
        return Vector3f(0.f, 0.f, 1.f);

    int x0, y0, x1, y1;
    float fx, fy;
    uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

    auto fetch = [&](int x, int y) -> Vector3f {
        int i = (y * tex.width + x) * 4;
        return Vector3f(tex.data[i] / 255.f * 2.f - 1.f, tex.data[i + 1] / 255.f * 2.f - 1.f,
                        tex.data[i + 2] / 255.f * 2.f - 1.f);
    };

    Vector3f n = bilerp(fetch(x0, y0), fetch(x1, y0), fetch(x0, y1), fetch(x1, y1), fx, fy);
    return normalize(n);
}

inline Vector2f sampleMetallicRoughness(const Texture &tex, const Vector2f &uv,
                                        float fallbackRoughness, float fallbackMetallic) {
    if (tex.empty())
        return Vector2f(fallbackRoughness, fallbackMetallic);

    int x0, y0, x1, y1;
    float fx, fy;
    uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

    auto fetch = [&](int x, int y) -> Vector2f {
        int i = (y * tex.width + x) * 4;
        return Vector2f(tex.data[i + 1] / 255.f,  // G = roughness
                        tex.data[i + 2] / 255.f); // B = metallic
    };

    return bilerp2(fetch(x0, y0), fetch(x1, y0), fetch(x0, y1), fetch(x1, y1), fx, fy);
}

inline Vector3f sampleEmissive(const Texture &tex, const Vector2f &uv, const Vector3f &factor) {
    if (tex.empty())
        return factor;

    int x0, y0, x1, y1;
    float fx, fy;
    uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

    Vector3f c = bilerp(fetchRGB(tex, x0, y0), fetchRGB(tex, x1, y0), fetchRGB(tex, x0, y1),
                        fetchRGB(tex, x1, y1), fx, fy);
    return factor * srgbToLinear(c);
}

} // namespace TextureUtils