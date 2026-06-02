#pragma once

#include "Vector.hpp"
#include <algorithm>
#include <cassert>
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

struct TexelQuad {
    int x0, y0, x1, y1;
    float fx, fy;
};

inline TexelQuad uvToTexelQuad(float u, float v, int width, int height) {
    u -= std::floor(u);
    v -= std::floor(v);

    float pixelX = u * width;
    float pixelY = v * height;

    TexelQuad quad;
    quad.x0 = (int) pixelX % width;
    quad.y0 = (int) pixelY % height;
    quad.x1 = (quad.x0 + 1) % width;
    quad.y1 = (quad.y0 + 1) % height;
    quad.fx = pixelX - std::floor(pixelX);
    quad.fy = pixelY - std::floor(pixelY);
    return quad;
}

inline Vector3f bilinear(const Vector3f &topLeft, const Vector3f &topRight,
                         const Vector3f &bottomLeft, const Vector3f &bottomRight, float fx,
                         float fy) {
    Vector3f top = topLeft + (topRight - topLeft) * fx;
    Vector3f bottom = bottomLeft + (bottomRight - bottomLeft) * fx;
    return top + (bottom - top) * fy;
}

inline Vector2f bilinear(const Vector2f &topLeft, const Vector2f &topRight,
                         const Vector2f &bottomLeft, const Vector2f &bottomRight, float fx,
                         float fy) {
    Vector2f top(topLeft.x + (topRight.x - topLeft.x) * fx,
                 topLeft.y + (topRight.y - topLeft.y) * fx);
    Vector2f bottom(bottomLeft.x + (bottomRight.x - bottomLeft.x) * fx,
                    bottomLeft.y + (bottomRight.y - bottomLeft.y) * fx);
    return Vector2f(top.x + (bottom.x - top.x) * fy, top.y + (bottom.y - top.y) * fy);
}

inline float srgbToLinear(float value) {
    value = std::max(0.f, value);
    return value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
}

inline Vector3f srgbToLinear(const Vector3f &color) {
    return Vector3f(srgbToLinear(color.x), srgbToLinear(color.y), srgbToLinear(color.z));
}

inline Vector3f fetchRGB(const Texture &texture, int x, int y) {
    assert(texture.channels >= 3);
    int i = (y * texture.width + x) * texture.channels;
    return Vector3f(texture.data[i] / 255.f, texture.data[i + 1] / 255.f,
                    texture.data[i + 2] / 255.f);
}

inline Vector3f fetchNormal(const Texture &texture, int x, int y) {
    int i = (y * texture.width + x) * texture.channels;
    return Vector3f(texture.data[i] / 255.f * 2.f - 1.f, texture.data[i + 1] / 255.f * 2.f - 1.f,
                    texture.data[i + 2] / 255.f * 2.f - 1.f);
}

inline Vector2f fetchMetallicRoughness(const Texture &texture, int x, int y) {
    int i = (y * texture.width + x) * texture.channels;
    return Vector2f(texture.data[i + 1] / 255.f,  // green = roughness
                    texture.data[i + 2] / 255.f); // blue = metallic
}

inline Vector3f sampleBaseColor(const Texture &texture, const Vector2f &uv,
                                const Vector3f &fallback) {
    if (texture.empty())
        return fallback;

    TexelQuad quad = uvToTexelQuad(uv.x, uv.y, texture.width, texture.height);
    Vector3f color = bilinear(
        fetchRGB(texture, quad.x0, quad.y0), fetchRGB(texture, quad.x1, quad.y0),
        fetchRGB(texture, quad.x0, quad.y1), fetchRGB(texture, quad.x1, quad.y1), quad.fx, quad.fy);
    return srgbToLinear(color);
}

inline Vector3f sampleNormalMap(const Texture &texture, const Vector2f &uv) {
    if (texture.empty())
        return Vector3f(0.f, 0.f, 1.f);

    TexelQuad quad = uvToTexelQuad(uv.x, uv.y, texture.width, texture.height);
    Vector3f normal =
        bilinear(fetchNormal(texture, quad.x0, quad.y0), fetchNormal(texture, quad.x1, quad.y0),
                 fetchNormal(texture, quad.x0, quad.y1), fetchNormal(texture, quad.x1, quad.y1),
                 quad.fx, quad.fy);
    return normalize(normal);
}

inline Vector2f sampleMetallicRoughness(const Texture &texture, const Vector2f &uv,
                                        float fallbackRoughness, float fallbackMetallic) {
    if (texture.empty())
        return Vector2f(fallbackRoughness, fallbackMetallic);

    assert(texture.channels >= 3);

    TexelQuad quad = uvToTexelQuad(uv.x, uv.y, texture.width, texture.height);
    return bilinear(fetchMetallicRoughness(texture, quad.x0, quad.y0),
                    fetchMetallicRoughness(texture, quad.x1, quad.y0),
                    fetchMetallicRoughness(texture, quad.x0, quad.y1),
                    fetchMetallicRoughness(texture, quad.x1, quad.y1), quad.fx, quad.fy);
}

inline Vector3f sampleEmissive(const Texture &texture, const Vector2f &uv, const Vector3f &factor) {
    if (texture.empty())
        return factor;

    TexelQuad quad = uvToTexelQuad(uv.x, uv.y, texture.width, texture.height);
    Vector3f color = bilinear(
        fetchRGB(texture, quad.x0, quad.y0), fetchRGB(texture, quad.x1, quad.y0),
        fetchRGB(texture, quad.x0, quad.y1), fetchRGB(texture, quad.x1, quad.y1), quad.fx, quad.fy);
    return factor * srgbToLinear(color);
}

} // namespace TextureUtils