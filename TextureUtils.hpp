#pragma once

#include "Vector.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

struct Texture {
    std::vector<unsigned char> data;
    int width = 0;
    int height = 0;
    int channels = 4; // gltf images are always RGBA via tinygltf

    bool empty() const { return data.empty(); }
};

/**
 UVs are wrapped (repeat mode). V is flipped since gltf uses a top-left origin and we use
 bottom-left. Bilinear filtering is used everywhere except sampleNormalMap, which normalises after
 interpolation.

 sRGB convention:
 Base colour and emissive textures are sRGB encoded per the gltf spec (section 5.19.6) and must be
 decoded to linear before use. Normal and metallic-roughness textures are already linear. */
namespace TextureUtils {

    /**
     * @brief Converts a UV coordinate to bilinear texel indices and fractional offsets.
     *
     * UVs are wrapped and V is flipped to match our bottom-left origin convention.
     *
     * @param u         U texture coordinate.
     * @param v         V texture coordinate.
     * @param width     Texture width in pixels.
     * @param height    Texture height in pixels.
     * @param x0        Left texel column (out).
     * @param y0        Top texel row (out).
     * @param x1        Right texel column, clamped to width - 1 (out).
     * @param y1        Bottom texel row, clamped to height - 1 (out).
     * @param fx        Horizontal fractional offset for interpolation (out).
     * @param fy        Vertical fractional offset for interpolation (out).
     */
    inline void uvToTexel(float u, float v, int width, int height, int &x0, int &y0, int &x1,
                          int &y1, float &fx, float &fy) {
        u = u - std::floor(u);
        v = v - std::floor(v);

        float px = u * (width - 1);
        float py = v * (height - 1);

        x0 = (int)px;
        y0 = (int)py;
        x1 = std::min(x0 + 1, width - 1);
        y1 = std::min(y0 + 1, height - 1);
        fx = px - x0;
        fy = py - y0;
    }

    /**
     * @brief Fetches raw RGB from an RGBA texture at integer texel coordinates.
     *
     * @param tex   The texture to sample.
     * @param x     Texel column.
     * @param y     Texel row.
     * @return Vector3f RGB value in [0, 1].
     */
    inline Vector3f fetchRGB(const Texture &tex, int x, int y) {
        int i = (y * tex.width + x) * 4;
        return Vector3f(tex.data[i] / 255.f, tex.data[i + 1] / 255.f, tex.data[i + 2] / 255.f);
    }

    /**
     * @brief Fetches a single channel from an RGBA texture at integer texel coordinates.
     *
     * @param tex       The texture to sample.
     * @param x         Texel column.
     * @param y         Texel row.
     * @param channel   Channel index (0 = R, 1 = G, 2 = B, 3 = A).
     * @return float Channel value in [0, 1].
     */
    inline float fetchChannel(const Texture &tex, int x, int y, int channel) {
        int i = (y * tex.width + x) * 4;
        return tex.data[i + channel] / 255.f;
    }

    /**
     * @brief Converts a single channel value from sRGB to linear (IEC 61966-2-1).
     *
     * @param x sRGB encoded value.
     * @return float Linear value.
     */
    inline float srgbToLinear(float x) {
        x = std::max(0.f, x);
        return x <= 0.04045f ? x / 12.92f : std::pow((x + 0.055f) / 1.055f, 2.4f);
    }

    /**
     * @brief Converts an RGB colour from sRGB to linear per-channel.
     *
     * @param c sRGB encoded colour.
     * @return Vector3f Linear colour.
     */
    inline Vector3f srgbToLinear(const Vector3f &c) {
        return Vector3f(srgbToLinear(c.x), srgbToLinear(c.y), srgbToLinear(c.z));
    }

    /**
     * @brief Bilinearly interpolates between four RGB values.
     *
     * @param c00 Top-left sample.
     * @param c10 Top-right sample.
     * @param c01 Bottom-left sample.
     * @param c11 Bottom-right sample.
     * @param fx  Horizontal fractional offset.
     * @param fy  Vertical fractional offset.
     * @return Vector3f Interpolated colour.
     */
    inline Vector3f bilerp(const Vector3f &c00, const Vector3f &c10, const Vector3f &c01,
                           const Vector3f &c11, float fx, float fy) {
        Vector3f top = c00 + (c10 - c00) * fx;
        Vector3f bottom = c01 + (c11 - c01) * fx;
        return top + (bottom - top) * fy;
    }

    /**
     * @brief Bilinearly interpolates between four 2D values.
     *
     * @param c00 Top-left sample.
     * @param c10 Top-right sample.
     * @param c01 Bottom-left sample.
     * @param c11 Bottom-right sample.
     * @param fx  Horizontal fractional offset.
     * @param fy  Vertical fractional offset.
     * @return Vector2f Interpolated value.
     */
    inline Vector2f bilerp2(const Vector2f &c00, const Vector2f &c10, const Vector2f &c01,
                            const Vector2f &c11, float fx, float fy) {
        auto lerp2 = [](Vector2f a, Vector2f b, float t) {
            return Vector2f(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        };
        return lerp2(lerp2(c00, c10, fx), lerp2(c01, c11, fx), fy);
    }

    /**
     * @brief Samples the base colour texture with bilinear filtering.
     *
     * Decodes from sRGB to linear per the gltf spec. Returns the fallback colour
     * if the texture is empty.
     *
     * @param tex       The base colour texture.
     * @param uv        Texture coordinates at the hit point.
     * @param fallback  Value returned when the texture is empty.
     * @return Vector3f Linear base colour.
     */
    inline Vector3f sampleBaseColor(const Texture &tex, const Vector2f &uv,
                                    const Vector3f &fallback) {
        if (tex.empty())
            return fallback;

        int x0, y0, x1, y1;
        float fx, fy;
        uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

        Vector3f c = bilerp(fetchRGB(tex, x0, y0), fetchRGB(tex, x1, y0), fetchRGB(tex, x0, y1),
                            fetchRGB(tex, x1, y1), fx, fy);
        return srgbToLinear(c);
    }

    /**
     * @brief Samples a tangent-space normal map with bilinear filtering.
     *
     * Normal maps are linear — no sRGB decode is applied. Returns (0, 0, 1)
     * if the texture is empty.
     *
     * @param tex   The normal map texture.
     * @param uv    Texture coordinates at the hit point.
     * @return Vector3f Normalised tangent-space normal.
     */
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

    /**
     * @brief Samples the metallic-roughness texture with bilinear filtering.
     *
     * Follows the gltf convention: G channel = roughness, B channel = metallic.
     * The texture is linear — no sRGB decode is applied. Returns the fallback
     * values if the texture is empty.
     *
     * @param tex               The metallic-roughness texture.
     * @param uv                Texture coordinates at the hit point.
     * @param fallbackRoughness Roughness returned when the texture is empty.
     * @param fallbackMetallic  Metallic returned when the texture is empty.
     * @return Vector2f (roughness, metallic).
     */
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

    /**
     * @brief Samples the emissive texture and modulates it by the material's emissive factor.
     *
     * Emissive textures are sRGB encoded per the gltf spec and are decoded to linear.
     * Returns the factor alone if the texture is empty.
     *
     * @param tex    The emissive texture.
     * @param uv     Texture coordinates at the hit point.
     * @param factor The material's emissive factor to modulate by.
     * @return Vector3f Linear emissive radiance.
     */
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

}