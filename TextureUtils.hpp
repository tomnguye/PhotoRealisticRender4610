#pragma once

#include "Texture.hpp"
#include "Vector.hpp"
#include <algorithm>
#include <cmath>

// ─── TextureUtils ─────────────────────────────────────────────────────────────
//
// Pure free functions for sampling Texture objects.
// No class dependency. Include this wherever you need to read texels.
//
// UV convention:
//   - UVs are wrapped (repeat mode)
//   - V is flipped: glTF uses top-left origin, we use bottom-left
//   - Bilinear filtering everywhere except sampleNormalMap which normalises
//
// sRGB convention:
//   - Base colour textures MUST be decoded from sRGB to linear before use
//     (glTF spec section 5.19.6)
//   - Normal, metallic-roughness, and emissive textures are already linear

namespace TextureUtils {

    // ─── Internal helpers ─────────────────────────────────────────────────────────

    // Wrap and flip UV to texel coordinates.
    inline void uvToTexel(float u, float v, int width, int height, int &x0, int &y0, int &x1, int &y1, float &fx, float &fy) {
        static int count = 0;
        if (count++ < 5) printf("UV in=(%.3f,%.3f) after wrap+flip=(%.3f,%.3f)\n", u, v, u - std::floor(u), 1.f - (v - std::floor(v)));
        u = u - std::floor(u); // wrap U
        v = v - std::floor(v); // wrap + flip V

        float px = u * (width - 1);
        float py = v * (height - 1);

        x0 = (int)px;
        y0 = (int)py;
        x1 = std::min(x0 + 1, width - 1);
        y1 = std::min(y0 + 1, height - 1);
        fx = px - x0;
        fy = py - y0;
    }

    // Fetch raw RGB from an RGBA texture at integer texel (x, y).
    inline Vector3f fetchRGB(const Texture &tex, int x, int y) {
        int i = (y * tex.width + x) * 4;
        return Vector3f(tex.data[i] / 255.f, tex.data[i + 1] / 255.f, tex.data[i + 2] / 255.f);
    }

    // Fetch single channel from an RGBA texture at integer texel (x, y).
    inline float fetchChannel(const Texture &tex, int x, int y, int channel) {
        int i = (y * tex.width + x) * 4;
        return tex.data[i + channel] / 255.f;
    }

    // sRGB -> linear for a single channel (IEC 61966-2-1)
    inline float srgbToLinear(float x) {
        x = std::max(0.f, x);
        return x <= 0.04045f ? x / 12.92f : std::pow((x + 0.055f) / 1.055f, 2.4f);
    }

    inline Vector3f srgbToLinear(const Vector3f &c) { return Vector3f(srgbToLinear(c.x), srgbToLinear(c.y), srgbToLinear(c.z)); }

    // Bilinear interpolation of two Vector3f values
    inline Vector3f bilerp(const Vector3f &c00, const Vector3f &c10, const Vector3f &c01, const Vector3f &c11, float fx, float fy) {
        Vector3f top = c00 + (c10 - c00) * fx;
        Vector3f bottom = c01 + (c11 - c01) * fx;
        return top + (bottom - top) * fy;
    }

    inline Vector2f bilerp2(const Vector2f &c00, const Vector2f &c10, const Vector2f &c01, const Vector2f &c11, float fx, float fy) {
        auto lerp2 = [](Vector2f a, Vector2f b, float t) { return Vector2f(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t); };
        return lerp2(lerp2(c00, c10, fx), lerp2(c01, c11, fx), fy);
    }

    // ─── Public sampling functions ────────────────────────────────────────────────

    // Sample base colour texture.
    // Returns fallback if texture is empty.
    // Decodes sRGB -> linear per the glTF spec.
    inline Vector3f sampleBaseColor(const Texture &tex, const Vector2f &uv, const Vector3f &fallback) {
        if (tex.empty()) return fallback;

        int x0, y0, x1, y1;
        float fx, fy;
        uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

        Vector3f c = bilerp(fetchRGB(tex, x0, y0), fetchRGB(tex, x1, y0), fetchRGB(tex, x0, y1), fetchRGB(tex, x1, y1), fx, fy);

        return srgbToLinear(c); // glTF base colour is sRGB encoded
    }

    // Sample normal map.
    // Returns tangent-space (0, 0, 1) if texture is empty.
    // Normal maps are linear — no sRGB decode.
    inline Vector3f sampleNormalMap(const Texture &tex, const Vector2f &uv) {
        if (tex.empty()) return Vector3f(0.f, 0.f, 1.f);

        int x0, y0, x1, y1;
        float fx, fy;
        uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

        // Decode [0,1] -> [-1,1]
        auto fetch = [&](int x, int y) -> Vector3f {
            int i = (y * tex.width + x) * 4;
            return Vector3f(tex.data[i] / 255.f * 2.f - 1.f, tex.data[i + 1] / 255.f * 2.f - 1.f, tex.data[i + 2] / 255.f * 2.f - 1.f);
        };

        Vector3f n = bilerp(fetch(x0, y0), fetch(x1, y0), fetch(x0, y1), fetch(x1, y1), fx, fy);
        return normalize(n);
    }

    // Sample metallic-roughness texture.
    // Returns (roughness, metallic) — G channel = roughness, B channel = metallic.
    // Returns (fallbackRoughness, fallbackMetallic) if texture is empty.
    // Linear — no sRGB decode.
    inline Vector2f sampleMetallicRoughness(const Texture &tex, const Vector2f &uv, float fallbackRoughness, float fallbackMetallic) {
        if (tex.empty()) return Vector2f(fallbackRoughness, fallbackMetallic);

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

    // Sample emissive texture, modulated by the material's emissive factor.
    // Returns factor alone if texture is empty.
    // Emissive textures are sRGB encoded per glTF spec.
    inline Vector3f sampleEmissive(const Texture &tex, const Vector2f &uv, const Vector3f &factor) {
        if (tex.empty()) return factor;

        int x0, y0, x1, y1;
        float fx, fy;
        uvToTexel(uv.x, uv.y, tex.width, tex.height, x0, y0, x1, y1, fx, fy);

        Vector3f c = bilerp(fetchRGB(tex, x0, y0), fetchRGB(tex, x1, y0), fetchRGB(tex, x0, y1), fetchRGB(tex, x1, y1), fx, fy);

        return factor * srgbToLinear(c);
    }

} // namespace TextureUtils