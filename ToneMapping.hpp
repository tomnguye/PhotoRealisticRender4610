#pragma once

// =============================================================================
//  ToneMapping.hpp
//
//  Display transforms ("tone mappers") for converting scene-referred linear
//  radiance into display-referred, sRGB-encoded values ready for an 8-bit
//  buffer.
//
//  DESIGN
//  ------
//  Each operator takes a linear, scene-referred Vector3f (channels may exceed
//  1.0) and returns a display-encoded Vector3f in [0, 1]. The returned values
//  are written DIRECTLY to the 8-bit framebuffer -- callers must NOT apply any
//  further gamma / sRGB encode afterwards. Each operator owns its complete
//  encode chain.
//
//  Adding a new operator: add an enum entry, write a static function, add one
//  case to apply(). Nothing else changes.
// =============================================================================

#include "Vector.hpp" // Vector3f
#include <algorithm>
#include <cmath>
#include <string>

namespace tonemap {

// -----------------------------------------------------------------------------
//  Operator selection
// -----------------------------------------------------------------------------
enum class ToneMapper {
    Raw,        // No transform at all. Scene-linear written straight out (clamped).
    Standard,   // sRGB OETF only, no tone curve (Blender's "Standard" view).
    AgX,        // AgX display transform (Godot / EaryChow config) + sRGB OETF.
    PBRNeutral, // Khronos PBR Neutral tone mapper + sRGB OETF.
};

// -----------------------------------------------------------------------------
//  Shared helpers
// -----------------------------------------------------------------------------

// Piecewise sRGB OETF (linear -> sRGB signal). Input clamped to [0, 1].
// Ported from Godot's linear_to_srgb().
inline float srgbOETF(float x) {
    x = std::clamp(x, 0.f, 1.f);
    return x < 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1.f / 2.4f) - 0.055f;
}

inline Vector3f srgbOETF(Vector3f c) {
    return Vector3f(srgbOETF(c.x), srgbOETF(c.y), srgbOETF(c.z));
}

// 3x3 matrix * column vector (result = M * v).
inline Vector3f mat3Mul(const float m[3][3], Vector3f v) {
    return Vector3f(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
}

inline Vector3f clamp01(Vector3f c) {
    return Vector3f(std::clamp(c.x, 0.f, 1.f), std::clamp(c.y, 0.f, 1.f),
                    std::clamp(c.z, 0.f, 1.f));
}

// -----------------------------------------------------------------------------
//  Raw: no transform whatsoever.
//
//  Scene-linear radiance written straight to the buffer (only clamped to the
//  representable [0,1] range). This matches Blender's "Raw" view transform: it
//  applies NO display encode, so it looks dark on an sRGB monitor. It is a
//  diagnostic / data passthrough, not a viewable image.
// -----------------------------------------------------------------------------
inline Vector3f raw(Vector3f linear) {
    return clamp01(linear);
}

// -----------------------------------------------------------------------------
//  Standard: sRGB display encode, no tone curve.
//
//  Linear -> sRGB OETF, with highlights above 1.0 hard-clipping. This is
//  Blender's "Standard" view transform and the honest baseline for isolating
//  what a *tone curve* (like AgX) adds: the encode is the same, only the curve
//  differs.
// -----------------------------------------------------------------------------
inline Vector3f standard(Vector3f linear) {
    return srgbOETF(linear); // srgbOETF clamps internally
}

// -----------------------------------------------------------------------------
//  AgX  (Godot's renderer implementation; EaryChow AgX config)
//
//  Faithful CPU port of Godot's tonemap_agx() combined with Allen Pestaluky's
//  "allenwp" tonemapping curve, which Godot uses to match Blender's AgX curve.
//
//  Sources (verbatim ports, not reconstructions):
//    - tonemap_agx() and the combined Rec.709<->Rec.2020 + AgX inset/outset
//      matrices: Godot's servers/rendering/.../tonemap.glsl
//    - allenwp_curve() (GPU) + parameter derivation (CPU), MIT licensed:
//      https://allenwp.com/blog/2025/05/29/allenwp-tonemapping-curve/
//
//  The curve is a piecewise function: a sigmoid power "toe" for dark-to-mid
//  values and a Reinhard-like "shoulder" for highlights. The middle anchor is
//  fixed at 18% middle grey (0.1841865), so 18% in maps to 18% out -- the AgX
//  design intent. Parameters that are constant per-frame are precomputed once
//  (the "CPU code") and the per-channel curve ("GPU code") uses them.
//
//  Input:  linear sRGB / Rec.709 primaries, scene-referred (non-negative).
//  Output: sRGB-encoded, [0, 1], ready for the 8-bit buffer.
// -----------------------------------------------------------------------------

// Precomputed allenwp-curve parameters (the per-frame "CPU code"). Constructed
// once; depends only on contrast / high-clip / output range, not on pixels.
struct AllenwpParams {
    // Must match the shader-side constant. 18% middle grey.
    static constexpr float crossover = 0.1841865f;

    float contrast;     // toe strength / "gamma"; >= 1.0
    float toe_a;        // solved so toe meets shoulder at the crossover
    float slope;        // derivative of the toe at the crossover
    float w;            // shoulder width term
    float shoulder_max; // output_max - crossover
    float output_max;   // 1.0 for SDR

    explicit AllenwpParams(float contrast_ = 1.25f, float highClip = 16.0f,
                           float outputMax = 1.0f) {
        contrast = contrast_;
        output_max = outputMax;

        // Keep the shoulder well-behaved across output ranges.
        float high_clip = std::max(highClip, outputMax);

        // toe_a: Mathematica solution ensuring the toe intersects the shoulder
        // exactly at the crossover point.
        toe_a = ((1.f / crossover) - 1.f) * std::pow(crossover, contrast);

        // slope: derivative of the toe function evaluated at the crossover.
        float denom = std::pow(crossover, contrast) + toe_a;
        slope = (contrast * std::pow(crossover, contrast - 1.f) * toe_a) / (denom * denom);

        shoulder_max = output_max - crossover;

        w = high_clip - crossover;
        w = w * w;
        w = w / shoulder_max;
        w = w * slope;
    }
};

// Per-channel allenwp curve (the "GPU code"), ported verbatim.
inline float allenwpCurve(float x, const AllenwpParams &p) {
    x = std::max(x, 0.f); // negative input -> undefined pow()

    if (x < AllenwpParams::crossover) {
        // Sigmoid power toe.
        float t = std::pow(x, p.contrast);
        return t / (t + p.toe_a);
    } else {
        // Reinhard-like shoulder.
        float s = x - AllenwpParams::crossover;
        float slope_s = p.slope * s;
        s = slope_s * (1.f + s / p.w) / (1.f + (slope_s / p.shoulder_max));
        s += AllenwpParams::crossover;
        return s;
    }
}

inline Vector3f allenwpCurve(Vector3f c, const AllenwpParams &p) {
    return Vector3f(allenwpCurve(c.x, p), allenwpCurve(c.y, p), allenwpCurve(c.z, p));
}

inline Vector3f agx(Vector3f color) {
    // Combined Rec.709 -> Rec.2020 + Blender AgX inset matrix (Godot).
    // NOTE: Godot lists these in column-major mat3 order; transposed here into
    // the row-major [row][col] layout mat3Mul expects, so result = M * v.
    static const float inset[3][3] = {
        {0.544814746488245f, 0.373787398372697f, 0.0813978551390581f},
        {0.140416948464053f, 0.754137554567394f, 0.105445496968552f},
        {0.0888104196149096f, 0.178871756420858f, 0.732317823964232f}};

    // Combined inverse AgX outset + Rec.2020 -> Rec.709 matrix (Godot).
    static const float outset[3][3] = {
        {1.96488741169489f, -0.855988495690215f, -0.108898916004672f},
        {-0.299313364904742f, 1.32639796461980f, -0.0270845997150571f},
        {-0.164352742528393f, -0.238183969428088f, 1.40253671195648f}};

    const float output_max_value = 1.0f; // SDR
    static const AllenwpParams params(1.25f /*contrast*/, 16.0f /*high clip*/, output_max_value);

    // Input must be non-negative (see Godot's note on the inset matrix).
    color.x = std::max(color.x, 0.f);
    color.y = std::max(color.y, 0.f);
    color.z = std::max(color.z, 0.f);

    // Inset (Rec.709 -> Rec.2020 + AgX inset).
    color = mat3Mul(inset, color);

    // AgX picture-formation curve (allenwp, matched to Blender's AgX).
    color = allenwpCurve(color, params);

    // Clip to the max output value (addresses a cyan tint on very bright input).
    color.x = std::min(output_max_value, color.x);
    color.y = std::min(output_max_value, color.y);
    color.z = std::min(output_max_value, color.z);

    // Outset (AgX outset + Rec.2020 -> Rec.709).
    color = mat3Mul(outset, color);

    // The outset can produce small negative components; clamp before encode.
    color = clamp01(color);

    // Display encode: sRGB OETF (Godot's linear_to_srgb).
    return srgbOETF(color);
}

// -----------------------------------------------------------------------------
//  Khronos PBR Neutral
//
//  Faithful CPU port of the reference implementation by Emmett Lalish (Khronos
//  3D Commerce working group). Designed for PBR e-commerce: it reproduces
//  baseColor hue and saturation faithfully (1:1 up to a threshold), then
//  compresses highlights and desaturates them toward white -- without the
//  gamut/hue shifts of AgX or ACES.
//
//  Source (verbatim port of the GLSL): https://modelviewer.dev/examples/tone-mapping
//
//  Operates on linear Rec.709 in and out; no gamut mapping (glTF assumes
//  Rec.709), so no matrices. The reference compression is followed by the sRGB
//  OETF for display, matching the Khronos OCIO config (... -> sRGB encode).
//
//  Input:  linear sRGB / Rec.709, scene-referred.
//  Output: sRGB-encoded, [0, 1].
// -----------------------------------------------------------------------------
inline Vector3f pbrNeutral(Vector3f color) {
    const float startCompression = 0.8f - 0.04f;
    const float desaturation = 0.15f;

    float x = std::min(color.x, std::min(color.y, color.z));
    float offset = x < 0.08f ? x - 6.25f * x * x : 0.04f;
    color.x -= offset;
    color.y -= offset;
    color.z -= offset;

    float peak = std::max(color.x, std::max(color.y, color.z));
    if (peak < startCompression)
        return srgbOETF(color);

    float d = 1.f - startCompression;
    float newPeak = 1.f - d * d / (peak + d - startCompression);
    float scale = newPeak / peak;
    color.x *= scale;
    color.y *= scale;
    color.z *= scale;

    float g = 1.f - 1.f / (desaturation * (peak - newPeak) + 1.f);
    // mix(color, newPeak*white, g)
    color.x = color.x * (1.f - g) + newPeak * g;
    color.y = color.y * (1.f - g) + newPeak * g;
    color.z = color.z * (1.f - g) + newPeak * g;

    return srgbOETF(color);
}

// -----------------------------------------------------------------------------
//  Dispatch
// -----------------------------------------------------------------------------
inline Vector3f apply(ToneMapper mapper, Vector3f linear) {
    switch (mapper) {
    case ToneMapper::Raw:
        return raw(linear);
    case ToneMapper::Standard:
        return standard(linear);
    case ToneMapper::AgX:
        return agx(linear);
    case ToneMapper::PBRNeutral:
        return pbrNeutral(linear);
    }
    return standard(linear); // unreachable; keeps the compiler quiet.
}

// Convenience: parse a mapper name (e.g. for CLI / settings).
inline ToneMapper fromString(const char *name) {
    if (name) {
        std::string n(name);
        if (n == "agx" || n == "AgX" || n == "AGX")
            return ToneMapper::AgX;
        if (n == "pbrneutral" || n == "PBRNeutral" || n == "pbr_neutral" || n == "pbr" ||
            n == "neutral")
            return ToneMapper::PBRNeutral;
        if (n == "standard" || n == "Standard" || n == "STANDARD")
            return ToneMapper::Standard;
        if (n == "raw" || n == "Raw" || n == "RAW")
            return ToneMapper::Raw;
    }
    return ToneMapper::AgX; // sensible default
}

inline const char *toString(ToneMapper mapper) {
    switch (mapper) {
    case ToneMapper::Raw:
        return "raw";
    case ToneMapper::Standard:
        return "standard";
    case ToneMapper::AgX:
        return "agx";
    case ToneMapper::PBRNeutral:
        return "pbrneutral";
    }
    return "standard";
}

} // namespace tonemap