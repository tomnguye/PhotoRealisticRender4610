#pragma once

#include "Vector.hpp"

// ─── Lobe flags ───────────────────────────────────────────────────────────────

enum LobeType {
    LOBE_DIFFUSE = 0,
    LOBE_SPECULAR = 1,
    LOBE_DELTA = 2 // perfect mirror / glass — skip MIS weighting
};

// ─── Result of Material::sample() ────────────────────────────────────────────

struct BSDFSample {
    Vector3f wi;   // sampled incoming direction (points away from surface)
    Vector3f f;    // BRDF value f(wi, wo) for this sample
    float pdf;     // probability density of wi
    LobeType lobe; // which lobe was sampled
};

// ─── Per-hit shading inputs ───────────────────────────────────────────────────
//
// Filled in by the integrator before calling sample / eval / pdf.
// Keeps BRDF functions free of raw hit-record dependencies.

struct ShadingData {
    Vector3f N;         // shading normal (normal-mapped, world space)
    Vector3f Ng;        // geometric normal (for self-intersection offset)
    Vector3f T;         // tangent  (world space, from mesh or built from N)
    Vector3f B;         // bitangent (world space)
    Vector2f uv;        // texture coordinates at hit point
    float roughness;    // perturbed by texture if present
    float metallic;     // perturbed by texture if present
    Vector3f baseColor; // perturbed by texture if present
};