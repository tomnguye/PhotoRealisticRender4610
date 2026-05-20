#pragma once

#include "BRDFUtils.hpp"
#include "TextureUtils.hpp"
#include "Vector.hpp"
#include "global.hpp"

// ─── Material types ───────────────────────────────────────────────────────────

enum MaterialType { DIFFUSE, GLASS, MIRROR, EMIT };

// ─── Material ─────────────────────────────────────────────────────────────────
//
// Owns PBR parameters and texture data for one glTF primitive.
//
// Responsibilities:
//   - Store raw material parameters (baseColor, roughness, metallic, emission)
//   - Store texture data (via Texture structs)
//   - Expose buildShadingData() to resolve all textures into a ShadingData
//   - Expose eval / sample / pdf which take ShadingData — pure math, no texture
//     lookups inside
//
// What Material does NOT do:
//   - Texture sampling inside eval/sample/pdf (done at hit time via buildShadingData)
//   - Store raw byte arrays directly (delegated to Texture)
//   - Apply normal maps (buildShadingData handles TBN transform)

class Material {
public:
    // ── Type and non-PBR properties ───────────────────────────────────────────

    MaterialType m_type = DIFFUSE;
    Vector3f m_emission = Vector3f(0);
    float ior = 1.5f; // glass only

    // ── PBR parameters (factors, multiplied with texture sample if present) ────

    Vector3f baseColor = Vector3f(1.f);
    float roughness = 1.f;
    float metallic = 0.f;
    float Ks = 0.2f; // specular lobe sampling weight

    // ── Textures ──────────────────────────────────────────────────────────────
    // Set by the glTF loader. Sampled via TextureUtils at hit time only.

    Texture baseColorTex;         // sRGB encoded, RGBA
    Texture normalTex;            // linear, tangent-space XYZ in RGB
    Texture metallicRoughnessTex; // linear, G = roughness, B = metallic
    Texture emissiveTex;          // sRGB encoded, RGB

    // ── Constructor ───────────────────────────────────────────────────────────

    explicit Material(MaterialType t = DIFFUSE, Vector3f color = Vector3f(1.f))
        : m_type(t), baseColor(color) {}

    // ── Accessors ─────────────────────────────────────────────────────────────

    MaterialType getType() const { return m_type; }
    Vector3f getEmission() const { return m_emission; }
    bool hasEmission() const { return m_emission.norm() > EPSILON; }

    // ── ShadingData builder ───────────────────────────────────────────────────
    //
    // Call this once per hit to resolve all textures into a flat ShadingData.
    // Caller supplies:
    //   uv  — texture coordinates at the hit point
    //   Ng  — geometric normal, already flipped to face the incoming ray
    //   T   — tangent vector from mesh (or generated if mesh has no tangents)
    //   B   — bitangent vector from mesh
    //
    // This is the only place texture sampling happens.
    // Normal map is transformed from tangent space to world space here.

    ShadingData buildShadingData(const Vector2f &uv, const Vector3f &Ng, const Vector3f &T,
                                 const Vector3f &B) const {
        ShadingData sd;
        sd.uv = uv;
        sd.Ng = Ng;
        sd.T = T;
        sd.B = B;

        // Base colour — sRGB decode handled inside TextureUtils
        sd.baseColor = TextureUtils::sampleBaseColor(baseColorTex, uv, baseColor);

        // Metallic / roughness — modulate factor by texture
        Vector2f mr =
            TextureUtils::sampleMetallicRoughness(metallicRoughnessTex, uv, roughness, metallic);
        sd.roughness = std::max(mr.x, 0.01f); // clamp: avoids degenerate GGX lobe
        sd.metallic = mr.y;

        // Shading normal — apply normal map if present
        if (!normalTex.empty()) {
            Vector3f tn = TextureUtils::sampleNormalMap(normalTex, uv);

            // Re-orthogonalise T against Ng to absorb interpolation drift
            Vector3f Tn = normalize(T - dotProduct(T, Ng) * Ng);
            Vector3f Bn = crossProduct(Ng, Tn);

            sd.N = normalize(tn.x * Tn + tn.y * Bn + tn.z * Ng);
        } else {
            sd.N = Ng;
        }

        return sd;
    }

    // Convenience: build ShadingData when the mesh has no tangents.
    // Generates an arbitrary tangent frame from Ng.
    ShadingData buildShadingData(const Vector2f &uv, const Vector3f &Ng) const {
        Vector3f T, B;
        buildTBN(Ng, T, B);
        return buildShadingData(uv, Ng, T, B);
    }

    // ── BRDF interface ────────────────────────────────────────────────────────
    //
    // All three functions take a fully resolved ShadingData — no texture work.
    //
    // Convention:
    //   wi = incoming light direction  (points AWAY from surface)
    //   wo = outgoing view direction   (points AWAY from surface)
    //   N  = sd.N, already normal-mapped and in world space

    // Importance-sample an incoming direction wi.
    // Returns BSDFSample with wi, f, pdf, and lobe type.
    BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const;

    // Evaluate the full PBR BRDF f(wi, wo) — Cook-Torrance specular + Lambertian diffuse.
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const;

    // Mixed PDF matching the sampling strategy in sample().
    float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const;

    // ── Emissive helper ───────────────────────────────────────────────────────
    // Resolve emissive value at a UV, factoring in the emissive texture.
    // Returns m_emission alone if no emissive texture is present.
    Vector3f evalEmissive(const Vector2f &uv) const {
        return TextureUtils::sampleEmissive(emissiveTex, uv, m_emission);
    }

    // ── Dielectric helpers (used by Scene::castRay for GLASS / MIRROR) ────────

    static Vector3f reflectDir(const Vector3f &I, const Vector3f &N) {
        return I - 2.f * dotProduct(I, N) * N;
    }

    static Vector3f refractDir(const Vector3f &I, const Vector3f &N, float ior_) {
        float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
        float etai = 1.f, etat = ior_;
        Vector3f n = N;
        if (cosi < 0.f) {
            cosi = -cosi;
        } else {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1.f - eta * eta * (1.f - cosi * cosi);
        return k < 0.f ? Vector3f(0) : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Returns reflectance kr. Transmittance = 1 - kr.
    static float fresnelDielectric(const Vector3f &I, const Vector3f &N, float ior_) {
        float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
        float etai = 1.f, etat = ior_;
        if (cosi > 0.f)
            std::swap(etai, etat);
        float sint = etai / etat * sqrtf(std::max(0.f, 1.f - cosi * cosi));
        if (sint >= 1.f)
            return 1.f;
        float cost = sqrtf(std::max(0.f, 1.f - sint * sint));
        cosi = std::abs(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) * 0.5f;
    }
};

// ─── BRDF implementations ─────────────────────────────────────────────────────

inline BSDFSample Material::sample(const Vector3f &wo, const ShadingData &sd) const {
    BSDFSample result;
    result.f = Vector3f(0);
    result.pdf = 0.f;

    switch (m_type) {
    case DIFFUSE: {
        float alpha = sd.roughness * sd.roughness;
        float specularWeight = sd.metallic + (1.f - sd.metallic) * Ks;
        bool doSpecular = get_random_float() < specularWeight;

        if (doSpecular) {
            Vector3f H = sampleGGX(sd.N, alpha);
            result.wi = normalize(reflectDir(-wo, H));
            result.lobe = LOBE_SPECULAR;
        } else {
            result.wi = sampleCosineHemisphere(sd.N);
            result.lobe = LOBE_DIFFUSE;
        }

        // Reject samples that go below the surface
        if (dotProduct(result.wi, sd.N) <= 0.f)
            return result;

        result.f = eval(result.wi, wo, sd);
        result.pdf = pdf(result.wi, wo, sd);
        return result;
    }
    // GLASS and MIRROR are delta distributions handled directly in Scene::castRay.
    default:
        return result;
    }
}

inline Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo,
                               const ShadingData &sd) const {
    switch (m_type) {
    case DIFFUSE: {
        float NdotWo = dotProduct(wo, sd.N);
        float NdotWi = dotProduct(wi, sd.N);
        if (NdotWo <= 0.f || NdotWi <= 0.f)
            return Vector3f(0);

        float alpha = sd.roughness * sd.roughness;
        Vector3f H = normalize(wi + wo);
        float NdotH = std::max(0.f, dotProduct(sd.N, H));
        float VdotH = std::max(0.f, dotProduct(wo, H));

        float D = D_GGX(NdotH, alpha);
        Vector3f F0 = lerp(Vector3f(0.04f), sd.baseColor, sd.metallic);
        Vector3f F = F_Schlick(VdotH, F0);
        float G = G_Smith(NdotWo, NdotWi, alpha);

        // Cook-Torrance specular
        Vector3f specular = D * F * G / (4.f * NdotWo * NdotWi + 1e-6f);

        // Lambertian diffuse — metals have no diffuse, Fresnel attenuates dielectrics
        Vector3f diffuse = (Vector3f(1.f) - F) * (1.f - sd.metallic) * sd.baseColor / M_PI;

        return diffuse + specular;
    }
    default:
        return Vector3f(0);
    }
}

inline float Material::pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const {
    switch (m_type) {
    case DIFFUSE: {
        float NdotWi = dotProduct(wi, sd.N);
        float NdotWo = dotProduct(wo, sd.N);
        if (NdotWi <= 0.f || NdotWo <= 0.f)
            return 0.f;

        float alpha = sd.roughness * sd.roughness;
        float specularWeight = sd.metallic + (1.f - sd.metallic) * Ks;
        Vector3f H = normalize(wi + wo);
        float NdotH = std::max(0.f, dotProduct(sd.N, H));
        float VdotH = std::max(0.f, dotProduct(wo, H));

        return pdfMixed(NdotWi, NdotH, VdotH, alpha, specularWeight);
    }
    default:
        return 0.f;
    }
}