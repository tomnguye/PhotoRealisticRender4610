#pragma once

#include "BRDFUtils.hpp"
#include "TextureUtils.hpp"
#include "Vector.hpp"
#include "global.hpp"

enum MaterialType { DIFFUSE, GLASS, MIRROR, EMIT };

class Material {
public:
    MaterialType m_type = DIFFUSE;
    Vector3f m_emission = Vector3f(0);
    float ior = 1.5f;
    Vector3f baseColor = Vector3f(1.f);
    float roughness = 1.f;
    float metallic = 0.f;

    // Textures are set by gltf loader, but use TextureUtils to sample them.
    Texture baseColorTex;
    Texture normalTex;
    Texture metallicRoughnessTex;
    Texture emissiveTex;

    explicit Material(MaterialType t = DIFFUSE, Vector3f color = Vector3f(1.f))
        : m_type(t), baseColor(color) {}

    MaterialType getType() const { return m_type; }
    Vector3f getEmission() const { return m_emission; }
    bool hasEmission() const { return m_emission.norm() > EPSILON; }

    ShadingData buildShadingData(const Vector2f &uv, const Vector3f &Ng, const Vector3f &T,
                                 const Vector3f &B) const;

    // Convenience: build ShadingData when the mesh has no tangents.
    // Generates an arbitrary tangent frame from Ng.
    ShadingData buildShadingData(const Vector2f &uv, const Vector3f &Ng) const {
        Vector3f T, B;
        buildTBN(Ng, T, B);
        return buildShadingData(uv, Ng, T, B);
    }

    // Importance-sample an incoming direction wi.
    // Returns BSDFSample with wi, f, pdf, and lobe type.
    BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const;

    // Evaluate the full PBR BRDF f(wi, wo) — Cook-Torrance specular + Lambertian diffuse.
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const;

    // Mixed PDF matching the sampling strategy in sample().
    float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const;

    // Resolve emissive value at a UV, factoring in the emissive texture.
    // Returns m_emission alone if no emissive texture is present.
    Vector3f evalEmissive(const Vector2f &uv) const {
        return TextureUtils::sampleEmissive(emissiveTex, uv, m_emission);
    }

    // static Vector3f reflectDir(const Vector3f &I, const Vector3f &N) {
    //     return I - 2.f * dotProduct(I, N) * N;
    // }

    // static Vector3f refractDir(const Vector3f &I, const Vector3f &N, float ior_) {
    //     float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
    //     float etai = 1.f, etat = ior_;
    //     Vector3f n = N;
    //     if (cosi < 0.f) {
    //         cosi = -cosi;
    //     } else {
    //         std::swap(etai, etat);
    //         n = -N;
    //     }
    //     float eta = etai / etat;
    //     float k = 1.f - eta * eta * (1.f - cosi * cosi);
    //     return k < 0.f ? Vector3f(0) : eta * I + (eta * cosi - sqrtf(k)) * n;
    // }

    // Returns reflectance kr. Transmittance = 1 - kr.
    // static float fresnelDielectric(const Vector3f &I, const Vector3f &N, float ior_) {
    //     float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
    //     float etai = 1.f, etat = ior_;
    //     if (cosi > 0.f)
    //         std::swap(etai, etat);
    //     float sint = etai / etat * sqrtf(std::max(0.f, 1.f - cosi * cosi));
    //     if (sint >= 1.f)
    //         return 1.f;
    //     float cost = sqrtf(std::max(0.f, 1.f - sint * sint));
    //     cosi = std::abs(cosi);
    //     float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    //     float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    //     return (Rs * Rs + Rp * Rp) * 0.5f;
    // }
};
