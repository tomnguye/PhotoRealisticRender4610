#pragma once
#include "BRDFUtils.hpp"
#include "TextureUtils.hpp"
#include "Vector.hpp"
#include "global.hpp"

enum LobeType {
    LOBE_DIFFUSE = 0,
    LOBE_SPECULAR = 1,
    LOBE_DELTA = 2 // For perfect mirrors and glass.
};

/**
 * @brief Result obtained from sampling a BSDF.
 */
struct BSDFSample {
    Vector3f wi;   // Sampled incoming direction. Points away from surface.
    Vector3f f;    // BSDF value f(wi, wo) for this sample.
    float pdf;     // Probability density of having selected wi.
    LobeType lobe; // The lobe that was sampled.
};

/**
 * @brief Surface state for when a ray hits a surface. Used when calling sample(), eval(), or pdf().
 */
struct ShadingData {
    Vector3f Ng; // Geometric normal. Use for ray offsets.
    Vector3f N;  // Shading normal. Use for BSDF calculations.
    Vector3f T;  // Tangent.
    Vector3f B;  // Bitangent.
    Vector2f uv; // Texture coordinates at hit point.
    float roughness;
    float metallic;
    Vector3f baseColor;
};

class Material {
public:
    Vector3f m_emission = Vector3f(0.0f);
    Vector3f baseColor = Vector3f(1.0f);
    float ior = 1.5f;
    Texture emissiveTex;

    virtual ~Material() = default;

    virtual BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const = 0;
    virtual Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const = 0;
    virtual float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const = 0;

    virtual bool isDelta() const { return false; }
    virtual bool isEmissive() const { return false; }

    bool hasEmission() const { return m_emission.norm() > EPSILON; }
    Vector3f evalEmissive(const Vector2f &uv) const {
        return TextureUtils::sampleEmissive(emissiveTex, uv, m_emission);
    }

    Vector3f computeF0(float metallic, const Vector3f &color) const {
        float f0Scalar = (ior - 1.0f) / (ior + 1.0f);
        f0Scalar *= f0Scalar;
        return lerp(Vector3f(f0Scalar), color, metallic);
    }
};

class DiffuseMaterial : public Material {
public:
    float roughness = 1.0f;
    float metallic = 0.0f;
    Texture baseColorTex;
    Texture normalTex;
    Texture metallicRoughnessTex;

    ShadingData buildShadingData(const Vector2f &uv, const Vector3f &Ng, const Vector3f &T,
                                 const Vector3f &B) const;
    ShadingData buildShadingData(const Vector2f &uv, const Vector3f &Ng) const {
        Vector3f T, B;
        buildTBN(Ng, T, B);
        return buildShadingData(uv, Ng, T, B);
    }

    BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const override;
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override;
    float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override;
};

class EmissiveMaterial : public Material {
public:
    bool isEmissive() const override { return true; }

    BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const override {
        return {Vector3f(0.0f), Vector3f(0.0f), 0.0f, LOBE_DIFFUSE};
    }
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override {
        return Vector3f(0.0f);
    }
    float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override {
        return 0.0f;
    }
};

class MirrorMaterial : public Material {
public:
    BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const override;
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override {
        return Vector3f(0.0f);
    }
    float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override {
        return 0.0f;
    }
    bool isDelta() const override { return true; }
};

class GlassMaterial : public Material {
public:
    BSDFSample sample(const Vector3f &wo, const ShadingData &sd) const override;
    Vector3f eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override {
        return Vector3f(0.0f);
    }
    float pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const override {
        return 0.0f;
    }
    bool isDelta() const override { return true; }
};