#include "Material.hpp"

ShadingData Material::buildShadingData(const Vector2f &uv, const Vector3f &Ng, const Vector3f &T,
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

        Vector3f Tn = normalize(T - dotProduct(T, Ng) * Ng);
        Vector3f Bn = crossProduct(Ng, Tn);

        sd.N = normalize(tn.x * Tn + tn.y * Bn + tn.z * Ng);
    } else {
        sd.N = Ng;
    }

    return sd;
}

BSDFSample Material::sample(const Vector3f &wo, const ShadingData &sd) const {
    BSDFSample result;
    result.f = Vector3f(0);
    result.pdf = 0.f;

    switch (m_type) {
    case DIFFUSE: {
        float alpha = sd.roughness * sd.roughness;
        Vector3f F0 = lerp(Vector3f(0.04f), sd.baseColor, sd.metallic);
        Vector3f fresnel =
            F0 + (Vector3f(1) - F0) * std::pow(1.f - std::max(0.f, dotProduct(wo, sd.N)), 5.f);
        float specularWeight =
            fresnel.x * 0.2126f + fresnel.y * 0.7152f + fresnel.z * 0.0722f; // luminance
        bool doSpecular = get_random_float() < specularWeight;

        if (doSpecular) {
            Vector3f H = sampleGGX(sd.N, alpha);
            result.wi = normalize(reflect(-wo, H));
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
    default:
        // TODO FIX: MIRRORS/GLASS ARE HANDLED IN INTEGRATOR (BAD BAD BAD)
        return result;
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const {
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
        // TODO FIX: MIRRORS/GLASS ARE HANDLED IN INTEGRATOR (BAD BAD BAD)
        return Vector3f(0);
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const {
    switch (m_type) {
    case DIFFUSE: {
        float NdotWi = dotProduct(wi, sd.N);
        float NdotWo = dotProduct(wo, sd.N);
        if (NdotWi <= 0.f || NdotWo <= 0.f)
            return 0.f;

        float alpha = sd.roughness * sd.roughness;
        Vector3f F0 = lerp(Vector3f(0.04f), sd.baseColor, sd.metallic);
        Vector3f fresnel =
            F0 + (Vector3f(1) - F0) * std::pow(1.f - std::max(0.f, dotProduct(wo, sd.N)), 5.f);
        float specularWeight =
            fresnel.x * 0.2126f + fresnel.y * 0.7152f + fresnel.z * 0.0722f; // luminance
        Vector3f H = normalize(wi + wo);
        float NdotH = std::max(0.f, dotProduct(sd.N, H));
        float VdotH = std::max(0.f, dotProduct(wo, H));

        return pdfMixed(NdotWi, NdotH, VdotH, alpha, specularWeight);
    }
    default:
        return 0.f;
    }
}