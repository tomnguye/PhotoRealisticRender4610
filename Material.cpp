#include "Material.hpp"

ShadingData DiffuseMaterial::buildShadingData(const Vector2f &uv, const Vector3f &Ng,
                                              const Vector3f &T, const Vector3f &B) const {
    ShadingData sd;
    sd.uv = uv;
    sd.Ng = Ng;
    sd.T = T;
    sd.B = B;

    sd.baseColor = TextureUtils::sampleBaseColor(baseColorTex, uv, baseColor);

    Vector2f mr =
        TextureUtils::sampleMetallicRoughness(metallicRoughnessTex, uv, roughness, metallic);
    sd.roughness = std::max(mr.x, 0.01f);
    sd.metallic = mr.y;

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

BSDFSample DiffuseMaterial::sample(const Vector3f &wo, const ShadingData &sd) const {
    BSDFSample result;
    result.f = Vector3f(0.0f);
    result.pdf = 0.0f;

    float alpha = sd.roughness * sd.roughness;
    Vector3f F0 = computeF0(sd.metallic, sd.baseColor);
    Vector3f fresnel =
        F0 + (Vector3f(1.0f) - F0) * std::pow(1.0f - std::max(0.0f, dotProduct(wo, sd.N)), 5.0f);
    float specularWeight = fresnel.x * 0.2126f + fresnel.y * 0.7152f + fresnel.z * 0.0722f;
    bool doSpecular = get_random_float() < specularWeight;

    if (doSpecular) {
        Vector3f H = sampleGGX(sd.N, alpha);
        result.wi = normalize(reflect(-wo, H));
        result.lobe = LOBE_SPECULAR;
    } else {
        result.wi = sampleCosineHemisphere(sd.N);
        result.lobe = LOBE_DIFFUSE;
    }

    if (dotProduct(result.wi, sd.N) <= 0.0f)
        return result;

    result.f = eval(result.wi, wo, sd);
    result.pdf = pdf(result.wi, wo, sd);
    return result;
}

Vector3f DiffuseMaterial::eval(const Vector3f &wi, const Vector3f &wo,
                               const ShadingData &sd) const {
    float NdotWo = dotProduct(wo, sd.N);
    float NdotWi = dotProduct(wi, sd.N);
    if (NdotWo <= 0.0f || NdotWi <= 0.0f)
        return Vector3f(0.0f);

    float alpha = sd.roughness * sd.roughness;
    Vector3f H = normalize(wi + wo);
    float NdotH = std::max(0.0f, dotProduct(sd.N, H));
    float VdotH = std::max(0.0f, dotProduct(wo, H));

    float D = D_GGX(NdotH, alpha);
    Vector3f F0 = computeF0(sd.metallic, sd.baseColor);
    Vector3f F = F_Schlick(VdotH, F0);
    float G = G_Smith(NdotWo, NdotWi, alpha);

    Vector3f specular = D * F * G / (4.0f * NdotWo * NdotWi + 1e-6f);
    Vector3f diffuse = (Vector3f(1.0f) - F) * (1.0f - sd.metallic) * sd.baseColor / M_PI;

    return diffuse + specular;
}

float DiffuseMaterial::pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd) const {
    float NdotWi = dotProduct(wi, sd.N);
    float NdotWo = dotProduct(wo, sd.N);
    if (NdotWi <= 0.0f || NdotWo <= 0.0f)
        return 0.0f;

    float alpha = sd.roughness * sd.roughness;
    Vector3f F0 = computeF0(sd.metallic, sd.baseColor);
    Vector3f fresnel =
        F0 + (Vector3f(1.0f) - F0) * std::pow(1.0f - std::max(0.0f, dotProduct(wo, sd.N)), 5.0f);
    float specularWeight = fresnel.x * 0.2126f + fresnel.y * 0.7152f + fresnel.z * 0.0722f;
    Vector3f H = normalize(wi + wo);
    float NdotH = std::max(0.0f, dotProduct(sd.N, H));
    float VdotH = std::max(0.0f, dotProduct(wo, H));

    return pdfMixed(NdotWi, NdotH, VdotH, alpha, specularWeight);
}

BSDFSample MirrorMaterial::sample(const Vector3f &wo, const ShadingData &sd) const {
    BSDFSample result;
    result.wi = reflect(-wo, sd.N).normalized();
    result.f = baseColor;
    result.pdf = 1.0f;
    result.lobe = LOBE_DELTA;
    return result;
}

BSDFSample GlassMaterial::sample(const Vector3f &wo, const ShadingData &sd) const {
    BSDFSample result;
    result.pdf = 1.0f;
    result.lobe = LOBE_DELTA;

    Vector3f incident = -wo;
    float kr = fresnel(incident, sd.Ng, ior);
    Vector3f refracted = refract(incident, sd.Ng, ior);
    bool tir = refracted.norm() < 1e-6f;

    if (get_random_float() < kr || tir)
        result.wi = reflect(incident, sd.Ng).normalized();
    else
        result.wi = refracted.normalized();

    result.f = baseColor;
    return result;
}