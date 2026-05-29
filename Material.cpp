#include "Material.hpp"

ShadingData Material::buildShadingData(const Vector2f &uv, const Vector3f &Ng, const Vector3f &T,
                                       const Vector3f &B) const
{
    ShadingData sd;
    sd.uv = uv;
    sd.Ng = Ng;
    sd.T = T;
    sd.B = B;

    sd.baseColor = TextureUtils::sampleBaseColor(baseColorTex, uv, baseColor);

    Vector2f mr =
        TextureUtils::sampleMetallicRoughness(metallicRoughnessTex, uv, roughness, metallic);
    sd.roughness = std::max(mr.x, 0.04f);
    sd.metallic = mr.y;

    if (!normalTex.empty())
    {
        Vector3f tn = TextureUtils::sampleNormalMap(normalTex, uv);
        sd.N = normalize(tn.x * T + tn.y * B + tn.z * Ng);
    }
    else
    {
        sd.N = Ng;
    }

    return sd;
}

BSDFSample DiffuseMaterial::sample(const Vector3f &wo, const ShadingData &sd) const
{
    float alpha = sd.roughness * sd.roughness;
    Vector3f F0 = computeF0(sd.metallic, sd.baseColor);
    float F0max = std::max({F0.x, F0.y, F0.z});
    float specWeight = std::clamp(sd.metallic + (1.f - sd.metallic) * F0max, 0.001f, 0.999f);

    Vector3f woLocal = toLocal(wo, sd.N);
    if (woLocal.z <= 0.f)
        return {};

    float u0 = get_random_float();
    float u1 = get_random_float();
    float u2 = get_random_float();

    Vector3f wiLocal;
    LobeType lobe;

    if (u0 < specWeight)
    {
        // Specular: sample VNDF
        Vector3f H = sampleGGX_VNDF(woLocal, alpha, u1, u2);
        if (dotProduct(H, H) < 1e-8f)
            return {};
        wiLocal = reflect(woLocal, H);
        lobe = LOBE_SPECULAR;
    }
    else
    {
        // Diffuse: sample cosine hemisphere in local space directly
        float r = std::sqrt(u1);
        float phi = 2.f * M_PI * u2;
        wiLocal =
            Vector3f(r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.f, 1.f - u1)));
        lobe = LOBE_DIFFUSE;
    }

    if (wiLocal.z <= 0.f)
        return {};

    Vector3f wi = toWorld(wiLocal, sd.N);

    BSDFSample s;
    s.wi = wi;
    s.lobe = lobe;
    s.pdf = pdf(wi, wo, sd, lobe);
    s.f = (s.pdf > 1e-6f) ? eval(wi, wo, sd, lobe) : Vector3f(0.f);
    return s;
}

Vector3f DiffuseMaterial::eval(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd,
                               LobeType lobe) const
{
    float alpha = sd.roughness * sd.roughness;
    Vector3f F0 = computeF0(sd.metallic, sd.baseColor);

    Vector3f wiLocal = toLocal(wi, sd.N);
    Vector3f woLocal = toLocal(wo, sd.N);
    float NdotWi = wiLocal.z;
    float NdotWo = woLocal.z;
    if (NdotWi <= 0.f || NdotWo <= 0.f)
        return Vector3f(0.f);

    Vector3f H = wiLocal + woLocal;
    if (dotProduct(H, H) < 1e-8f)
        return Vector3f(0.f);
    H = normalize(H);
    if (H.z < 0.f)
        H = -H;

    float NdotH = std::max(0.f, H.z);
    float WoDotH = std::max(0.f, dotProduct(woLocal, H));

    Vector3f F = F_Schlick(WoDotH, F0);

    if (lobe == LOBE_SPECULAR)
    {
        float D = D_GGX(NdotH, alpha);
        float G = G_SmithHeightCorrelated(NdotWo, NdotWi, alpha);
        float denom = 4.f * std::max(NdotWo, 1e-4f) * std::max(NdotWi, 1e-4f);
        Vector3f specular = D * G * F / denom;
        return specular * NdotWi;
    }
    else if (lobe == LOBE_DIFFUSE)
    {
        Vector3f kD = (Vector3f(1.f) - F) * (1.f - sd.metallic);
        return kD * sd.baseColor / M_PI * NdotWi;
    }
    else
    {
        // LOBE_ALL: evaluate both lobes combined (used for direct light MIS)
        float D = D_GGX(NdotH, alpha);
        float G = G_SmithHeightCorrelated(NdotWo, NdotWi, alpha);
        float denom = 4.f * NdotWo * NdotWi;
        Vector3f specular = (denom > 1e-6f) ? D * G * F / denom : Vector3f(0.f);
        Vector3f kD = (Vector3f(1.f) - F) * (1.f - sd.metallic);
        Vector3f diffuse = kD * sd.baseColor / M_PI;
        return (diffuse + specular) * NdotWi;
    }
}

float DiffuseMaterial::pdf(const Vector3f &wi, const Vector3f &wo, const ShadingData &sd,
                           LobeType lobe) const
{
    float alpha = sd.roughness * sd.roughness;
    Vector3f F0 = computeF0(sd.metallic, sd.baseColor);
    float F0max = std::max({F0.x, F0.y, F0.z});
    float specWeight = std::clamp(sd.metallic + (1.f - sd.metallic) * F0max, 0.001f, 0.999f);

    Vector3f wiLocal = toLocal(wi, sd.N);
    Vector3f woLocal = toLocal(wo, sd.N);
    float NdotWi = wiLocal.z;
    float NdotWo = woLocal.z;
    if (NdotWi <= 0.f || NdotWo <= 0.f)
        return 0.f;

    Vector3f H = wiLocal + woLocal;
    if (dotProduct(H, H) < 1e-8f)
        return 0.f;
    H = normalize(H);
    if (H.z < 0.f)
        H = -H;
    float NdotH = std::max(0.f, H.z);
    float WoDotH = std::max(0.f, dotProduct(woLocal, H));

    float ps = pdfGGX_VNDF(NdotWo, NdotH, WoDotH, alpha);
    float pd = pdfCosineHemisphere(NdotWi);

    if (lobe == LOBE_SPECULAR)
        return specWeight * ps;
    else if (lobe == LOBE_DIFFUSE)
        return (1.f - specWeight) * pd;
    else
        // LOBE_ALL: full mixed pdf (used for direct light MIS weight)
        return specWeight * ps + (1.f - specWeight) * pd;
}

BSDFSample MirrorMaterial::sample(const Vector3f &wo, const ShadingData &sd) const
{
    BSDFSample result;
    result.wi = normalize(reflect(wo, sd.N));
    result.f = baseColor;
    result.pdf = 1.0f;
    result.lobe = LOBE_DELTA;
    return result;
}

BSDFSample GlassMaterial::sample(const Vector3f &wo, const ShadingData &sd) const
{
    BSDFSample result;
    result.pdf = 1.0f;
    result.lobe = LOBE_DELTA;

    Vector3f incident = -wo;
    float kr = fresnel(incident, sd.N, ior);
    Vector3f refracted = refract(incident, sd.N, ior);
    bool tir = refracted.norm() < 1e-6f;

    bool entering = dotProduct(incident, sd.N) < 0.f;
    float eta = entering ? (1.0f / ior) : ior;

    if (get_random_float() < kr || tir)
    {
        result.wi = normalize(reflect(wo, sd.N));
        result.f = Vector3f(1.0f);
    }
    else
    {
        result.wi = normalize(refracted);
        result.f = Vector3f(eta * eta); // non-reciprocal eta^2 correction
    }

    return result;
}