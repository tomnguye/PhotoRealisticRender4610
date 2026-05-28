#pragma once

#include "Vector.hpp"
#include "global.hpp"

/*
The convention for this is:
incoming ray wi points from surface -> light.
outgoing ray wo points from surface -> camera.
*/

/**
 * @brief Given a normal N, build a tangent T and bitangent B that are orthonormal to N.
 */
inline void buildTBN(const Vector3f &N, Vector3f &T, Vector3f &B) {
    if (std::fabs(N.x) > std::fabs(N.y)) {
        float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
        T = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
    } else {
        float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
        T = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
    }
    // Right-handed basis: with toWorld() doing x*T + y*B + z*N, we need
    // cross(T, B) == N, which requires B = cross(N, T). The previous
    // cross(T, N) produced a left-handed (reflected) frame. That was invisible
    // when sd.N == Ng because toLocal/toWorld used it consistently and the
    // reflection cancelled on round-trip, but it corrupted tangent-space normal
    // map perturbations, which are built in a separate right-handed UV frame.
    B = crossProduct(N, T);
}

/**
 * @brief Transform a direction from tangent/local space to world space, aligned to N.
 */
inline Vector3f toWorld(const Vector3f &local, const Vector3f &N) {
    Vector3f T, B;
    buildTBN(N, T, B);
    return local.x * T + local.y * B + local.z * N;
}

/**
 * @brief Transform a direction from world space to local/tangent space aligned to N.
 *        In local space, N maps to (0,0,1).
 */
inline Vector3f toLocal(const Vector3f &world, const Vector3f &N) {
    Vector3f T, B;
    buildTBN(N, T, B);
    return Vector3f(dotProduct(world, T), dotProduct(world, B), dotProduct(world, N));
}

// ─── Reflect ─────────────────────────────────────────────────────────────────

/**
 * @brief Reflect direction I about normal N. I should point away from the surface.
 */
static Vector3f reflect(const Vector3f &wo, const Vector3f &N) {
    return 2.f * dotProduct(wo, N) * N - wo;
}

// ─── Cosine-Weighted Hemisphere ───────────────────────────────────────────────

/**
 * @brief Sample a cosine-weighted direction on the hemisphere aligned to N.
 *        pdf = NdotL / PI
 */
inline Vector3f sampleCosineHemisphere(const Vector3f &N) {
    float r1 = get_random_float();
    float r2 = get_random_float();
    float r = std::sqrt(r1);
    float phi = 2.f * M_PI * r2;
    return toWorld(
        Vector3f(r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.f, 1.f - r1))), N);
}

/**
 * @brief PDF for a cosine-weighted hemisphere sample.
 * @param NdotWi dot(N, wi), wi points surface -> light
 */
inline float pdfCosineHemisphere(float NdotWi) {
    return std::max(0.f, NdotWi) / M_PI;
}

// ─── GGX NDF ─────────────────────────────────────────────────────────────────

/**
 * @brief GGX Normal Distribution Function.
 * @param NdotH dot(N, H)
 * @param alpha  roughness² (perceptual roughness squared)
 */
inline float D_GGX(float NdotH, float alpha) {
    float alpha2 = alpha * alpha;
    float denom = NdotH * NdotH * (alpha2 - 1.f) + 1.f;
    return alpha2 / (M_PI * denom * denom);
}

// ─── Smith Geometry ───────────────────────────────────────────────────────────

/**
 * @brief Smith GGX single-direction masking term (G1).
 * @param NdotV dot(N, V) for the direction being evaluated
 * @param alpha  roughness²
 */
inline float G1_GGX(float NdotV, float alpha) {
    float alpha2 = alpha * alpha;
    float denom = NdotV + std::sqrt(alpha2 + (1.f - alpha2) * NdotV * NdotV);
    return NdotV / std::max(denom, 1e-6f);
}

/**
 * @brief Height-correlated Smith G2 (separable approximation).
 *        More accurate than naive G1(wi)*G1(wo) but still cheap.
 * @param NdotWo dot(N, wo), wo points surface -> camera
 * @param NdotWi dot(N, wi), wi points surface -> light
 * @param alpha   roughness²
 */
inline float G_Smith(float NdotWo, float NdotWi, float alpha) {
    return G1_GGX(NdotWo, alpha) * G1_GGX(NdotWi, alpha);
}

// ─── Fresnel ──────────────────────────────────────────────────────────────────

/**
 * @brief Schlick Fresnel approximation.
 * @param WoDotH dot(wo, H)
 * @param F0     base reflectance at normal incidence
 */
inline Vector3f F_Schlick(float WoDotH, const Vector3f &F0) {
    float t = std::pow(std::max(0.f, 1.f - WoDotH), 5.f);
    return F0 + (Vector3f(1.f) - F0) * t;
}

// ─── GGX VNDF Sampling ───────────────────────────────────────────────────────

/**
 * @brief Sample a microfacet normal from the GGX Visible Normal Distribution (Heitz 2018).
 *        Produces far less variance than naive NDF sampling, especially at low roughness.
 *
 *        IMPORTANT: wo must be in LOCAL space (N = Z axis).
 *        Returns the sampled half-vector H in LOCAL space.
 *
 * @param wo    Outgoing direction in local space (surface -> camera). Must have wo.z > 0.
 * @param alpha roughness² (perceptual roughness squared)
 * @param u1    Uniform random sample in [0, 1)
 * @param u2    Uniform random sample in [0, 1)
 * @return Vector3f Sampled half-vector H in local space.
 */
inline Vector3f sampleGGX_VNDF(const Vector3f &wo, float alpha, float u1, float u2) {
    Vector3f Vh = normalize(Vector3f(alpha * wo.x, alpha * wo.y, wo.z));

    float lensq = Vh.x * Vh.x + Vh.y * Vh.y;
    Vector3f T1 =
        lensq > 0.f ? Vector3f(-Vh.y, Vh.x, 0.f) / std::sqrt(lensq) : Vector3f(1.f, 0.f, 0.f);
    Vector3f T2 = crossProduct(Vh, T1);

    float r = std::sqrt(u1);
    float phi = 2.f * M_PI * u2;
    float t1 = r * std::cos(phi);
    float t2 = r * std::sin(phi);
    float s = 0.5f * (1.f + Vh.z);
    t2 = (1.f - s) * std::sqrt(1.f - t1 * t1) + s * t2;

    Vector3f Nh = t1 * T1 + t2 * T2 + std::sqrt(std::max(0.f, 1.f - t1 * t1 - t2 * t2)) * Vh;

    return normalize(Vector3f(alpha * Nh.x, alpha * Nh.y, std::max(0.f, Nh.z)));
}

/**
 * @brief PDF for a GGX VNDF sample, expressed as a density over wi directions.
 *
 *        Use this (not pdfGGX) when sampling with sampleGGX_VNDF.
 *        Derivation: pdf(H) = D(H)*G1(wo)*dot(wo,H)/dot(N,wo),
 *        then change of variables to wi gives the /4*dot(wo,H) Jacobian.
 *
 * @param NdotWo dot(N, wo)
 * @param NdotH  dot(N, H)
 * @param WoDotH dot(wo, H)
 * @param alpha  roughness²
 */
inline float pdfGGX_VNDF(float NdotWo, float NdotH, float WoDotH, float alpha) {
    float D = D_GGX(NdotH, alpha);
    float G1 = G1_GGX(NdotWo, alpha);
    // pdf(H) = D * G1 * WoDotH / NdotWo
    // pdf(wi) = pdf(H) / (4 * WoDotH)  <- reflection Jacobian
    return D * G1 * std::max(0.f, WoDotH) /
           (std::max(1e-6f, NdotWo) * 4.f * std::max(1e-4f, WoDotH));
    // Simplifies to:
    // return D * G1 / (4.f * std::max(1e-6f, NdotWo));
}

/**
 * @brief Mixed PDF for one-sample MIS between diffuse (Lambert) and specular (GGX VNDF) lobes.
 *
 * @param NdotWi     dot(N, wi)
 * @param NdotWo     dot(N, wo)
 * @param NdotH      dot(N, H)
 * @param WoDotH     dot(wo, H)
 * @param alpha      roughness²
 * @param specWeight probability of having chosen the specular lobe [0,1]
 */
inline float pdfMixed(float NdotWi, float NdotWo, float NdotH, float WoDotH, float alpha,
                      float specWeight) {
    float pd = pdfCosineHemisphere(NdotWi);
    float ps = pdfGGX_VNDF(NdotWo, NdotH, WoDotH, alpha);
    return specWeight * ps + (1.f - specWeight) * pd;
}

// ─── Misc ─────────────────────────────────────────────────────────────────────

inline float luminance(const Vector3f &c) {
    return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

// ─── Legacy dielectric helpers ────────────────────────────────────────────────
// (kept for compatibility with glass/dielectric materials)

static Vector3f refract(const Vector3f &I, const Vector3f &N, float ior) {
    float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
    float etai = 1.f, etat = ior;
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

static float fresnel(const Vector3f &I, const Vector3f &N, float ior) {
    float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
    float etai = 1.f, etat = ior;
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

/**
 * @brief Height-correlated Smith G2 for GGX (Heitz 2014).
 *        More accurate than separable G1*G1 — always <= separable,
 *        so the separable form was slightly over-bright at grazing angles.
 * @param NdotWo dot(N, wo)
 * @param NdotWi dot(N, wi)
 * @param alpha  roughness²
 */
inline float G_SmithHeightCorrelated(float NdotWo, float NdotWi, float alpha) {
    float a2 = alpha * alpha;
    float t2o = (1.f - NdotWo * NdotWo) / std::max(NdotWo * NdotWo, 1e-6f); // tan²θo
    float t2i = (1.f - NdotWi * NdotWi) / std::max(NdotWi * NdotWi, 1e-6f); // tan²θi
    return 2.f / (1.f + std::sqrt(1.f + a2 * t2o) + std::sqrt(1.f + a2 * t2i));
}