#pragma once

#include "Vector.hpp"
#include "global.hpp"

/*
The convention for this is:
incoming ray wi points from surface -> light.
outgoing ray wo points from surface -> camera.
*/

/**
 * @brief Given a normal N, build a tangent T and bitangent B that are othornormal to N.
 *
 * @param N The normal to build an othonormal basis with.
 * @param T The tangent (This is mutated).
 * @param B The bitangent (This is mutated).
 */
inline void buildTBN(const Vector3f &N, Vector3f &T, Vector3f &B) {
    if (std::fabs(N.x) > std::fabs(N.y)) {
        float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
        T = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
    } else {
        float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
        T = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
    }
    B = crossProduct(T, N);
}

/**
 * @brief Transform a direction from tangent space to world space, aligned to N
 *
 * @param local The direction in tangent space
 * @param N The normal.
 * @return Vector3f Transformed direction.
 */
inline Vector3f toWorld(const Vector3f &local, const Vector3f &N) {
    /*  */
    Vector3f T, B;
    buildTBN(N, T, B);
    return local.x * T + local.y * B + local.z * N;
}

/* Cosine Weighted Hemisphere */

/**
 * @brief Sample a cosine weighted direction on the hemisphere aligned to N
 * pdf =  NdotL / PI
 *
 * @param N The normal to align the hemisphere with.
 * @return Vector3f The sampled direction.
 */
inline Vector3f sampleCosineHemisphere(const Vector3f &N) {
    float r1 = get_random_float();
    float r2 = get_random_float();
    float r = std::sqrt(r1);
    float phi = 2.f * M_PI * r2;
    return toWorld(
        Vector3f(r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.f, 1.f - r1))), N);
}
// PDF for a cosine-weighted hemisphere sample.
/**
 * @brief Calculate the PDF for a cosine-weighted hemisphere sample.
 *
 * @param NdotWi Dot product of normal and incoming ray (surface -> light)
 * @return float The PDF.
 */
inline float pdfCosineHemisphere(float NdotWi) { return std::max(0.f, NdotWi) / M_PI; }

/* GGX */

/**
 * @brief Importance sample a microfacet normal weighted by the GGX normal distribution function.
 * PDF = D_GGX(NdotH, alpha) * NdotH / (4 * VdotH)
 * @param N The surface shading normal
 * @param alpha the surface's material roughness squared (roughness^2)
 * @return Vector3f The sampled half vector in world space.
 */
inline Vector3f sampleGGX(const Vector3f &N, float alpha) {
    float r1 = get_random_float();
    float r2 = get_random_float();
    float alpha2 = alpha * alpha;

    float cosTheta2 = (1.f - r1) / (r1 * (alpha2 - 1.f) + 1.f);
    float cosTheta = std::sqrt(std::max(0.f, cosTheta2));
    float sinTheta = std::sqrt(std::max(0.f, 1.f - cosTheta2));
    float phi = 2.f * M_PI * r2;

    Vector3f H = toWorld(Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta), N);
    return dotProduct(H, N) < 0.f ? -H : H;
}

/**
 * @brief Evaljuate the GGX Normal Distribution Function.
 *
 * @param NdotH Dot product of surface normal N and half vector H
 * @param alpha Surface material's roughness squared (roughness^2)
 * @return float Microfacet density at this angle.
 */
inline float D_GGX(float NdotH, float alpha) {
    float alpha2 = alpha * alpha;
    float denom = NdotH * NdotH * (alpha2 - 1.f) + 1.f;
    return alpha2 / (M_PI * denom * denom);
}

// Smith GGX single-direction geometry term (G1).
// alpha = roughness^2

/**
 * @brief Calculates the Smith GGX single-drection geometry term (G1).
 * Calculates the term for a single direction.
 * V is the view direction (From surface to camera)
 * @param NdotV
 * @param alpha roughness^2
 * @return float
 */
inline float G1_GGX(float NdotV, float alpha) {
    float alpha2 = alpha * alpha;
    float denom = NdotV + std::sqrt(alpha2 + (1.f - alpha2) * NdotV * NdotV);
    return 2.f * NdotV / std::max(denom, 1e-6f);
}

/**
 * @brief Calculates the Smith Geometric term for GGX
 *
 * @param NdotWo Dot product of normal and outgoing ray (surface -> camera)
 * @param NdotWi dot product of normal and incoming ray (surface -> light)
 * @param alpha roughness ^2
 * @return float The Smith Geometric term.
 */
inline float G_Smith(float NdotWo, float NdotWi, float alpha) {
    return G1_GGX(NdotWo, alpha) * G1_GGX(NdotWi, alpha);
}

// Schlick Fresnel approximation.
// F0 = base reflectance at normal incidence.

/**
 * @brief Calculates the Shlick Fresnel approximation
 *
 * @param WoDotH Dot product of outgoing ray and half vector.
 * @param F0 Base reflectance at normal incidence angle.
 * @return Vector3f The fresnel approximation.
 */
inline Vector3f F_Schlick(float WoDotH, const Vector3f &F0) {
    float t = std::pow(std::max(0.f, 1.f - WoDotH), 5.f);
    return F0 + (Vector3f(1.f) - F0) * t;
}

/**
 * @brief Calculates the PDF for a GGX NDF sample.
 *  Half vector H is obtained from sampling NDF.
 * @param NdotH Dot product of normal and half vector
 * @param WoDotH Dot product of outgoing ray and half vector.
 * @param alpha roughness^2
 * @return float The PDF.
 */
inline float pdfGGX(float NdotH, float WoDotH, float alpha) {
    return D_GGX(NdotH, alpha) * NdotH / (4.f * std::max(WoDotH, 1e-4f));
}

/**
 * @brief Calculates the weighted PDF for a combination of diffuse (Lambert) and specular (GGX) PDFs
 *
 * @param NdotWi Dot product of normal and incoming ray.
 * @param NdotH Dot product of normal and GGX half vector.
 * @param WodotH Dot product of outgoing ray and GGX half vector.
 * @param alpha roughness^2
 * @param specularWeight metallic + (1 - metallic) * Ks
 * @return float
 */
inline float pdfMixed(float NdotWi, float NdotH, float WodotH, float alpha, float specularWeight) {
    float pd = pdfCosineHemisphere(NdotWi);
    float ps = pdfGGX(NdotH, WodotH, alpha);
    return specularWeight * ps + (1.f - specularWeight) * pd;
}

// ── Dielectric helpers ──────────────────────────────────────────────────── TODO: UPDATE DOCS

static Vector3f reflect(const Vector3f &I, const Vector3f &N) {
    return I - 2.f * dotProduct(I, N) * N;
}

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