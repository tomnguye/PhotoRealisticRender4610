#pragma once

#include "BRDFTypes.hpp"
#include "Vector.hpp"
#include "global.hpp"

// ─── Frame / basis ────────────────────────────────────────────────────────────

// Build a tangent (T) and bitangent (B) orthonormal to N.
// Uses the Frisvad / Duff method — numerically stable across all normals.
inline void buildTBN(const Vector3f &N, Vector3f &T, Vector3f &B) {
    if (std::fabs(N.x) > std::fabs(N.y)) {
        float invLen = 1.f / std::sqrt(N.x * N.x + N.z * N.z);
        T = Vector3f(N.z * invLen, 0.f, -N.x * invLen);
    } else {
        float invLen = 1.f / std::sqrt(N.y * N.y + N.z * N.z);
        T = Vector3f(0.f, N.z * invLen, -N.y * invLen);
    }
    B = crossProduct(T, N);
}

// Transform a direction from local hemisphere space (z=up) to world space
// aligned around N.
inline Vector3f toWorld(const Vector3f &local, const Vector3f &N) {
    Vector3f T, B;
    buildTBN(N, T, B);
    return local.x * T + local.y * B + local.z * N;
}

// ─── Sampling ─────────────────────────────────────────────────────────────────

// Cosine-weighted hemisphere sample aligned to N.
// PDF = NdotL / PI  (use pdfCosineHemisphere below)
inline Vector3f sampleCosineHemisphere(const Vector3f &N) {
    float r1 = get_random_float();
    float r2 = get_random_float();
    float r = std::sqrt(r1);
    float phi = 2.f * M_PI * r2;
    return toWorld(Vector3f(r * std::cos(phi), r * std::sin(phi), std::sqrt(std::max(0.f, 1.f - r1))), N);
}

// GGX Visible Normal Distribution Function (VNDF) importance sample.
//
// Samples the half-vector H proportional to D(H) * dot(wo, H), i.e. the
// distribution of normals *visible* from the outgoing direction wo.  This
// eliminates back-facing microfacet samples that NDF sampling can produce,
// which are the primary cause of fireflies (near-zero VdotH -> exploding pdf).
//
// Algorithm: Heitz 2018 "Sampling the GGX Distribution of Visible Normals".
// alpha = roughness^2  (pass pre-squared)
// PDF in solid angle of wi = D_GGX(NdotH)*G1(NdotV)*VdotH / (NdotV)
//                            / (4*VdotH)  -- see pdfGGXVNDF below
inline Vector3f sampleGGXVNDF(const Vector3f &wo, const Vector3f &N, float alpha) {
    // Build a local frame with N as Z axis
    Vector3f T, B;
    buildTBN(N, T, B);

    // Transform wo into local space (z = NdotV)
    Vector3f woLocal(dotProduct(wo, T), dotProduct(wo, B), dotProduct(wo, N));

    // Stretch wo into the hemisphere configuration
    Vector3f woStretched = normalize(Vector3f(alpha * woLocal.x, alpha * woLocal.y, woLocal.z));

    // Build an orthonormal basis around woStretched
    Vector3f T1, T2;
    if (woStretched.z < 0.9999f) {
        T1 = normalize(crossProduct(Vector3f(0.f, 0.f, 1.f), woStretched));
    } else {
        T1 = Vector3f(1.f, 0.f, 0.f);
    }
    T2 = crossProduct(woStretched, T1);

    // Sample a point on the unit disk, biased toward the upper hemisphere
    float r = std::sqrt(get_random_float());
    float phi = 2.f * M_PI * get_random_float();
    float t1 = r * std::cos(phi);
    float t2 = r * std::sin(phi);
    float s = 0.5f * (1.f + woStretched.z);
    t2 = (1.f - s) * std::sqrt(std::max(0.f, 1.f - t1 * t1)) + s * t2;

    // Reproject onto hemisphere, unstretch to recover the microfacet normal
    Vector3f Hn = t1 * T1 + t2 * T2 + std::sqrt(std::max(0.f, 1.f - t1 * t1 - t2 * t2)) * woStretched;
    Vector3f HLocal(alpha * Hn.x, alpha * Hn.y, std::max(0.f, Hn.z));

    // Transform back to world space
    return normalize(HLocal.x * T + HLocal.y * B + HLocal.z * N);
}

// ─── BRDF terms ───────────────────────────────────────────────────────────────

// GGX / Trowbridge-Reitz Normal Distribution Function.
// alpha = roughness^2
inline float D_GGX(float NdotH, float alpha) {
    float alpha2 = alpha * alpha;
    float denom = NdotH * NdotH * (alpha2 - 1.f) + 1.f;
    return alpha2 / (M_PI * denom * denom);
}

// Smith GGX single-direction geometry term (G1).
// alpha = roughness^2
inline float G1_GGX(float NdotV, float alpha) {
    float alpha2 = alpha * alpha;
    float denom = NdotV + std::sqrt(alpha2 + (1.f - alpha2) * NdotV * NdotV);
    return 2.f * NdotV / std::max(denom, 1e-6f);
}

// Height-correlated Smith G2.
//
// The uncorrelated form G1(V)*G1(L) overestimates shadowing — the
// height-correlated form accounts for the fact that rays hitting a tall
// microfacet from one side are also likely to be blocked from the other.
// This is what PBRT-v4 and Mitsuba 3 both use.
// alpha = roughness^2
inline float G_Smith(float NdotV, float NdotL, float alpha) {
    float alpha2 = alpha * alpha;
    float lambdaV = (-1.f + std::sqrt(alpha2 + (1.f - alpha2) * NdotV * NdotV)) / std::max(2.f * NdotV, 1e-6f);
    float lambdaL = (-1.f + std::sqrt(alpha2 + (1.f - alpha2) * NdotL * NdotL)) / std::max(2.f * NdotL, 1e-6f);
    return 1.f / (1.f + lambdaV + lambdaL);
}

// Schlick Fresnel approximation.
// F0 = base reflectance at normal incidence.
inline Vector3f F_Schlick(float VdotH, const Vector3f &F0) {
    float t = std::pow(std::max(0.f, 1.f - VdotH), 5.f);
    return F0 + (Vector3f(1.f) - F0) * t;
}

// ─── PDFs ─────────────────────────────────────────────────────────────────────

// PDF for a cosine-weighted hemisphere sample.
inline float pdfCosineHemisphere(float NdotL) { return std::max(0.f, NdotL) / M_PI; }

// PDF for a direction wi sampled via VNDF.
//
// The VNDF samples H with density D(H)*G1(NdotV)*VdotH / NdotV.
// Converting from half-vector to wi solid angle introduces the Jacobian 1/(4*VdotH),
// giving:  pdf(wi) = D(NdotH) * G1(NdotV) / (4 * NdotV)
//
// This is always well-conditioned: G1 -> 0 as NdotV -> 0, killing the 1/NdotV pole,
// and VdotH is guaranteed positive by the VNDF construction.
// alpha = roughness^2
inline float pdfGGXVNDF(float NdotH, float NdotV, float VdotH, float alpha) {
    return D_GGX(NdotH, alpha) * G1_GGX(NdotV, alpha) / std::max(4.f * NdotV, 1e-6f);
}

// Mixed PDF: weighted combination of VNDF specular and cosine diffuse.
// specularWeight = metallic + (1 - metallic) * Ks
inline float pdfMixed(float NdotL, float NdotH, float NdotV, float VdotH, float alpha, float specularWeight) {
    float pd = pdfCosineHemisphere(NdotL);
    float ps = pdfGGXVNDF(NdotH, NdotV, VdotH, alpha);
    return specularWeight * ps + (1.f - specularWeight) * pd;
}