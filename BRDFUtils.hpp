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

// GGX NDF importance sample — returns a microfacet half-vector H.
// Reflect wo around H to get wi.
// alpha = roughness^2  (pass pre-squared)
// PDF = D_GGX(NdotH, alpha) * NdotH / (4 * VdotH)  (use pdfGGX below)
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

// Height-correlated Smith G2 = G1(NdotV) * G1(NdotL).
// alpha = roughness^2
inline float G_Smith(float NdotV, float NdotL, float alpha) { return G1_GGX(NdotV, alpha) * G1_GGX(NdotL, alpha); }

// Schlick Fresnel approximation.
// F0 = base reflectance at normal incidence.
inline Vector3f F_Schlick(float VdotH, const Vector3f &F0) {
    float t = std::pow(std::max(0.f, 1.f - VdotH), 5.f);
    return F0 + (Vector3f(1.f) - F0) * t;
}

// ─── PDFs ─────────────────────────────────────────────────────────────────────

// PDF for a cosine-weighted hemisphere sample.
inline float pdfCosineHemisphere(float NdotL) { return std::max(0.f, NdotL) / M_PI; }

// PDF for a GGX NDF sample converted to a wi direction.
// NdotH and VdotH come from the half-vector between wi and wo.
// alpha = roughness^2
inline float pdfGGX(float NdotH, float VdotH, float alpha) { return D_GGX(NdotH, alpha) * NdotH / (4.f * std::max(VdotH, 1e-4f)); }

// Mixed PDF: weighted combination of diffuse and specular PDFs.
// specularWeight = metallic + (1 - metallic) * Ks
inline float pdfMixed(float NdotL, float NdotH, float VdotH, float alpha, float specularWeight) {
    float pd = pdfCosineHemisphere(NdotL);
    float ps = pdfGGX(NdotH, VdotH, alpha);
    return specularWeight * ps + (1.f - specularWeight) * pd;
}