#pragma once

#include "BVH.hpp"
#include "Camera.hpp"
#include "EnvMap.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "Photon.hpp"
#include "PointLight.hpp"
#include "Ray.hpp"
#include "Scene.hpp"
#include "Vector.hpp"
#include <vector>

struct DirectSample {
    Vector3f L;
    Vector3f dir;
    float pdfSolidAngle;
    float pdfArea;
    float cosThetaLight;
    float dist;
    bool visible;
};

class Integrator {

public:
    Integrator(const Scene &scene_, int maxDepth_, float rrThreshold_)
        : scene(scene_), maxDepth(maxDepth_), rrThreshold(rrThreshold_) {}

    Vector3f castRay(const Ray &ray) const;

private:
    const Scene &scene;
    int maxDepth;
    float rrThreshold;
    Vector3f shadeDiffuse(const Ray &ray, const Intersection &firstInter, int depth) const;

    // ── Direct light sample ───────────────────────────────────────────────────

    DirectSample sampleDirectLight(const Vector3f &hitPoint, const Vector3f &N) const {
        Intersection lightSample;
        float lightPdfArea;
        scene.sampleLight(lightSample, lightPdfArea);

        Vector3f toLight = (lightSample.coords - hitPoint).normalized();
        float dist = (lightSample.coords - hitPoint).norm();
        float cosThetaLight = std::max(0.f, dotProduct(-toLight, lightSample.normal.normalized()));

        Ray shadowRay(hitPoint + toLight * EPSILON, toLight);
        Intersection shadowInter = scene.intersect(shadowRay);
        bool visible = shadowInter.happened && shadowInter.obj->hasEmit() &&
                       std::abs(shadowInter.tnear - (dist - EPSILON)) < 1e-2f * dist;

        float pdfSolidAngle = lightPdfArea * dist * dist / std::max(cosThetaLight, 1e-4f);
        if (cosThetaLight <= 0.f) {
            visible = false; /* or return early */
        }

        return {lightSample.material->m_emission,
                toLight,
                pdfSolidAngle,
                lightPdfArea,
                cosThetaLight,
                dist,
                visible};
    }

    // ── MIS weight (power heuristic, beta=2) ──────────────────────────────────

    inline float mis(float a, float b) const { return (a * a) / (a * a + b * b + 1e-6f); }

    // ── NEE: environment map ──────────────────────────────────────────────────

    Vector3f evalEnvSampleAt(const Vector3f &hitPoint, const Vector3f &wo, const ShadingData &sd,
                             Material *mat) const {
        if (scene.envMap.empty())
            return Vector3f(0);

        float envPdf;
        Vector3f envDir = scene.envMap.importanceSample(envPdf);
        if (envPdf < 1e-6f)
            return Vector3f(0);

        Ray shadowRay(hitPoint + envDir * EPSILON, envDir);
        Intersection shadowInter = scene.intersect(shadowRay);
        if (shadowInter.happened)
            return Vector3f(0);

        float cosTheta = std::max(0.f, dotProduct(envDir, sd.N));
        Vector3f brdf = mat->eval(envDir, wo, sd);
        float brdfPdf = mat->pdf(envDir, wo, sd);
        float wEnv = mis(envPdf, brdfPdf);
        Vector3f envL = scene.envMap.sample(envDir);

        return wEnv * envL * brdf * cosTheta / (envPdf + 1e-6f);
    }

    // ── NEE: geometry light ───────────────────────────────────────────────────

    Vector3f evalLightSample(const DirectSample &light, const Vector3f &wo, const ShadingData &sd,
                             Material *mat) const {
        if (!light.visible)
            return Vector3f(0);

        float cosThetaSurface = std::max(0.f, dotProduct(light.dir, sd.N));
        Vector3f brdf = mat->eval(light.dir, wo, sd);
        float brdfPdf = mat->pdf(light.dir, wo, sd);
        float wLight = mis(light.pdfSolidAngle, brdfPdf);

        // pdfSolidAngle already incorporates the area->solid-angle conversion
        // (dist² / cosThetaLight), so just divide by it directly.
        return wLight * light.L * brdf * cosThetaSurface / std::max(light.pdfSolidAngle, 1e-6f);
    }

    // ── BRDF sample + next intersection ──────────────────────────────────────
    //
    // Fires the BRDF sample ray, returns MIS-weighted radiance contribution,
    // and hands back the next intersection so the caller avoids a second BVH
    // traversal.

    Vector3f evalBRDFSample(const Vector3f &wi, float brdfPdf, const Vector3f &hitPoint,
                            const Vector3f &wo, const ShadingData &sd, Material *mat,
                            bool &hitLight, Intersection &nextInter) const {
        hitLight = false;

        if (brdfPdf < 1e-6f)
            return Vector3f(0);
        Vector3f brdf = mat->eval(wi, wo, sd);
        float cosTheta = std::max(0.f, dotProduct(wi, sd.N));

        Ray nextRay(hitPoint + wi * EPSILON, wi);
        nextInter = scene.intersect(nextRay);

        // Emissive geometry hit
        if (nextInter.happened && nextInter.obj->hasEmit()) {
            hitLight = true;
            float cosThetaLight = std::max(0.f, dotProduct(-wi, nextInter.normal.normalized()));
            float hitDist = (nextInter.coords - hitPoint).norm();
            float lightPdfArea = scene.pdfLight(nextInter);
            float lightPdfSolidAngle =
                lightPdfArea * hitDist * hitDist / std::max(cosThetaLight, 1e-4f);
            float wBrdf = mis(brdfPdf, lightPdfSolidAngle);
            return wBrdf * nextInter.material->m_emission * brdf * cosTheta / brdfPdf;
        }

        // Miss — sample environment
        if (!nextInter.happened && !scene.envMap.empty()) {
            Vector3f envL = scene.envMap.sample(wi);
            float envPdfSolidAngle = scene.envMap.importanceSamplePdf(wi);
            float wBrdf = mis(brdfPdf, envPdfSolidAngle);
            return wBrdf * envL * brdf * cosTheta / brdfPdf;
        }

        return Vector3f(0);
    }
};
