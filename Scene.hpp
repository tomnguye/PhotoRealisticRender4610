#pragma once

#include "BVH.hpp"
#include "Camera.hpp"
#include "EnvMap.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "Photon.hpp"
#include "PointLight.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include <vector>

class Scene {
public:
    int width = 1280;
    int height = 960;
    double fov = 40;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 30;
    float RussianRoulette = 0.95f;
    int spp = 1024;
    float adaptiveThreshold = 0.05;
    float exposure = 0.18f;

    Scene(int w, int h) : width(w), height(h) {}

    void Add(Object *object) { objects.push_back(object); }

    void build() {
        buildBVH();
        camera.init(width, height);
    }

    const std::vector<Object *> &get_objects() const { return objects; }

    Intersection intersect(const Ray &ray) const;
    void buildBVH();
    Vector3f castRay(const Ray &ray, int depth) const;
    void sampleLight(Intersection &pos, float &pdf) const;
    float pdfLight(const Intersection &lightInter) const;
    void buildPhotonMaps(int num_photons);
    Vector3f shadeDiffuse(const Ray &ray, const Intersection &firstInter, int depth) const;
    Vector3f shadeGlass(const Ray &ray, const Intersection &inter, int depth) const;
    Vector3f shadeMirror(const Ray &ray, const Intersection &inter, int depth) const;
    bool trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index,
               Object **hitObject);

    std::vector<Object *> objects;
    BVHAccel *bvh;
    EnvMap envMap;
    Camera camera;
    float totalEmitArea = 0.f;
    PhotonMap photon_map;

    // ── Direct light sample ───────────────────────────────────────────────────

    struct DirectSample {
        Vector3f L;
        Vector3f dir;
        float pdfSolidAngle;
        float pdfArea;
        float cosThetaLight;
        float dist;
        bool visible;
    };

    DirectSample sampleDirectLight(const Vector3f &hitPoint, const Vector3f &N) const {
        Intersection lightSample;
        float lightPdfArea;
        sampleLight(lightSample, lightPdfArea);

        Vector3f toLight = (lightSample.coords - hitPoint).normalized();
        float dist = (lightSample.coords - hitPoint).norm();
        float cosThetaLight = std::max(0.f, dotProduct(-toLight, lightSample.normal.normalized()));

        Ray shadowRay(hitPoint + toLight * EPSILON, toLight);
        Intersection shadowInter = intersect(shadowRay);
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
        if (envMap.empty())
            return Vector3f(0);

        float envPdf;
        Vector3f envDir = envMap.importanceSample(envPdf);
        if (envPdf < 1e-6f)
            return Vector3f(0);

        Ray shadowRay(hitPoint + envDir * EPSILON, envDir);
        Intersection shadowInter = intersect(shadowRay);
        if (shadowInter.happened)
            return Vector3f(0);

        float cosTheta = std::max(0.f, dotProduct(envDir, sd.N));
        Vector3f brdf = mat->eval(envDir, wo, sd);
        float brdfPdf = mat->pdf(envDir, wo, sd);
        float wEnv = mis(envPdf, brdfPdf);
        Vector3f envL = envMap.sample(envDir);

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
        nextInter = intersect(nextRay);

        // Emissive geometry hit
        if (nextInter.happened && nextInter.obj->hasEmit()) {
            hitLight = true;
            float cosThetaLight = std::max(0.f, dotProduct(-wi, nextInter.normal.normalized()));
            float hitDist = (nextInter.coords - hitPoint).norm();
            float lightPdfArea = pdfLight(nextInter);
            float lightPdfSolidAngle =
                lightPdfArea * hitDist * hitDist / std::max(cosThetaLight, 1e-4f);
            float wBrdf = mis(brdfPdf, lightPdfSolidAngle);
            return wBrdf * nextInter.material->m_emission * brdf * cosTheta / brdfPdf;
        }

        // Miss — sample environment
        if (!nextInter.happened && !envMap.empty()) {
            Vector3f envL = envMap.sample(wi);
            float envPdfSolidAngle = envMap.importanceSamplePdf(wi);
            float wBrdf = mis(brdfPdf, envPdfSolidAngle);
            return wBrdf * envL * brdf * cosTheta / brdfPdf;
        }

        return Vector3f(0);
    }

    // ── Dielectric helpers ────────────────────────────────────────────────────

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
};