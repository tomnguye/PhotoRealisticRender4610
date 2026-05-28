#include "Integrator.hpp"
#include "Volume.hpp"
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif
#include "Intersection.hpp"
#include "Ray.hpp"

/**
 * @brief Direct samples a light in the scene.
 */
LightSample Integrator::sampleDirectLight(const Vector3f &hitPoint, const Vector3f &N) const {
    Intersection lightSample;
    float lightPdfArea;
    scene.sampleLight(lightSample, lightPdfArea);

    Vector3f wi = (lightSample.coords - hitPoint).normalized();
    float dist = (lightSample.coords - hitPoint).norm();
    float cosAtLight = std::max(0.0f, dotProduct(-wi, lightSample.normal.normalized()));

    if (cosAtLight <= 0.0f)
        return {Vector3f(0), wi, 1.0f, false};

    float pdfSolidAngle = lightPdfArea * dist * dist / cosAtLight;

    Ray shadowRay(hitPoint + wi * EPSILON, wi);
    Intersection shadowInter = scene.intersect(shadowRay);
    bool visible = shadowInter.happened && shadowInter.obj->hasEmit() &&
                   std::abs(shadowInter.tnear - (dist - EPSILON)) < EPSILON * dist;

    return {lightSample.material->m_emission, wi, pdfSolidAngle, visible};
}

/**
 * @brief Direct samples light coming from the environment map.
 */
LightSample Integrator::sampleEnvironmentMap(const Vector3f &hitPoint) const {
    LightSample lightSample;
    if (scene.envMap.empty()) {
        lightSample.visible = false;
        return lightSample;
    }

    float pdf;
    Vector3f sampleDir = scene.envMap.importanceSample(pdf);

    if (pdf < 1e-6f) {
        lightSample.visible = false;
        return lightSample;
    }

    Ray shadowRay(hitPoint + sampleDir * EPSILON, sampleDir);
    Intersection intersection = scene.intersect(shadowRay);
    if (intersection.happened) {
        lightSample.visible = false;
        return lightSample;
    }

    lightSample.dir = sampleDir;
    lightSample.emission = scene.envMap.sample(sampleDir);
    lightSample.pdf = pdf;
    lightSample.visible = true;

    return lightSample;
}

/**
 * @brief Evaluates how much a direct light sample contributes to the render.
 * Includes multiple importance sampling.
 * NOTE: eval() returns f(wi,wo)*NdotWi already, so we do NOT multiply cosine here.
 */
Vector3f Integrator::evalLightSample(const LightSample &light, const Vector3f &wo,
                                     const ShadingData &sd, Material *mat) const {
    if (!light.visible)
        return Vector3f(0.f);

    // LOBE_ALL: direct light can come from any direction, evaluate both lobes.
    Vector3f brdf = mat->eval(light.dir, wo, sd, LOBE_ALL);
    float brdfPdf = mat->pdf(light.dir, wo, sd, LOBE_ALL);
    float misWeight = mis(light.pdf, brdfPdf);
    return misWeight * light.emission * brdf / light.pdf;
}

/**
 * @brief Evaluates how much an environment light sample contributes to the render.
 * Includes multiple importance sampling.
 */
Vector3f Integrator::evalEnvironmentSample(const LightSample &sample, const Vector3f &wo,
                                           const ShadingData &sd, Material *mat) const {
    if (!sample.visible)
        return Vector3f(0.f);

    // LOBE_ALL: env light can come from any direction, evaluate both lobes.
    Vector3f brdf = mat->eval(sample.dir, wo, sd, LOBE_ALL);
    float brdfPdf = mat->pdf(sample.dir, wo, sd, LOBE_ALL);
    float wEnv = mis(sample.pdf, brdfPdf);
    return wEnv * sample.emission * brdf / (sample.pdf + 1e-6f);
}

Vector3f Integrator::castRay(const Ray &ray) const {
    auto clampIndirect = [this](const Vector3f &c) -> Vector3f {
        if (indirectClamp <= 0.f)
            return c;
        float maxComp = std::max({c.x, c.y, c.z});
        return (maxComp > indirectClamp) ? Vector3f(indirectClamp) : c;
    };

    Vector3f L(0.f);
    Vector3f beta(1.f);
    Ray currentRay = ray;
    bool specularBounce = false;

    for (int bounce = 0;; ++bounce) {

        auto inter = scene.intersect(currentRay);

        if (bounce == 0 || specularBounce) {
            if (!inter.happened) {
                Vector3f bg = !scene.envMap.empty() ? scene.envMap.sample(currentRay.direction)
                                                    : scene.backgroundColor;
                Vector3f contrib = beta * bg;
                L += (bounce > 0) ? clampIndirect(contrib) : contrib;
            } else if (inter.material->isEmissive()) {
                Vector3f contrib = beta * inter.material->m_emission;
                L += (bounce > 0) ? clampIndirect(contrib) : contrib;
            }
        }

        if (!inter.happened || bounce >= maxDepth)
            break;

        Material *mat = inter.material;
        Vector3f hitPoint = inter.coords;
        Vector3f geoNormal = inter.normal;

        if (mat->isDelta()) {
            ShadingData sd = inter.hasTangent ? mat->buildShadingData(inter.tcoords, geoNormal,
                                                                      normalize(inter.tangent),
                                                                      inter.tangentHandedness)
                                              : mat->buildShadingData(inter.tcoords, geoNormal);

            BSDFSample bsdf = mat->sample(-currentRay.direction, sd);
            if (bsdf.pdf < 1e-6f)
                break;

            beta = beta * bsdf.f / bsdf.pdf;

            bool isTransmission = dotProduct(bsdf.wi, geoNormal) < 0.f;
            Vector3f offset = isTransmission ? -geoNormal : geoNormal;

            if (isTransmission) {
                currentRay = Ray(hitPoint + offset * EPSILON, bsdf.wi);
                auto nextInter = scene.intersect(currentRay);

                float dist = (nextInter.coords - hitPoint).norm();
                Vector3f tint = sd.baseColor;
                Vector3f absorb(-std::log(std::max(tint.x, 1e-6f)),
                                -std::log(std::max(tint.y, 1e-6f)),
                                -std::log(std::max(tint.z, 1e-6f)));
                // beta = beta * Vector3f(std::exp(-absorb.x * dist), std::exp(-absorb.y * dist),
                //                        std::exp(-absorb.z * dist));
                beta = beta * sd.baseColor;
                inter = nextInter;
            } else {
                currentRay = Ray(hitPoint + offset * EPSILON, bsdf.wi);
                inter = scene.intersect(currentRay);
            }

            specularBounce = true;
            continue;
        }

        {
            specularBounce = false;
            DiffuseMaterial *dm = static_cast<DiffuseMaterial *>(mat);

            Vector3f shadingNormal =
                dotProduct(currentRay.direction, geoNormal) < 0.f ? geoNormal : -geoNormal;

            ShadingData sd = inter.hasTangent ? dm->buildShadingData(inter.tcoords, shadingNormal,
                                                                     normalize(inter.tangent),
                                                                     inter.tangentHandedness)
                                              : dm->buildShadingData(inter.tcoords, shadingNormal);

            Vector3f wo = -currentRay.direction;
            bool isIndirect = (bounce > 0);

            if (scene.totalEmitArea > 0.f) {
                LightSample light = sampleDirectLight(hitPoint, sd.N);
                Vector3f contrib = beta * evalLightSample(light, wo, sd, dm);
                L += isIndirect ? clampIndirect(contrib) : contrib;
            }

            if (!scene.envMap.empty()) {
                LightSample envSample = sampleEnvironmentMap(hitPoint);
                Vector3f contrib = beta * evalEnvironmentSample(envSample, wo, sd, dm);
                L += isIndirect ? clampIndirect(contrib) : contrib;
            }

            BSDFSample bsdf = dm->sample(wo, sd);
            if (bsdf.pdf < 1e-6f)
                break;

            Vector3f betaScale = bsdf.f / bsdf.pdf;

            Vector3f offset = dotProduct(bsdf.wi, sd.N) >= 0.f ? sd.N : -sd.N;
            currentRay = Ray(hitPoint + offset * EPSILON, bsdf.wi);
            inter = scene.intersect(currentRay);

            if (!inter.happened) {
                if (!scene.envMap.empty()) {
                    Vector3f envL = scene.envMap.sample(bsdf.wi);
                    float envPdf = scene.envMap.importanceSamplePdf(bsdf.wi);
                    float wBrdf = mis(bsdf.pdf, envPdf);
                    Vector3f c = beta * betaScale * wBrdf * envL;
                    L += isIndirect ? clampIndirect(c) : c;
                } else {
                    Vector3f c = beta * betaScale * scene.backgroundColor;
                    L += isIndirect ? clampIndirect(c) : c;
                }
                break;
            }

            if (inter.material->isEmissive()) {
                float cosThetaLight =
                    std::max(0.f, dotProduct(-bsdf.wi, inter.normal.normalized()));
                float hitDist = (inter.coords - hitPoint).norm();
                float lightPdfArea = scene.pdfLight(inter);
                float lightPdfSA =
                    lightPdfArea * hitDist * hitDist / std::max(cosThetaLight, 1e-4f);
                float wBrdf = mis(bsdf.pdf, lightPdfSA);
                Vector3f c = beta * betaScale * wBrdf * inter.material->m_emission;
                L += isIndirect ? clampIndirect(c) : c;
                break;
            }

            beta = beta * betaScale;

            if (!std::isfinite(beta.x) || !std::isfinite(beta.y) || !std::isfinite(beta.z))
                break;
        }

        if (bounce >= 3) {
            float maxComp = std::max({beta.x, beta.y, beta.z});
            float q = std::max(0.05f, 1.f - maxComp);
            if (get_random_float() < q)
                break;
            beta = beta * (1.f / (1.f - q));
        }
    }

    if (!std::isfinite(L.x) || !std::isfinite(L.y) || !std::isfinite(L.z))
        return Vector3f(0.f);

    return L;
}