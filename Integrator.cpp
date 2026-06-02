#include "Integrator.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <algorithm>
#include <cmath>

LightSample Integrator::sampleDirectLight(const Vector3f &hitPoint, const Vector3f &N) const {
    Intersection lightSample;
    float lightPdfArea;
    scene.sampleLight(lightSample, lightPdfArea);

    Vector3f toLight = lightSample.coords - hitPoint;
    float dist2 = dotProduct(toLight, toLight);
    float dist = std::sqrt(dist2);
    Vector3f wi = toLight / dist;

    float cosAtLight = std::max(0.f, dotProduct(-wi, lightSample.normal.normalized()));
    if (cosAtLight <= 0.f)
        return {Vector3f(0), wi, 1.f, false};

    float pdfSolidAngle = lightPdfArea * dist2 / cosAtLight;

    // attempt at shadow toggling
    bool visible;
    if (!shadowsEnabled) {
        visible = true;
    } else {
        // intersectP is cheaper than intersect — returns on first hit.
        // tMax set just short of the light to avoid self-intersection on it.
        visible = !scene.intersectP(Ray(hitPoint + wi * EPSILON, wi), dist * (1.f - 1e-3f));
    }

    // Ray shadowRay(hitPoint + N * EPSILON, wi);
    // bool visible = !scene.intersectP(shadowRay, dist - 2.f * EPSILON);

    return {lightSample.material->m_emission, wi, pdfSolidAngle, visible};
}

/**
 * @brief Direct samples light coming from the environment map.
 */
LightSample Integrator::sampleEnvironmentMap(const Vector3f &hitPoint, const Vector3f &N) const {
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

    if (shadowsEnabled) {
        Ray shadowRay(hitPoint + N * EPSILON, sampleDir);
        if (scene.intersectP(shadowRay)) {
            lightSample.visible = false;
            return lightSample;
        }
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
 */
Vector3f Integrator::evalLightSample(const LightSample &light, const Vector3f &wo,
                                     const ShadingData &sd, Material *mat) const {
    if (!light.visible)
        return Vector3f(0.f);

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

    Vector3f brdf = mat->eval(sample.dir, wo, sd, LOBE_ALL);
    float brdfPdf = mat->pdf(sample.dir, wo, sd, LOBE_ALL);
    float wEnv = mis(sample.pdf, brdfPdf);
    return wEnv * sample.emission * brdf / (sample.pdf + 1e-6f);
}

Vector3f Integrator::castRay(const Ray &ray) const {
    auto clampIndirect = [this](const Vector3f &c) -> Vector3f {
        if (indirectClamp <= 0.f)
            return c;
        // float lum = 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
        float lum = (c.x + c.y + c.z) / 3;
        return (lum > indirectClamp) ? c * (indirectClamp / lum) : c;
    };

    Vector3f L(0.f);
    Vector3f beta(1.f);
    Ray currentRay = ray;
    bool specularBounce = false;

    Intersection inter = scene.intersect(currentRay);

    for (int bounce = 0; bounce < maxDepth; bounce++) {

        if (bounce == 0 || specularBounce) {
            if (!inter.happened) {
                Vector3f bg = !scene.envMap.empty() ? scene.envMap.sample(currentRay.direction)
                                                    : scene.backgroundColor;
                Vector3f contrib = beta * bg;
                L += (bounce > 0) ? clampIndirect(contrib) : contrib;
            } else if (inter.material->isEmissive()) {
                bool frontFace = dotProduct(-currentRay.direction, inter.normal.normalized()) > 0.f;
                if (frontFace) {
                    Vector3f contrib = beta * inter.material->m_emission;
                    L += (bounce > 0) ? clampIndirect(contrib) : contrib;
                }
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

            BRDFSample brdf = mat->sample(-currentRay.direction, sd);
            if (brdf.pdf < 1e-6f)
                break;

            beta = beta * brdf.f / brdf.pdf;

            bool isTransmission = dotProduct(brdf.wi, geoNormal) < 0.f;
            Vector3f offset = isTransmission ? -geoNormal : geoNormal;

            currentRay = Ray(hitPoint + offset * EPSILON, brdf.wi);
            inter = scene.intersect(currentRay);

            if (isTransmission) {
                beta = beta * sd.baseColor;
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

            // Direct lights
            if (scene.totalEmitArea > 0.f) {
                LightSample light = sampleDirectLight(hitPoint, sd.N);
                Vector3f contrib = beta * evalLightSample(light, wo, sd, dm);
                L += isIndirect ? clampIndirect(contrib) : contrib;
            }

            // Direct env map
            if (!scene.envMap.empty()) {
                LightSample envSample = sampleEnvironmentMap(hitPoint, sd.N);
                Vector3f contrib = beta * evalEnvironmentSample(envSample, wo, sd, dm);
                L += isIndirect ? clampIndirect(contrib) : contrib;
            }

            BRDFSample brdf = dm->sample(wo, sd);
            if (brdf.pdf < 1e-6f)
                break;

            Vector3f betaScale = brdf.f / brdf.pdf;

            Vector3f offset = dotProduct(brdf.wi, sd.N) >= 0.f ? sd.N : -sd.N;
            currentRay = Ray(hitPoint + offset * EPSILON, brdf.wi);
            inter = scene.intersect(currentRay);

            if (!inter.happened) {
                if (!scene.envMap.empty()) {
                    Vector3f envL = scene.envMap.sample(brdf.wi);
                    float envPdf = scene.envMap.importanceSamplePdf(brdf.wi);
                    float wBrdf = mis(brdf.pdf, envPdf);
                    Vector3f c = beta * betaScale * wBrdf * envL;
                    L += isIndirect ? clampIndirect(c) : c;
                } else {
                    Vector3f c = beta * betaScale * scene.backgroundColor;
                    L += isIndirect ? clampIndirect(c) : c;
                }
                break;
            }

            if (inter.material->isEmissive()) {
                float cosThetaLight = dotProduct(-brdf.wi, inter.normal.normalized());
                if (cosThetaLight <= 0.f)
                    break;
                Vector3f d = inter.coords - hitPoint;
                float hitDist2 = dotProduct(d, d);
                float lightPdfArea = scene.pdfLight(inter);
                float lightPdfSA = lightPdfArea * hitDist2 / std::max(cosThetaLight, 1e-4f);
                float wBrdf = mis(brdf.pdf, lightPdfSA);
                Vector3f c = beta * betaScale * wBrdf * inter.material->m_emission;
                L += isIndirect ? clampIndirect(c) : c;
                break;
            }

            beta = beta * betaScale;

            if (!std::isfinite(beta.x) || !std::isfinite(beta.y) || !std::isfinite(beta.z))
                break;
        }

        // russian roulette
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