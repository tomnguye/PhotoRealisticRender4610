#include "Integrator.hpp"
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

/**
 * @brief Direct samples a light in the scene.
 */
LightSample Integrator::sampleDirectLight(const Vector3f &hitPoint, const Vector3f &N) const {
    // Sample a point on an emitter.
    Intersection lightSample;
    float lightPdfArea;
    scene.sampleLight(lightSample, lightPdfArea);

    // Calculate geometry between hit point and light sample.
    Vector3f wi = (lightSample.coords - hitPoint).normalized();
    float dist = (lightSample.coords - hitPoint).norm();
    float cosAtLight = std::max(0.0f, dotProduct(-wi, lightSample.normal.normalized()));

    // Cull lights where ray hits back face.
    if (cosAtLight <= 0.0f)
        return {Vector3f(0), wi, 1.0f, false};

    // Convert area PDF to solid angle PDF.
    float pdfSolidAngle = lightPdfArea * dist * dist / cosAtLight;

    // Cast ray towards light and check for visibility.
    Ray shadowRay(hitPoint + wi * EPSILON, wi);
    Intersection shadowInter = scene.intersect(shadowRay);
    bool visible = shadowInter.happened && shadowInter.obj->hasEmit() &&
                   std::abs(shadowInter.tnear - (dist - EPSILON)) < EPSILON * dist;

    return {lightSample.material->m_emission, wi, pdfSolidAngle, visible};
}

/**
 * @brief Evaluates how much a direct light sample contributes to the render.
 * Includes multiple importance sampling.
 */
Vector3f Integrator::evalLightSample(const LightSample &light, const Vector3f &wo,
                                     const ShadingData &sd, Material *mat) const {
    if (!light.visible)
        return Vector3f(0);

    float cosAtSurface = std::max(0.f, dotProduct(light.dir, sd.N));
    Vector3f brdf = mat->eval(light.dir, wo, sd);
    float pdf = mat->pdf(light.dir, wo, sd);
    float misWeight = mis(light.pdf, pdf);
    return misWeight * light.emission * brdf * cosAtSurface / light.pdf;
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
 * @brief Evaluates how much an environment light sample contributes to the render.
 * Includes multiple importance sampling.
 */
Vector3f Integrator::evalEnvironmentSample(const LightSample sample, const Vector3f &wo,
                                           const ShadingData &sd, Material *mat) const {

    if (!sample.visible) {
        return Vector3f(0);
    }

    Vector3f dir = sample.dir;
    float cosTheta = std::max(0.f, dotProduct(dir, sd.N));
    Vector3f brdf = mat->eval(dir, wo, sd);
    float brdfPdf = mat->pdf(dir, wo, sd);
    float wEnv = mis(sample.pdf, brdfPdf);

    return wEnv * sample.emission * brdf * cosTheta / (sample.pdf + 1e-6f);
}

/**
 * @brief Evaluates how much a BRDF sample contributes to the render if it hits a light.
 * Includes multiple importance sampling.
 */
Vector3f Integrator::evalBRDFSample(const Vector3f &wi, float brdfPdf, const Vector3f &hitPoint,
                                    const Vector3f &wo, const ShadingData &sd, Material *mat,
                                    bool &hitLight, Intersection &nextInter) const {
    hitLight = false;

    if (brdfPdf < 1e-6f)
        return Vector3f(0);

    Vector3f brdf = mat->eval(wi, wo, sd);
    float cosTheta = std::max(0.f, dotProduct(wi, sd.N));

    Ray nextRay(hitPoint + wi * EPSILON, wi);
    nextInter = scene.intersect(nextRay);

    // No intersection so evaluate environment map hit.
    if (!nextInter.happened) {
        if (!scene.envMap.empty()) {
            Vector3f envL = scene.envMap.sample(wi);
            float envPdfSolidAngle = scene.envMap.importanceSamplePdf(wi);
            float wBrdf = mis(brdfPdf, envPdfSolidAngle);
            return wBrdf * envL * brdf * cosTheta / brdfPdf;
        }

        return Vector3f(0);
    }

    // Emissive object hit
    if (nextInter.obj->hasEmit()) {
        hitLight = true;
        float cosThetaLight = std::max(0.f, dotProduct(-wi, nextInter.normal.normalized()));
        float hitDist = (nextInter.coords - hitPoint).norm();
        float lightPdfArea = scene.pdfLight(nextInter);
        float lightPdfSolidAngle =
            lightPdfArea * hitDist * hitDist / std::max(cosThetaLight, 1e-4f);
        float wBrdf = mis(brdfPdf, lightPdfSolidAngle);
        return wBrdf * nextInter.material->m_emission * brdf * cosTheta / brdfPdf;
    }

    return Vector3f(0);
}

Vector3f Integrator::castRay(const Ray &ray) const {
    auto inter = scene.intersect(ray);

    if (!inter.happened) {
        if (!scene.envMap.empty())
            return scene.envMap.sample(ray.direction);
        return scene.backgroundColor;
    }

    if (inter.material->m_type == EMIT)
        return inter.material->m_emission;

    Vector3f radiance(0), throughput(1);
    Ray currentRay = ray;

    bool prevWasDelta = true;

    for (int bounce = 0; bounce < maxDepth; bounce++) {
        if (!inter.happened) {
            if (!scene.envMap.empty())
                radiance += throughput * scene.envMap.sample(currentRay.direction);
            break;
        }

        if (inter.obj->hasEmit()) {
            if (prevWasDelta) {
                radiance += throughput * inter.material->m_emission;
            }
            break;
        }

        Material *mat = inter.material;
        Vector3f hitPoint = inter.coords;

        // Flip geometric normal to face incoming ray.
        Vector3f geometricNormal =
            dotProduct(currentRay.direction, inter.normal) < 0 ? inter.normal : -inter.normal;

        if (mat->m_type == MIRROR) {
            Vector3f reflDir =
                currentRay.direction -
                2.f * dotProduct(currentRay.direction, geometricNormal) * geometricNormal;
            currentRay = Ray(hitPoint + reflDir * EPSILON, reflDir);
            inter = scene.intersect(currentRay);
            prevWasDelta = true;
            throughput = throughput * mat->baseColor;
        }

        if (mat->m_type == GLASS) {
            float kr = fresnel(currentRay.direction, inter.normal, mat->ior);
            bool doReflect = get_random_float() < kr;

            Vector3f nextDir;
            if (doReflect) {
                nextDir = reflect(currentRay.direction, geometricNormal).normalized();
            } else {
                nextDir = refract(currentRay.direction, inter.normal, mat->ior);
                if (nextDir.norm() < 1e-6f) {
                    nextDir = reflect(currentRay.direction, geometricNormal).normalized();
                }
            }

            throughput = throughput * mat->baseColor;
            currentRay = Ray(hitPoint + nextDir * EPSILON, nextDir);
            inter = scene.intersect(currentRay);
            prevWasDelta = true;
        }

        if (mat->m_type == DIFFUSE) {

            // Build shading data for hit.
            ShadingData sd;
            if (inter.hasTangent) {
                Vector3f T = normalize(inter.tangent);
                sd = mat->buildShadingData(inter.tcoords, geometricNormal, T,
                                           inter.tangentHandedness);
            } else {
                sd = mat->buildShadingData(inter.tcoords, geometricNormal);
            }

            // Use photon mapping for caustics.
            if (!scene.photon_map.caustic_map.empty() && prevWasDelta) {
                Vector3f caustic =
                    scene.photon_map.estimateIrradiance(hitPoint, sd.N) * sd.baseColor / M_PI;
                radiance += throughput * caustic;
            }
            prevWasDelta = false;

            Vector3f emissive = mat->evalEmissive(inter.tcoords);
            if (emissive.norm() > EPSILON) {
                radiance += throughput * emissive;
                break;
            }

            Vector3f wo = -currentRay.direction;
            BSDFSample bsdf = mat->sample(wo, sd);
            if (bsdf.pdf < 1e-6f)
                break;

            if (scene.totalEmitArea > 0.f) {
                LightSample light = sampleDirectLight(hitPoint, sd.N);
                radiance += throughput * evalLightSample(light, wo, sd, mat);
            }

            LightSample environmentSample = sampleEnvironmentMap(hitPoint);
            radiance += throughput * evalEnvironmentSample(environmentSample, wo, sd, mat);

            bool hitLight = false;
            Intersection nextInter;
            radiance += throughput * evalBRDFSample(bsdf.wi, bsdf.pdf, hitPoint, wo, sd, mat,
                                                    hitLight, nextInter);
            if (hitLight)
                break;

            float cosTheta = std::max(0.f, dotProduct(bsdf.wi, sd.N));
            throughput = throughput * bsdf.f * cosTheta / bsdf.pdf;

            currentRay = Ray(hitPoint + bsdf.wi * EPSILON, bsdf.wi);
            inter = nextInter;
        }

        float rrProb = std::min(rrThreshold, std::max({throughput.x, throughput.y, throughput.z}));
        if (get_random_float() > rrProb)
            break;
        throughput = throughput * (1.f / rrProb);

        // Clamp throughput based on lumiance.
        float lum = 0.2126f * throughput.x + 0.7152f * throughput.y + 0.0722f * throughput.z;
        if (lum > 20.f)
            throughput = throughput * (20.f / lum);
    }
    return radiance;
}