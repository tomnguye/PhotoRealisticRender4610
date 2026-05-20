
#include "Integrator.hpp"
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

// ─────────────────────────────────────────────
//  Shading helpers
// ─────────────────────────────────────────────

// Helper: sample mirror direction inline, no recursion
static Ray mirrorRay(const Vector3f &dir, const Vector3f &N, const Vector3f &hitPoint, float eps) {
    Vector3f r = dir - 2.f * dotProduct(dir, N) * N;
    return Ray(hitPoint + r * eps, r);
}

Vector3f Integrator::shadeDiffuse(const Ray &ray, const Intersection &firstInter, int depth) const {
    Vector3f radiance(0), throughput(1);
    Ray currentRay = ray;
    auto inter = firstInter; // use caller-supplied intersection for the first bounce

    // Camera ray has no prior NEE, so treat it like a delta bounce —
    // any emitter hit on the first bounce must be counted directly.
    bool prevWasDelta = true;

    for (int bounce = depth; bounce < maxDepth; bounce++) {
        // ── Miss ──────────────────────────────────────────────────────────────
        if (!inter.happened) {
            if (!scene.envMap.empty())
                radiance += throughput * scene.envMap.sample(currentRay.direction);
            break;
        }

        if (inter.obj->hasEmit()) {
            if (prevWasDelta || true) {
                radiance += throughput * inter.material->m_emission;
            }
            break;
        }

        Material *mat = inter.material;
        Vector3f hitPoint = inter.coords;

        // ── Flip geometric normal to face incoming ray ────────────────────────
        Vector3f geoN =
            dotProduct(currentRay.direction, inter.normal) < 0 ? inter.normal : -inter.normal;

        if (mat->m_type == MIRROR) {
            Vector3f reflDir =
                currentRay.direction - 2.f * dotProduct(currentRay.direction, geoN) * geoN;
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
                nextDir = reflect(currentRay.direction, geoN).normalized();
            } else {
                nextDir = refract(currentRay.direction, inter.normal, mat->ior);
                if (nextDir.norm() < 1e-6f) {
                    nextDir = reflect(currentRay.direction, geoN).normalized();
                }
            }

            throughput = throughput * mat->baseColor;
            currentRay = Ray(hitPoint + nextDir * EPSILON, nextDir);
            inter = scene.intersect(currentRay);
            prevWasDelta = true;
        }

        if (mat->m_type == DIFFUSE) {

            ShadingData sd;
            if (inter.hasTangent) {
                Vector3f T = normalize(inter.tangent);
                sd = mat->buildShadingData(inter.tcoords, geoN, T, inter.tangentHandedness);
            } else {
                sd = mat->buildShadingData(inter.tcoords, geoN);
            }

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
                DirectSample light = sampleDirectLight(hitPoint, sd.N);
                radiance += throughput * evalLightSample(light, wo, sd, mat);
            }

            radiance += throughput * evalEnvSampleAt(hitPoint, wo, sd, mat);

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

        float lum = 0.2126f * throughput.x + 0.7152f * throughput.y + 0.0722f * throughput.z;
        if (lum > 20.f)
            throughput = throughput * (20.f / lum);
    }
    return radiance;
}

// ─────────────────────────────────────────────
//  castRay
// ─────────────────────────────────────────────

Vector3f Integrator::castRay(const Ray &ray) const {
    auto inter = scene.intersect(ray);

    // Miss — sample the environment map if present, otherwise return background.
    if (!inter.happened) {
        if (!scene.envMap.empty())
            return scene.envMap.sample(ray.direction);
        return scene.backgroundColor;
    }

    // Direct emitter hit on the camera ray — return emission immediately.
    // (No MIS needed: the camera ray has no prior BRDF pdf to weight against.)
    if (inter.material->m_type == EMIT)
        return inter.material->m_emission;

    // All surface types enter the unified path-tracing loop in shadeDiffuse.
    // Mirror and glass are handled inline there without recursion.
    // The first intersection is passed in to avoid re-tracing the camera ray.
    return shadeDiffuse(ray, inter, 0);
}