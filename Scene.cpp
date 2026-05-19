#include "Scene.hpp"
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

// ─────────────────────────────────────────────
//  Scene setup
// ─────────────────────────────────────────────

void Scene::buildPhotonMaps(int num_photons) {
    printf("emitting photons\n");
    photon_map.emit(num_photons, *this);
    printf("building caustic grid\n");
    photon_map.build();
}

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);

    totalEmitArea = 0.f;
    for (auto obj : objects)
        if (obj->hasEmit()) totalEmitArea += obj->getArea();
}

Intersection Scene::intersect(const Ray &ray) const { return this->bvh->Intersect(ray); }

void Scene::sampleLight(Intersection &pos, float &pdf) const {
    if (totalEmitArea <= 0.f) return;
    float p = get_random_float() * totalEmitArea;
    float area_accum = 0;
    for (auto &obj : objects) {
        if (obj->hasEmit()) {
            pos.happened = true;
            area_accum += obj->getArea();
            if (p <= area_accum) {
                obj->Sample(pos, pdf);
                pdf = 1.0f / totalEmitArea;
                break;
            }
        }
    }
}

float Scene::pdfLight(const Intersection &lightInter) const { return totalEmitArea > 0.f ? 1.f / totalEmitArea : 0.f; }

// ─────────────────────────────────────────────
//  Shading helpers
// ─────────────────────────────────────────────

// Helper: sample mirror direction inline, no recursion
static Ray mirrorRay(const Vector3f &dir, const Vector3f &N, const Vector3f &hitPoint, float eps) {
    Vector3f r = dir - 2.f * dotProduct(dir, N) * N;
    return Ray(hitPoint + r * eps, r);
}

Vector3f Scene::shadeDiffuse(const Ray &ray, const Intersection &firstInter, int depth) const {
    Vector3f radiance(0), throughput(1);
    Ray currentRay = ray;
    auto inter = firstInter; // use caller-supplied intersection for the first bounce

    // Camera ray has no prior NEE, so treat it like a delta bounce —
    // any emitter hit on the first bounce must be counted directly.
    bool prevWasDelta = true;

    for (int bounce = depth; bounce < maxDepth; bounce++) {
        // ── Miss ──────────────────────────────────────────────────────────────
        if (!inter.happened) {
            if (!envMap.empty()) radiance += throughput * envMap.sample(currentRay.direction);
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
        Vector3f geoN = dotProduct(currentRay.direction, inter.normal) < 0 ? inter.normal : -inter.normal;

        if (mat->m_type == MIRROR) {
            Vector3f reflDir = currentRay.direction - 2.f * dotProduct(currentRay.direction, geoN) * geoN;
            currentRay = Ray(hitPoint + reflDir * EPSILON, reflDir);
            inter = intersect(currentRay);
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
            inter = intersect(currentRay);
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

            if (!photon_map.caustic_map.empty() && prevWasDelta) {
                Vector3f caustic = photon_map.estimateIrradiance(hitPoint, sd.N) * sd.baseColor / M_PI;
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
            if (bsdf.pdf < 1e-6f) break;

            if (totalEmitArea > 0.f) {
                DirectSample light = sampleDirectLight(hitPoint, sd.N);
                radiance += throughput * evalLightSample(light, wo, sd, mat);
            }

            radiance += throughput * evalEnvSampleAt(hitPoint, wo, sd, mat);

            bool hitLight = false;
            Intersection nextInter;
            radiance += throughput * evalBRDFSample(bsdf.wi, bsdf.pdf, hitPoint, wo, sd, mat, hitLight, nextInter);
            if (hitLight) break;

            float cosTheta = std::max(0.f, dotProduct(bsdf.wi, sd.N));
            throughput = throughput * bsdf.f * cosTheta / bsdf.pdf;

            currentRay = Ray(hitPoint + bsdf.wi * EPSILON, bsdf.wi);
            inter = nextInter;
        }

        float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
        if (get_random_float() > rrProb) break;
        throughput = throughput * (1.f / rrProb);

        float lum = 0.2126f * throughput.x + 0.7152f * throughput.y + 0.0722f * throughput.z;
        if (lum > 20.f) throughput = throughput * (20.f / lum);
    }
    return radiance;
}

// ─────────────────────────────────────────────
//  castRay
// ─────────────────────────────────────────────

Vector3f Scene::castRay(const Ray &ray, int depth) const {
    auto inter = intersect(ray);

    // Miss — sample the environment map if present, otherwise return background.
    if (!inter.happened) {
        if (!envMap.empty()) return envMap.sample(ray.direction);
        return backgroundColor;
    }

    // Direct emitter hit on the camera ray — return emission immediately.
    // (No MIS needed: the camera ray has no prior BRDF pdf to weight against.)
    if (inter.material->m_type == EMIT) return inter.material->m_emission;

    // All surface types enter the unified path-tracing loop in shadeDiffuse.
    // Mirror and glass are handled inline there without recursion.
    // The first intersection is passed in to avoid re-tracing the camera ray.
    return shadeDiffuse(ray, inter, depth);
}