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

Vector3f Scene::shadeDiffuse(const Ray &ray, int depth) const {
    Vector3f radiance(0), throughput(1);
    Ray currentRay = ray;
    auto inter = intersect(ray);

    // Camera ray has no prior NEE, so treat it like a delta bounce —
    // any emitter hit on the first bounce must be counted directly.
    bool prevWasDelta = true;

    for (int bounce = depth; bounce < maxDepth; bounce++) {
        // ── Miss ──────────────────────────────────────────────────────────────
        if (!inter.happened) {
            if (!envMap.empty()) radiance += throughput * envMap.sample(currentRay.direction);
            break;
        }

        // ── Hit emitter ───────────────────────────────────────────────────────
        // Count emission only when the previous bounce was delta (mirror/glass)
        // or this is the camera ray. After a diffuse bounce, NEE already
        // sampled this light via MIS so adding it again would double-count.
        if (inter.obj->hasEmit()) {
            if (prevWasDelta) radiance += throughput * inter.material->m_emission;
            break;
        }

        Material *mat = inter.material;
        Vector3f hitPoint = inter.coords;

        // ── Flip geometric normal to face incoming ray ────────────────────────
        Vector3f geoN = dotProduct(currentRay.direction, inter.normal) < 0 ? inter.normal : -inter.normal;

        // ── Handle delta materials (MIRROR / GLASS) inline ───────────────────
        //
        // For a perfect delta BSDF the NEE MIS weight is exactly 0 (pdf → ∞),
        // so we skip NEE entirely and just follow the deterministic direction.
        // This continues the main loop rather than recursing, so RR and depth
        // limits apply normally.

        if (mat->m_type == MIRROR) {
            Vector3f reflDir = currentRay.direction - 2.f * dotProduct(currentRay.direction, geoN) * geoN;
            currentRay = Ray(hitPoint + reflDir * EPSILON, reflDir);
            inter = intersect(currentRay);
            prevWasDelta = true; // next emitter hit must be counted — NEE was skipped
            // throughput unchanged — perfect mirror reflectance = 1
            // Apply RR so infinite mirror chains can terminate
            float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
            if (get_random_float() > rrProb) break;
            throughput = throughput * (1.f / rrProb);
            continue;
        }

        if (mat->m_type == GLASS) {
            float kr = fresnel(currentRay.direction, geoN, mat->ior);
            bool doReflect = get_random_float() < kr;

            Vector3f nextDir;
            float weight;
            if (doReflect) {
                nextDir = currentRay.direction - 2.f * dotProduct(currentRay.direction, geoN) * geoN;
                weight = 1.f / kr;
            } else {
                nextDir = refract(currentRay.direction, geoN, mat->ior);
                if (nextDir.norm() < 1e-6f) {
                    // TIR fallback — treat as reflection
                    nextDir = currentRay.direction - 2.f * dotProduct(currentRay.direction, geoN) * geoN;
                    weight = 1.f;
                } else {
                    weight = 1.f / (1.f - kr);
                }
            }

            throughput = throughput * mat->baseColor * weight;
            currentRay = Ray(hitPoint + nextDir * EPSILON, nextDir);
            inter = intersect(currentRay);
            prevWasDelta = true; // next emitter hit must be counted — NEE was skipped

            // RR for glass chains
            float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
            if (get_random_float() > rrProb) break;
            throughput = throughput * (1.f / rrProb);
            continue;
        }

        // ── From here: non-delta (GGX / diffuse) surface ─────────────────────
        prevWasDelta = false; // NEE fires below, so don't count emitters next bounce

        // ── Build ShadingData ─────────────────────────────────────────────────
        ShadingData sd;
        if (inter.hasTangent) {
            Vector3f T = normalize(inter.tangent);
            Vector3f B = inter.tangentHandedness * crossProduct(geoN, T);
            sd = mat->buildShadingData(inter.tcoords, geoN, T, B);
        } else {
            sd = mat->buildShadingData(inter.tcoords, geoN);
        }

        // ── Emissive texture ──────────────────────────────────────────────────
        Vector3f emissive = mat->evalEmissive(inter.tcoords);
        if (emissive.norm() > EPSILON) {
            radiance += throughput * emissive;
            break;
        }

        // ── Sample next direction via GGX BSDF ────────────────────────────────
        Vector3f wo = -currentRay.direction;
        BSDFSample bsdf = mat->sample(wo, sd);
        if (bsdf.pdf < 1e-6f) break;

        // ── NEE: geometry light (MIS with BRDF pdf) ───────────────────────────
        //
        // Safe to fire because we're on a non-delta surface.
        if (totalEmitArea > 0.f) {
            DirectSample light = sampleDirectLight(hitPoint, sd.N);
            radiance += throughput * evalLightSample(light, wo, sd, mat);
        }

        // ── NEE: environment map ──────────────────────────────────────────────
        radiance += throughput * evalEnvSampleAt(hitPoint, wo, sd, mat);

        // ── Caustics from photon map — only at first diffuse hit after specular ─
        // NEE handles direct light. Photon map handles indirect caustics.
        // We terminate here so BRDF sampling doesn't also trace the caustic path
        // (which would double-count since photon map already covers it).
        if (!photon_map.caustic_map.empty() && prevWasDelta) {
            Vector3f caustic = photon_map.estimateIrradiance(hitPoint, sd.N) * sd.baseColor / M_PI;
            radiance += throughput * caustic;
            break;
        }

        // ── BRDF sample (GGX) — MIS-weighted hit on light or env ─────────────
        bool hitLight = false;
        Intersection nextInter;
        radiance += throughput * evalBRDFSample(bsdf.wi, bsdf.pdf, hitPoint, wo, sd, mat, hitLight, nextInter);
        if (hitLight) break;

        // ── Update throughput with GGX weight ─────────────────────────────────
        float cosTheta = std::max(0.f, dotProduct(bsdf.wi, sd.N));
        throughput = throughput * bsdf.f * cosTheta / bsdf.pdf;

        // ── Advance ray — reuse intersection from evalBRDFSample ─────────────
        currentRay = Ray(hitPoint + bsdf.wi * EPSILON, bsdf.wi);
        inter = nextInter;

        // ── Russian Roulette ──────────────────────────────────────────────────
        float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
        if (get_random_float() > rrProb) break;
        throughput = throughput * (1.f / rrProb);
    }
    return radiance;
}

// ─────────────────────────────────────────────
//  castRay
// ─────────────────────────────────────────────
//
// shadeGlass / shadeMirror are no longer called from the main loop.
// castRay dispatches to shadeDiffuse for ALL surface types so the unified
// loop handles specular chains inline.  The old shadeGlass / shadeMirror
// helpers are kept in Scene.hpp as statics for use by the photon mapper
// or any other caller that still needs them, but are removed from here.

Vector3f Scene::castRay(const Ray &ray, int depth) const {
    auto inter = intersect(ray);
    if (!inter.happened) {
        if (!envMap.empty()) return envMap.sample(ray.direction);
        return backgroundColor;
    }

    if (inter.material->m_type == EMIT) return inter.material->m_emission;

    // All surface types — specular chains handled inline inside shadeDiffuse
    return shadeDiffuse(ray, depth);
}