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

        // ── Mirror: perfect specular reflection ───────────────────────────────
        // Delta BSDF — NEE and BRDF-MIS both have zero weight (pdf is a Dirac
        // delta), so we skip both and just follow the deterministic reflect
        // direction. Throughput picks up baseColor for tinted mirrors (white = 1).
        // RR still applies so infinite mirror chains can terminate.
        if (mat->m_type == MIRROR) {
            Vector3f reflDir = currentRay.direction - 2.f * dotProduct(currentRay.direction, geoN) * geoN;
            currentRay = Ray(hitPoint + reflDir * EPSILON, reflDir);
            inter = intersect(currentRay);
            prevWasDelta = true;
            throughput = throughput * mat->baseColor;
            float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
            if (get_random_float() > rrProb) break;
            throughput = throughput * (1.f / rrProb);
            continue;
        }

        // ── Glass: perfect specular dielectric (reflect or refract) ──────────
        // Importance-sample the two delta lobes by choosing reflect with
        // probability kr, refract with probability (1-kr).  Sampling
        // proportional to contribution means the Monte Carlo weight for
        // whichever path is chosen cancels to exactly 1 — no division needed.
        //
        // fresnel / refract receive inter.normal (raw, unflipped) so they can
        // detect inside/outside themselves via sign(dot(I,N)) and apply the
        // correct IOR ratio.  reflect uses geoN (already facing the ray) since
        // the reflection formula requires the normal on the incident side.
        //
        // TIR: refract returns a zero vector when the angle exceeds the
        // critical angle; fall back to full reflection with weight 1.
        if (mat->m_type == GLASS) {
            float kr = fresnel(currentRay.direction, inter.normal, mat->ior);
            bool doReflect = get_random_float() < kr;

            Vector3f nextDir;
            if (doReflect) {
                nextDir = reflect(currentRay.direction, geoN).normalized();
            } else {
                nextDir = refract(currentRay.direction, inter.normal, mat->ior);
                if (nextDir.norm() < 1e-6f) {
                    // TIR: refract returned zero — fall back to full reflection
                    nextDir = reflect(currentRay.direction, geoN).normalized();
                }
            }

            // Weight = 1: the sampling probability cancels the BSDF value exactly.
            throughput = throughput * mat->baseColor;
            currentRay = Ray(hitPoint + nextDir * EPSILON, nextDir);
            inter = intersect(currentRay);
            prevWasDelta = true;

            float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
            if (get_random_float() > rrProb) break;
            throughput = throughput * (1.0f / rrProb);
            continue;
        }

        // ── From here: non-delta (GGX / diffuse) surface ─────────────────────
        bool wasDelta = prevWasDelta; // save before overwriting — caustic check below needs it
        prevWasDelta = false;         // NEE fires this bounce, so emitter hits next bounce must not be double-counted

        // ── Build ShadingData ─────────────────────────────────────────────────
        // Resolves all textures (base colour, normal map, roughness/metallic)
        // into a flat struct.  This is the only place texture sampling happens.
        ShadingData sd;
        if (inter.hasTangent) {
            Vector3f T = normalize(inter.tangent);
            sd = mat->buildShadingData(inter.tcoords, geoN, T, inter.tangentHandedness);
        } else {
            sd = mat->buildShadingData(inter.tcoords, geoN);
        }

        // ── Emissive texture ──────────────────────────────────────────────────
        // Meshes with an emissive texture but no EMIT material type end up here.
        // Treat them as a light and terminate — no further bounce needed.
        Vector3f emissive = mat->evalEmissive(inter.tcoords);
        if (emissive.norm() > EPSILON) {
            radiance += throughput * emissive;
            break;
        }

        // ── Sample next direction via VNDF BSDF ───────────────────────────────
        // sample() draws wi from a mixture of VNDF-GGX (specular) and
        // cosine-hemisphere (diffuse), weighted by specularWeight.
        // VNDF guarantees VdotH > 0, eliminating the firefly source that
        // plain NDF sampling produces.
        Vector3f wo = -currentRay.direction;
        BSDFSample bsdf = mat->sample(wo, sd);
        if (bsdf.pdf < 1e-6f) break;

        // ── NEE: area light (MIS, power heuristic) ────────────────────────────
        // Sample the light surface directly and combine with the BRDF pdf via
        // MIS.  Only valid on non-delta surfaces; delta surfaces skip NEE above.
        if (totalEmitArea > 0.f) {
            DirectSample light = sampleDirectLight(hitPoint, sd.N);
            radiance += throughput * evalLightSample(light, wo, sd, mat);
        }

        // ── NEE: environment map (MIS, power heuristic) ───────────────────────
        // Importance-samples the env map and combines with the BRDF pdf via MIS.
        radiance += throughput * evalEnvSampleAt(hitPoint, wo, sd, mat);

        // ── Caustics from photon map ───────────────────────────────────────────
        // Inject caustic irradiance only at the first diffuse hit after a specular
        // bounce.  NEE covers direct illumination; the photon map covers indirect
        // caustic energy that path tracing alone would need many bounces to find.
        // Do NOT terminate here — the path continues for indirect GI as normal.
        if (!photon_map.caustic_map.empty() && wasDelta) {
            Vector3f caustic = photon_map.estimateIrradiance(hitPoint, sd.N) * sd.baseColor / M_PI;
            radiance += throughput * caustic;
        }

        // ── BRDF sample — MIS-weighted direct hit on light or env ────────────
        // Traces the sampled direction and computes the MIS-weighted contribution
        // if it hits an emitter or misses into the env map.  Also returns the
        // next intersection so we avoid a redundant BVH traversal.
        bool hitLight = false;
        Intersection nextInter;
        radiance += throughput * evalBRDFSample(bsdf.wi, bsdf.pdf, hitPoint, wo, sd, mat, hitLight, nextInter);
        if (hitLight) break;

        // ── Path throughput update ─────────────────────────────────────────────
        // throughput *= f(wi,wo) * cos(theta_i) / pdf(wi)
        // This is the standard Monte Carlo weight for the rendering equation.
        float cosTheta = std::max(0.f, dotProduct(bsdf.wi, sd.N));
        throughput = throughput * bsdf.f * cosTheta / bsdf.pdf;

        // Clamp luminance to suppress firefly variance spikes
        float lum = 0.2126f * throughput.x + 0.7152f * throughput.y + 0.0722f * throughput.z;
        if (lum > 10.f) {
            throughput = throughput * (10.f / lum);
        }

        // Reuse the intersection already computed in evalBRDFSample.
        currentRay = Ray(hitPoint + bsdf.wi * EPSILON, bsdf.wi);
        inter = nextInter;

        // ── Russian Roulette path termination ─────────────────────────────────
        // Terminate with probability (1 - rrProb), compensate surviving paths
        // by dividing throughput by rrProb so the estimator stays unbiased.
        // Clamping rrProb to RussianRoulette prevents throughput blow-up on
        // bright paths.
        float rrProb = std::min(RussianRoulette, std::max({throughput.x, throughput.y, throughput.z}));
        if (get_random_float() > rrProb) break;
        throughput = throughput * (1.f / rrProb);
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