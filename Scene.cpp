#include "Scene.hpp"
#ifdef _OPENMP
#include <omp.h>
#endif

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────

Vector3f Scene::estimateIrradiance(Vector3f pos, Vector3f normal,
                                    const PhotonGrid& grid, float radius) const
{
    auto nearby = grid.query(pos, radius);
    if (nearby.empty()) return {};

    float max_dist = 0;
    for (auto& p : nearby)
        max_dist = std::max(max_dist, (p.position - pos).norm());

    float area = M_PI * max_dist * max_dist;
    if (area < EPSILON) return {};

    Vector3f irradiance = {};
    for (auto& p : nearby) {
        float cos_theta = dotProduct(-p.direction, normal);
        if (cos_theta > 0)
            irradiance += p.power * cos_theta;
    }
    return irradiance / area;
}

// ─────────────────────────────────────────────
//  Photon tracing
// ─────────────────────────────────────────────

void Scene::tracePhotonThread(Photon p, int depth, std::vector<Photon>& t_caustic)
{
    if (depth > maxDepth) return;

    auto hit = intersect(Ray(p.position, p.direction));
    if (!hit.happened || hit.material->hasEmission()) return;

    if (hit.material->m_type == MIRROR) {
        p.is_caustic = true;
        p.direction  = reflect(p.direction, hit.normal).normalized();
        p.position   = hit.coords + hit.normal * EPSILON;
        tracePhotonThread(p, depth + 1, t_caustic);
    }
    else if (hit.material->m_type == GLASS) {
        float     kr        = fresnel(p.direction, hit.normal, hit.material->ior);
        Vector3f  refracted = refract(p.direction, hit.normal, hit.material->ior);
        bool      tir       = (refracted.norm() < EPSILON);

        if (get_random_float() < kr || tir) {
            p.power    = p.power * kr;
            p.direction = reflect(p.direction, hit.normal).normalized();
            p.position  = hit.coords + hit.normal * EPSILON;
        } else {
            p.is_caustic = true;
            p.power     = p.power * (1.0f - kr);
            p.direction  = refracted.normalized();
            p.position   = hit.coords - hit.normal * EPSILON;
        }
        tracePhotonThread(p, depth + 1, t_caustic);
    }
    else { // diffuse
        if (depth > 0 && p.is_caustic) {
            t_caustic.push_back({ hit.coords, p.direction, p.power, p.is_caustic });
        }

        float survival = std::min(1.0f, std::max({p.power.x, p.power.y, p.power.z}));
        if (get_random_float() < survival && depth < maxDepth) {
            p.power     = p.power * hit.material->getColor() / survival;
            p.position  = hit.coords + hit.normal * EPSILON;
            p.direction = hit.material->sample(p.direction, hit.normal);
            tracePhotonThread(p, depth + 1, t_caustic);
        }
    }
}

void Scene::emitPhotons(int num_photons)
{
    const int num_threads = 8;
    std::vector<std::vector<Photon>> thread_caustic(num_threads);
    for (auto& v : thread_caustic)
        v.reserve(num_photons / num_threads / 10);

    #pragma omp parallel for num_threads(num_threads) schedule(dynamic, 100)
    for (int i = 0; i < num_photons; i++) {
        int tid = omp_get_thread_num();

        Intersection lightPoint;
        float light_pdf;
        sampleLight(lightPoint, light_pdf);

        float    area  = 1.0f / light_pdf;
        Vector3f dir   = lightPoint.material->sample(0, lightPoint.normal);
        Vector3f power = lightPoint.material->getEmission() * area / num_photons;

        Photon p;
        p.position   = lightPoint.coords + lightPoint.normal * EPSILON;
        p.direction  = dir;
        p.power      = power;
        p.is_caustic = false;

        tracePhotonThread(p, 0, thread_caustic[tid]);
    }

    size_t total = 0;
    for (auto& v : thread_caustic) total += v.size();
    caustic_map.reserve(total);
    for (auto& v : thread_caustic)
        caustic_map.insert(caustic_map.end(), v.begin(), v.end());
}

// ─────────────────────────────────────────────
//  Scene setup
// ─────────────────────────────────────────────

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);

    for (auto& obj : objects)
        if (obj->hasEmit())
            total_emit_area += obj->getArea();
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (auto& obj : objects)
        if (obj->hasEmit()) {
            pos.happened = true;
            emit_area_sum += obj->getArea();
        }

    float p = get_random_float() * emit_area_sum;
    float area_accum = 0;
    for (auto& obj : objects) {
        if (obj->hasEmit()) {
            area_accum += obj->getArea();
            if (p <= area_accum) {
                obj->Sample(pos, pdf);
                pdf = 1.0f / emit_area_sum;
                break;
            }
        }
    }
}

// ─────────────────────────────────────────────
//  Shading helpers
// ─────────────────────────────────────────────

Vector3f Scene::shadeDiffuse(const Ray& ray, const Intersection& inter, int depth) const
{
    Vector3f color = inter.material->textured
        ? inter.material->getColorAt(inter.tcoords.x, inter.tcoords.y)
        : inter.material->getColor();

    // --- indirect ---
    Vector3f indirect = {};
    if (get_random_float() < RussianRoulette) {
        Vector3f wo        = inter.material->sample(ray.direction, inter.normal);
        float    pdf_bsdf  = std::max(EPSILON, inter.material->pdf(ray.direction, wo, inter.normal));
        Vector3f brdf      = inter.material->eval(wo, inter.normal);
        float    cosTheta  = std::max(0.f, dotProduct(wo, inter.normal));
        Ray      bounce(inter.coords + inter.normal * EPSILON, wo);
        auto     bounce_hit = intersect(bounce);

        if (bounce_hit.happened && !bounce_hit.material->hasEmission()) {
            auto bleed = castRay(bounce, depth + 1);
            indirect   = Vector3f::Min(1, color * brdf * bleed * cosTheta / pdf_bsdf / RussianRoulette);
        }
    }

    // --- direct ---
    Vector3f direct = {};
    Intersection shadingPoint;
    float light_pdf;
    sampleLight(shadingPoint, light_pdf);

    if (shadingPoint.happened) {
        Vector3f towards_light    = (shadingPoint.coords - inter.coords).normalized();
        float    dist             = (shadingPoint.coords - inter.coords).norm();
        float    dist2            = dist * dist;
        float    cos_theta_light  = std::max(0.f, dotProduct(-towards_light, shadingPoint.normal));
        float    cos_theta_surf   = std::max(0.f, dotProduct(towards_light, inter.normal));

        Ray  shadow_ray(inter.coords + inter.normal * EPSILON, towards_light);
        auto shadow_hit = intersect(shadow_ray);
        bool occluded   = shadow_hit.happened &&
                          (shadow_hit.coords - inter.coords).norm() <
                          (shadingPoint.coords - inter.coords).norm() * (1 - EPSILON);

        if (!occluded && cos_theta_light > 0) {
            float    pdf_solid  = (1.0f / total_emit_area) * dist2 / cos_theta_light;
            float    pdf_bsdf   = inter.material->pdf(ray.direction, towards_light, inter.normal);
            float    w_light    = (pdf_solid * pdf_solid) /
                                  (pdf_solid * pdf_solid + pdf_bsdf * pdf_bsdf);
            Vector3f brdf       = inter.material->eval(towards_light, inter.normal);
            Vector3f emission   = shadingPoint.material->getEmission();
            direct = w_light * color * brdf * emission * cos_theta_surf / pdf_solid;
        }
    }

    // --- caustics ---
    Vector3f caustic = {};
    if (!caustic_map.empty())
        caustic = estimateIrradiance(inter.coords, inter.normal, caustic_grid, photon_radius)
                  * color / M_PI;

    return direct + indirect + caustic;
}

Vector3f Scene::shadeGlass(const Ray& ray, const Intersection& inter, int depth) const
{
    if (depth > maxDepth) return {};

    float    kr          = fresnel(ray.direction, inter.normal, inter.material->ior);
    Vector3f reflect_dir = reflect(ray.direction, inter.normal);
    Vector3f refract_dir = refract(ray.direction, inter.normal, inter.material->ior);

    Ray reflected(inter.coords + reflect_dir * EPSILON, reflect_dir);
    Ray refracted(inter.coords + refract_dir * EPSILON, refract_dir);

    return kr * castRay(reflected, depth + 1) + (1 - kr) * castRay(refracted, depth + 1);
}

Vector3f Scene::shadeMirror(const Ray& ray, const Intersection& inter, int depth) const
{
    if (depth > maxDepth) return {};

    Vector3f reflect_dir = reflect(ray.direction, inter.normal);
    Ray reflected(inter.coords + reflect_dir * EPSILON, reflect_dir);
    return castRay(reflected, depth + 1);
}

// ─────────────────────────────────────────────
//  castRay
// ─────────────────────────────────────────────

Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    auto inter = intersect(ray);
    if (!inter.happened) return backgroundColor;

    switch (inter.material->m_type) {
        case EMIT:   return inter.material->m_emission;
        case DIFFUSE: return shadeDiffuse(ray, inter, depth);
        case GLASS:  return shadeGlass(ray, inter, depth);
        case MIRROR: return shadeMirror(ray, inter, depth);
        default:     return {};
    }
}