#include "Scene.hpp"
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

// ─────────────────────────────────────────────
//  Scene setup
// ─────────────────────────────────────────────

void Scene::buildPhotonMaps(int num_photons)
    {
        printf("emitting photons\n");
        photon_map.emit(num_photons, *this);
        printf("building caustic grid\n");
        photon_map.build();
    }
    
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
    if (!photon_map.caustic_map.empty())
        caustic = photon_map.estimateIrradiance(inter.coords, inter.normal)
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