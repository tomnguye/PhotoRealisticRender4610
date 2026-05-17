//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#ifdef _OPENMP
#include <omp.h>
#endif

Vector3f Scene::estimateIrradiance(Vector3f pos, Vector3f normal, const PhotonGrid& grid, float radius) const {
    // get nearby photons
    auto nearby = grid.query(pos, radius);
    if (nearby.empty()) return {0, 0, 0};
    
    // find actual max distance for area calculation
    float max_dist = 0;
    for (auto& p : nearby) {
        float dist = (p.position - pos).norm();
        max_dist = std::max(max_dist, dist);
    }
    
    float area = M_PI * max_dist * max_dist;
    if (area < EPSILON) return {0, 0, 0};
    
    Vector3f irradiance = {0, 0, 0};
    for (auto& p : nearby) {
        // only count photons coming from front of surface
        float cos_theta = dotProduct(-p.direction, normal);
        if (cos_theta <= 0) continue;
        
        // evaluate BSDF at this point for photon direction
        // for diffuse: BRDF = color / pi
        irradiance += p.power * cos_theta;
    }
    
    return irradiance / area;
}

void Scene::tracePhotonThread(Photon p, int depth,
                               std::vector<Photon>& t_caustic) {
    if (depth > maxDepth) return;
    
    auto hit = intersect(Ray(p.position, p.direction));
    if (!hit.happened) return;
    if (hit.material->hasEmission()) return;
    
    if (hit.material->m_type == MIRROR) {
        p.is_caustic = true;
        p.direction = reflect(p.direction, hit.normal);
        p.position = hit.coords + hit.normal * EPSILON;
        tracePhotonThread(p, depth + 1, t_caustic);
    }
    else if (hit.material->m_type == GLASS) {
        float kr = fresnel(p.direction, hit.normal, hit.material->ior);
        
        Vector3f refracted = refract(p.direction, hit.normal, hit.material->ior);
        bool tir = (refracted.x == 0 && refracted.y == 0 && refracted.z == 0);
        
        // probabilistically choose reflect or refract
        if (get_random_float() < kr || tir) {
            // reflect
            p.direction = reflect(p.direction, hit.normal);
            p.position = hit.coords + hit.normal * EPSILON;
        } else {
            // refract
            p.is_caustic = true; // glass creates caustics
            p.direction = refracted;
            p.position = hit.coords - hit.normal * EPSILON;
        }
        
        tracePhotonThread(p, depth + 1, t_caustic);
    }
    else {
        if (depth > 0) {
            Photon stored;
            stored.position = hit.coords;
            stored.direction = p.direction;
            stored.power = p.power;
            stored.is_caustic = p.is_caustic;
            
            if (p.is_caustic)
                t_caustic.push_back(stored);
            // else
            //     t_global.push_back(stored);
        }
        float survival = std::max({p.power.x, p.power.y, p.power.z});
        if (get_random_float() < survival && depth < maxDepth) {
            p.power = p.power * hit.material->getColor() / survival;
            p.position = hit.coords + hit.normal * EPSILON;
            p.direction = hit.material->sample(p.direction, hit.normal);
            p.is_caustic = false;
            tracePhotonThread(p, depth + 1, t_caustic);
        }
    }
}

void Scene::emitPhotons(int num_photons) {
    // global_map.reserve(num_photons);
    int num_threads = 8;
    std::vector<std::vector<Photon>> thread_caustic(num_threads);
    for (auto& v : thread_caustic)
        v.reserve(num_photons / num_threads / 10);
    
    #pragma omp parallel for num_threads(num_threads) schedule(dynamic, 100)
    for (int i = 0; i < num_photons; i++) {
        // step 1: sample a point on a light (you already have this)
        int tid = omp_get_thread_num();
        Intersection lightPoint;
        float light_pdf;
        sampleLight(lightPoint, light_pdf);
        float area = 1.0f / light_pdf;

        // step 2: sample a direction from the light
        // uniform hemisphere around light normal
        Vector3f dir = lightPoint.material->sample(0, lightPoint.normal);
        // step 3: compute photon power
        // total power divided by number of photons
        Vector3f power = lightPoint.material->getEmission() 
                       * area
                       / num_photons;
        
        // step 4: create and trace photon
        Photon p;
        p.position = lightPoint.coords + lightPoint.normal * EPSILON;
        p.direction = dir;
        p.power = power;
        p.is_caustic = false;
        
        tracePhotonThread(p, 0, thread_caustic[tid]);
    }

    size_t total = 0;
    for (auto& v : thread_caustic) total += v.size();
    caustic_map.reserve(total);
    for (int t = 0; t < num_threads; t++) {
        caustic_map.insert(caustic_map.end(), thread_caustic[t].begin(), thread_caustic[t].end());
    }
}

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);

    for (auto& obj : objects)
        if (obj->hasEmit())
            total_emit_area += obj->getArea();
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}


void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    // First pass: sum total emissive area
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            pos.happened = true;
            emit_area_sum += objects[k]->getArea();
        }
    }

    // Select a light proportional to area
    float p = get_random_float() * emit_area_sum;
    float area_accum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            area_accum += objects[k]->getArea();
            if (p <= area_accum) {
                // Sample a point on this light
                objects[k]->Sample(pos, pdf);

                // pdf from Sample() is 1/this_object_area
                // but we need 1/total_emit_area
                // since p(choosing this light) = this_area/total_area
                // p(this point) = p(point|light) * p(light)
                //               = (1/this_area) * (this_area/total_area)
                //               = 1/total_area
                pdf = pdf * objects[k]->getArea() / emit_area_sum;
                // which simplifies to just:
                // pdf = 1 / emit_area_sum
                break;
            }
        }
    }
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{   
    Vector3f hitColor = Vector3f(0);
    auto inter = intersect(ray);  // find the cloest intersection of the ray and the objects

    if (!inter.happened)return backgroundColor;  // if no intersection, return background color

    Vector3f hitPoint = inter.coords;  // the intersection point
    Vector3f N = inter.normal; // normal
    Vector2f st = inter.tcoords; // texture coordinates (u, v)
    Vector3f dir = ray.direction;  // ray direction
    Vector3f indirect = 0;
    Vector3f direct = 0;

    if (inter.material->m_type == EMIT) {  // if the object is a light source
        return inter.material->m_emission;  // return light color
    } else if (inter.material->m_type == DIFFUSE ) {
        // TODO: task 2 Monte Carlo Path Tracing with Russian Roulette termination
        Vector3f color = inter.material->textured ?
                inter.material->getColorAt(inter.tcoords.x, inter.tcoords.y) :
                inter.material->getColor();

        if (get_random_float() < RussianRoulette) {
            Vector3f wo = inter.material->sample(ray.direction, inter.normal);
            float pdf_bsdf = std::max(EPSILON, inter.material->pdf(ray.direction, wo, inter.normal));
            Vector3f brdf = inter.material->eval(wo, inter.normal);
            float cosTheta = std::max(0.f, dotProduct(wo, inter.normal));
            Ray indirect_ray(inter.coords + inter.normal * EPSILON, wo);
            auto indirect_hit = intersect(indirect_ray);

            if (indirect_hit.happened) {
                if (!indirect_hit.material->hasEmission()) {
                    // normal indirect bounce, no emission, no MIS needed
                    auto bleed = castRay(indirect_ray, depth + 1);
                    indirect = color * brdf * bleed * cosTheta / pdf_bsdf / RussianRoulette;
                    indirect = Vector3f::Min(1, indirect);
                }
            }
        }

        // Direct
        Intersection shadingPoint;
        float light_pdf;
        sampleLight(shadingPoint, light_pdf);
        if (shadingPoint.happened) {
            Vector3f towards_light = (shadingPoint.coords - inter.coords).normalized();
            Ray shadow_ray = Ray(inter.coords + inter.normal * EPSILON, towards_light);
            auto shadow_hit = intersect(shadow_ray);

            float dist = (shadingPoint.coords - inter.coords).norm();
            float dist2 = dist * dist;
            float cos_theta_light = std::max(0.f, dotProduct(-towards_light, shadingPoint.normal));
            float cos_theta_surface = std::max(0.f, dotProduct(towards_light, inter.normal));

            bool occluded = shadow_hit.happened &&
                        (shadow_hit.coords - inter.coords).norm() <
                        (shadingPoint.coords - inter.coords).norm() * (1 - EPSILON);
            if (!occluded && cos_theta_light > 0) {
                float pdf_light_solid_angle = (1.0f / total_emit_area) * dist2 / cos_theta_light;

                // what would bsdf sampler have given this direction?
                float pdf_bsdf_at_light = inter.material->pdf(ray.direction, towards_light, inter.normal);

                // power heuristic
                float w_light = (pdf_light_solid_angle * pdf_light_solid_angle) /
                            (pdf_light_solid_angle * pdf_light_solid_angle + 
                                pdf_bsdf_at_light * pdf_bsdf_at_light);

                Vector3f brdf_direct = inter.material->eval(towards_light, inter.normal);
                Vector3f emission = shadingPoint.material->getEmission();
                direct = w_light * color * brdf_direct * emission * cos_theta_surface / pdf_light_solid_angle;
            }
        }

        Vector3f caustic = {0, 0, 0};
        Vector3f indirect_pm = {0, 0, 0};
        
        if (!caustic_map.empty()) {
            caustic = estimateIrradiance(inter.coords, inter.normal, 
                                        caustic_grid, photon_radius);
            caustic = caustic * color / M_PI;
        }

        return direct + caustic + indirect;

    } else if (inter.material->m_type == GLASS && TASK_N >= 1.3f) {  // if the object is glass
        // TODO: task 1.3 Glass Material
        // if the depth exceeds the maximum depth, return the hitColor to avoid infinite recursion
        if (depth <= maxDepth) {
            float kr = fresnel(ray.direction, inter.normal, inter.material->ior);
            Vector3f reflect_dir = reflect(ray.direction, inter.normal);
            Vector3f refract_dir = refract(ray.direction, inter.normal, inter.material->ior);
            Ray reflected_ray(inter.coords + reflect_dir * EPSILON, reflect_dir);
            Ray refracted_ray(inter.coords + refract_dir * EPSILON, refract_dir);
            Vector3f color_reflection = castRay(reflected_ray, depth + 1);
            Vector3f color_refraction = castRay(refracted_ray, depth + 1);
            hitColor += kr * color_reflection + (1-kr) * color_refraction;
        }
    } else if (inter.material->m_type == MIRROR && TASK_N >= 1.3f) {  // if the object is mirror
        // TODO: task 1.3 Mirror Refection
        if (depth <= maxDepth) {
            Vector3f reflect_dir = reflect(ray.direction, inter.normal);
            Ray reflected_ray(inter.coords + reflect_dir * EPSILON, reflect_dir);
            hitColor += castRay(reflected_ray, depth + 1);
        }
        
    }
    return hitColor;
}
