#include "Scene.hpp"
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

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
        if (obj->hasEmit())
            totalEmitArea += obj->getArea();
}

Intersection Scene::intersect(const Ray &ray) const {
    return this->bvh->Intersect(ray);
}

bool Scene::intersectP(const Ray &ray, float tMax) const {
    return this->bvh->IntersectP(ray, tMax);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
    if (totalEmitArea <= 0.f)
        return;
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

float Scene::pdfLight(const Intersection &lightInter) const {
    return totalEmitArea > 0.f ? 1.f / totalEmitArea : 0.f;
}
