#pragma once

#include "BVH.hpp"
#include "Camera.hpp"
#include "EnvMap.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "Photon.hpp"
#include "PointLight.hpp"
#include "Ray.hpp"
#include "Vector.hpp"
#include "Volume.hpp"
#include <vector>

class Scene {
  public:
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    std::vector<Object *> objects;
    BVHAccel *bvh;
    EnvMap envMap;
    Camera camera;
    float totalEmitArea = 0.f;
    PhotonMap photon_map;
    Medium medium;

    void Add(Object *object) {
        objects.push_back(object);
    }

    const std::vector<Object *> &get_objects() const {
        return objects;
    }

    Intersection intersect(const Ray &ray) const;
    bool intersectP(const Ray &ray, float tMax = std::numeric_limits<float>::infinity()) const;
    void buildBVH();

    void sampleLight(Intersection &pos, float &pdf) const;
    float pdfLight(const Intersection &lightInter) const;
    void buildPhotonMaps(int num_photons);
};