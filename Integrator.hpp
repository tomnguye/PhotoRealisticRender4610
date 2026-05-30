#pragma once

#include "EnvMap.hpp"
#include "Ray.hpp"
#include "Scene.hpp"
#include "Vector.hpp"

struct LightSample {
    Vector3f emission;
    Vector3f dir;
    float pdf; // Solid angle PDF
    bool visible;
};

class Integrator {

  public:
    float directClamp = 0.f;
    float indirectClamp = 10.0f;
    Integrator(const Scene &scene_, int maxDepth_, float rrThreshold_)
        : scene(scene_), maxDepth(maxDepth_), rrThreshold(rrThreshold_) {}

    Vector3f castRay(const Ray &ray) const;

  private:
    const Scene &scene;
    int maxDepth;
    float rrThreshold;

    inline float mis(float a, float b) const {
        return (a * a) / (a * a + b * b + 1e-6f);
    }

    LightSample sampleDirectLight(const Vector3f &hitPoint, const Vector3f &N) const;
    Vector3f evalLightSample(const LightSample &light, const Vector3f &wo, const ShadingData &sd,
                             Material *mat) const;

    LightSample sampleEnvironmentMap(const Vector3f &hitpoint) const;
    Vector3f evalEnvironmentSample(const LightSample &sample, const Vector3f &wo,
                                   const ShadingData &sd, Material *mat) const;

    Vector3f evalBRDFSample(const Vector3f &wi, float brdfPdf, const Vector3f &hitPoint,
                            const Vector3f &wo, const ShadingData &sd, Material *mat,
                            bool &hitLight, Intersection &nextInter) const;
};
