//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Material.hpp"
#include "Vector.hpp"
class Object;
class Sphere;

struct Intersection {
    Intersection() {
        happened = false;
        coords = Vector3f();
        normal = Vector3f();
        tangent = Vector3f();
        tangentHandedness = 1.f;
        hasTangent = false;
        tnear = std::numeric_limits<double>::max();
        obj = nullptr;
        material = nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector2f tcoords; // interpolated UVs (not raw barycentric)
    Vector3f normal;
    Vector3f tangent;        // interpolated tangent for normal mapping
    float tangentHandedness; // gltf w component  +1 or -1, for bitangent sign
    bool hasTangent;
    double tnear;
    Object *obj;
    Material *material;
};
#endif // RAYTRACING_INTERSECTION_H