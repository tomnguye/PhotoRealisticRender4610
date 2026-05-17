//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        tnear= std::numeric_limits<double>::max();
        obj =nullptr;
        material=nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector2f tcoords;
    Vector3f normal;
    double tnear;  // the distance between the intersection point and the ray origin
    Object* obj;
    Material* material;
};
#endif //RAYTRACING_INTERSECTION_H
