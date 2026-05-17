//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include "Vector.hpp"

class PointLight
{
public:
    PointLight(const Vector3f &p, const Vector3f &i) : position(p), intensity(i) {}
    virtual ~PointLight() = default;
    Vector3f position;
    Vector3f intensity;
};
