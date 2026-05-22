#pragma once

#include "Ray.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <cmath>

struct Camera {
    Vector3f eye = Vector3f(0, 0, -1);
    Vector3f target = Vector3f(0, 0, 0);
    Vector3f up = Vector3f(0, 1, 0);
    float fovv = 40.0f;
    float fovh = 40.0f;
    float aperture = 0.0f;
    float focusDistance = -1.0f;

    Vector3f forward = Vector3f(0, 0, 1);
    Vector3f right = Vector3f(1, 0, 0);
    Vector3f camUp = Vector3f(0, 1, 0);

    static Camera create(Vector3f eye_, Vector3f target_, Vector3f up_, float fovv_, float fovh_) {
        Camera c;
        c.eye = eye_;
        c.target = target_;
        c.fovv = fovv_;
        c.fovh = fovh_;
        c.up = up_;
        return c;
    }

    static Camera fromBlender(Vector3f eye_, Vector3f target_, Vector3f up_, float fovh_,
                              float fovv_, float focusDist_) {
        Camera c;
        c.eye = Vector3f(eye_.x, eye_.z, -eye_.y);
        c.target = Vector3f(target_.x, target_.z, -target_.y);
        c.up = normalize(Vector3f(up_.x, up_.z, -up_.y));
        c.fovv = fovv_;
        c.fovh = fovh_;
        c.focusDistance = focusDist_;
        return c;
    }

    void init(int width, int height) {
        forward = normalize(target - eye);
        right = normalize(crossProduct(forward, up));
        camUp = crossProduct(right, forward);

        aspectRatio = (float) width / (float) height;
        halfFovHRad = fovh * M_PI / 180.0f * 0.5f;
        halfFovVRad = fovv * M_PI / 180.0f * 0.5f;

        if (focusDistance < 0.0f) {
            Vector3f d = target - eye;
            focusDistance = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        }
    }

    static Vector3f sampleDisc(float u, float v) {
        float a = 2.0f * u - 1.0f;
        float b = 2.0f * v - 1.0f;
        if (a == 0.0f && b == 0.0f)
            return Vector3f(0, 0, 0);

        float r, phi;
        if (std::abs(a) > std::abs(b)) {
            r = a;
            phi = (M_PI / 4.0f) * (b / a);
        } else {
            r = b;
            phi = (M_PI / 2.0f) - (M_PI / 4.0f) * (a / b);
        }
        return Vector3f(r * std::cos(phi), r * std::sin(phi), 0.0f);
    }

    Ray generateRay(int i, int j, int width, int height, float xOffset, float yOffset) const {
        float focalLengthPx = ((float) width * 0.5f) / std::tan(halfFovHRad);

        float x = ((float) i + xOffset - (float) width * 0.5f) / focalLengthPx;
        float y = ((float) (height - j) + yOffset - (float) height * 0.5f) / focalLengthPx;

        Vector3f dir = normalize(forward + x * right + y * camUp);

        if (aperture <= 0.0f) {
            return Ray(eye, dir);
        }

        Vector3f focusPoint = eye + dir * (focusDistance / dotProduct(dir, forward));

        float u = get_random_float();
        float v = get_random_float();
        Vector3f lensOffset = sampleDisc(u, v);
        Vector3f lensPoint = eye + aperture * (lensOffset.x * right + lensOffset.y * camUp);
        Vector3f dirDoF = normalize(focusPoint - lensPoint);
        return Ray(lensPoint, dirDoF);
    }

  private:
    float halfFovHRad = 1.0f;
    float halfFovVRad = 1.0f;
    float aspectRatio = 1.0f;
};