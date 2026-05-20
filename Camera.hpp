#pragma once

#include "Ray.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <cmath>

/**
 * @brief Struct for managing scene camera
 *
 * Example usage:
 * scene.camera = Camera::create(Vector3f(2.78f, 2.73f, -8.0f), Vector3f(2.78f, 2.73f, 1), 40.0f);
 * scene.camera.aperture = 0.05f;
 * scene.camera.focusDistance = 9.2981;
 * scene.camera.init(scene.width, scene.height);
 *
 */
struct Camera {
    Vector3f eye = Vector3f(0, 0, -1);
    Vector3f target = Vector3f(0, 0, 0);
    Vector3f up = Vector3f(0, 1, 0);
    float fov = 40.0f;           // vertical fov in degrees
    float aperture = 0.0f;       // lens radius — 0 = pinhole, >0 = DoF blur
    float focusDistance = -1.0f; // negative = auto (uses distance to target)

    /**
     * @brief Create's a camera from eye, target, and up vectors with given fov.
     *
     * @param eye_
     * @param target_
     * @param fov_
     * @param up_
     * @return Camera
     */
    static Camera create(Vector3f eye_, Vector3f target_, Vector3f up_, float fov_) {
        Camera c;
        c.eye = eye_;
        c.target = target_;
        c.fov = fov_;
        c.up = up_;
        return c;
    }

    /**
     * @brief Create's a camera from eye, target, up, and fov obtained from blender.
     * Blender uses Z-up world coordinate frame, but our renderer uses Y-up.
     * Our renderer also looks along +Z
     *
     * @param eye_
     * @param target_
     * @param fov_
     * @return Camera
     */
    static Camera fromBlender(Vector3f eye_, Vector3f target_, float fov_) {
        Camera c;
        c.eye = Vector3f(eye_.x, eye_.z, -eye_.y);
        c.target = Vector3f(target_.x, target_.z, -target_.y);
        c.fov = fov_;
        c.up = Vector3f(0, 1, 0);
        return c;
    }

    /**
     * @brief Initialises the camera based on the scene.
     * Call this after constructing the camera.
     */
    void init(int width, int height) {
        forward = normalize(target - eye);
        right = normalize(crossProduct(forward, up));
        camUp = crossProduct(right, forward);
        scale = std::tan(fov * M_PI / 180.0f * 0.5f);
        aspectRatio = (float)width / (float)height;

        // Use focus distance < 0 to auto focus on target position.
        if (focusDistance < 0.0f) {
            Vector3f d = target - eye;
            focusDistance = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        }
    }

    /**
     * @brief Samples a point on a point an a disk using provided random numbers u and v.
     *
     * @param u A random number from [0, 1)
     * @param v A random number from [0, 1)
     * @return Vector3f A point on a disk.
     */
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

    /**
     * @brief Generate a camera ray for pixel (i, j) with optional depth of field.
     *
     * Applies sub-pixel jitter via (xOffset, yOffset) for anti-aliasing.
     * When aperture > 0, samples the lens disc to produce DoF blur —
     * objects at focusDistance are sharp, everything else falls off.
     *
     * @param i Pixel column
     * @param j Pixel row
     * @param W Image width in pixels
     * @param H Image height in pixels
     * @param xOffset Sub-pixel jitter in x [0, 1)
     * @param yOffset Sub-pixel jitter in y [0, 1)
     * @return Ray in world space
     */
    Ray generateRay(int i, int j, int width, int height, float xOffset, float yOffset) const {
        float x = (2.0f * ((float)i + xOffset) / (float)width - 1.0f) * scale * aspectRatio;
        float y = (1.0f - 2.0f * ((float)j + yOffset) / (float)height) * scale;
        Vector3f pinholeDir = normalize(forward + x * right + y * camUp);

        // If there is no aperture then treat camera as pinhole.
        if (aperture <= 0.0f) {
            return Ray(eye, pinholeDir);
        }

        // Point on the focus plane that the pinhole ray passes through
        Vector3f focusPoint = eye + pinholeDir * (focusDistance / dotProduct(pinholeDir, forward));

        // Sample random point on lens disc.
        float u = get_random_float();
        float v = get_random_float();
        Vector3f lensOffset = sampleDisc(u, v);

        // Lens sample point in world space
        Vector3f lensPoint = eye + aperture * (lensOffset.x * right + lensOffset.y * camUp);

        // Ray from lens point toward focus point
        Vector3f dir = normalize(focusPoint - lensPoint);
        return Ray(lensPoint, dir);
    }

private:
    Vector3f forward = Vector3f(0, 0, 1);
    Vector3f right = Vector3f(1, 0, 0);
    Vector3f camUp = Vector3f(0, 1, 0);
    float scale = 1.0f;
    float aspectRatio = 1.0f;
};