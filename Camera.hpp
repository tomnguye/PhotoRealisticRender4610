#pragma once

#include "Ray.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <cmath>

// ─── Camera ───────────────────────────────────────────────────────────────────
//
// Look-at camera with optional thin lens depth of field.
//
// Pinhole (default):
//   aperture = 0  →  no blur, all depths sharp
//
// Thin lens DoF:
//   aperture > 0  →  objects at focusDistance are sharp,
//                    objects closer/farther are blurred
//   focusDistance  →  distance from eye to the sharp plane
//                    tip: set this to length(target - eye) to keep
//                    your target point in focus
//
// Blender coordinate conversion:
//   Blender is Z-up, your renderer is Y-up.
//   Use Camera::fromBlender() to convert automatically.
//   Blender (x, y, z) → Renderer (x, z, y)
//
// Getting values from Blender scripting tab:
//
//   import bpy
//   from mathutils import Vector
//   cam = bpy.context.scene.camera
//   eye    = cam.location
//   target = cam.location + cam.matrix_world.to_3x3() @ Vector((0, 0, -1))
//   print(f"eye:    ({eye.x:.3f}, {eye.y:.3f}, {eye.z:.3f})")
//   print(f"target: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})")
//   print(f"fov:    {bpy.data.cameras[cam.data.name].angle * 57.2958:.3f} deg")

struct Camera {
    Vector3f eye = Vector3f(0, 0, -1);
    Vector3f target = Vector3f(0, 0, 0);
    Vector3f up = Vector3f(0, 1, 0);
    float fov = 40.f;           // vertical fov in degrees
    float aperture = 0.f;       // lens radius — 0 = pinhole, >0 = DoF blur
    float focusDistance = -1.f; // negative = auto (uses distance to target)

    // ── Constructor helpers ───────────────────────────────────────────────────

    static Camera create(Vector3f eye_, Vector3f target_, float fov_, Vector3f up_ = Vector3f(0, 1, 0)) {
        Camera c;
        c.eye = eye_;
        c.target = target_;
        c.fov = fov_;
        c.up = up_;
        return c;
    }

    // Blender → renderer coordinate conversion (Z-up to Y-up)
    static Camera fromBlender(Vector3f blenderEye, Vector3f blenderTarget, float fov_) {
        Camera c;
        c.eye = Vector3f(blenderEye.x, blenderEye.z, -blenderEye.y);
        c.target = Vector3f(blenderTarget.x, blenderTarget.z, -blenderTarget.y);
        c.fov = fov_;
        c.up = Vector3f(0, 1, 0);
        return c;
    }

    // ── Initialise derived basis ───────────────────────────────────────────────
    // Call after setting all parameters.

    void init(int width, int height) {
        forward = normalize(target - eye);
        right = normalize(crossProduct(forward, up));
        camUp = crossProduct(right, forward);
        scale = std::tan(fov * M_PI / 180.f * 0.5f);
        aspectRatio = (float)width / (float)height;

        // Auto focus distance — defaults to distance to target
        if (focusDistance < 0.f) {
            Vector3f d = target - eye;
            focusDistance = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        }

        printf("[Camera] eye=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) "
               "fov=%.1f aperture=%.4f focusDist=%.2f\n",
               eye.x, eye.y, eye.z, target.x, target.y, target.z, fov, aperture, focusDistance);
    }

    // ── Generate a ray for pixel (i, j) ───────────────────────────────────────
    //
    // xOffset, yOffset: jitter in [0, 1) for anti-aliasing
    //
    // Thin lens math:
    //   1. Find the point on the focus plane that the pinhole ray would hit
    //   2. Sample a random point on the lens disc
    //   3. Fire a ray from the lens point toward the focus point
    //      — all rays through the same focus point converge, so that point
    //        is sharp; everything else blurs proportional to aperture

    Ray generateRay(int i, int j, int W, int H, float xOffset, float yOffset) const {
        // NDC pixel direction (pinhole)
        float x = (2.f * ((float)i + xOffset) / (float)W - 1.f) * scale * aspectRatio;
        float y = (1.f - 2.f * ((float)j + yOffset) / (float)H) * scale;
        Vector3f pinholeDir = normalize(forward + x * right + y * camUp);

        if (aperture <= 0.f) {
            // Pinhole — no DoF
            return Ray(eye, pinholeDir);
        }

        // Point on the focus plane that the pinhole ray passes through
        Vector3f focusPoint = eye + pinholeDir * (focusDistance / dotProduct(pinholeDir, forward));

        // Sample random point on lens disc (rejection sampling — simple and correct)
        Vector3f lensOffset;
        do {
            float u = get_random_float() * 2.f - 1.f;
            float v = get_random_float() * 2.f - 1.f;
            lensOffset = Vector3f(u, v, 0);
        } while (lensOffset.x * lensOffset.x + lensOffset.y * lensOffset.y > 1.f);

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
    float scale = 1.f;
    float aspectRatio = 1.f;
};