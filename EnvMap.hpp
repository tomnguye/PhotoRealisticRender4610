#pragma once

#include "Vector.hpp"
#include "global.hpp"
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

// stb_image is bundled with tinygltf — just needs this define once
// #define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// ─── EnvMap ───────────────────────────────────────────────────────────────────
//
// Equirectangular HDR environment map with 2D importance sampling.
//
// Coordinate convention:
//   phi   = atan2(dir.z, dir.x)  in [-PI, PI]
//   theta = acos(dir.y)          in [0,   PI]
//   u     = (phi + PI) / (2*PI)  in [0,   1]    (no V flip — equirect not glTF)
//   v     = theta / PI           in [0,   1]
//
// Sampling strategy:
//   Build a 2D CDF over pixel luminance at load time.
//   At sample time invert the marginal CDF (which row) then the conditional
//   CDF (which column within that row). This concentrates samples on bright
//   regions (sun, lamps) and avoids wasting them on dark sky.

struct EnvMap {
    std::vector<Vector3f> pixels; // linear HDR, row-major
    int width = 0;
    int height = 0;
    float totalLuminance = 0.f;

    // 2D CDF — stored flat, size = width * height
    // marginalCDF[row] = CDF over row averages (length = height)
    std::vector<float> cdf;         // conditional CDF per row, flattened
    std::vector<float> marginalCdf; // CDF over rows

    bool loaded = false;

    // ── Load ──────────────────────────────────────────────────────────────────

    bool load(const std::string &path) {
        int w, h, ch;
        float *data = stbi_loadf(path.c_str(), &w, &h, &ch, 3);
        if (!data) {
            fprintf(stderr, "[EnvMap] Failed to load: %s\n", path.c_str());
            return false;
        }

        width = w;
        height = h;
        pixels.resize(w * h);

        for (int i = 0; i < w * h; i++)
            pixels[i] = Vector3f(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);

        stbi_image_free(data);
        buildCDF();
        loaded = true;
        printf("[EnvMap] Loaded %s (%dx%d) totalLuminance=%.2f\n", path.c_str(), w, h, totalLuminance);
        return true;
    }

    bool empty() const { return !loaded; }

    // ── Direction <-> UV conversion ───────────────────────────────────────────

    static Vector2f dirToUV(const Vector3f &dir) {
        float phi = std::atan2(dir.z, dir.x);
        float theta = std::acos(std::max(-1.f, std::min(1.f, dir.y)));
        return Vector2f((phi + M_PI) / (2.f * M_PI), theta / M_PI);
    }

    static Vector3f uvToDir(float u, float v) {
        float phi = u * 2.f * M_PI - M_PI;
        float theta = v * M_PI;
        float sinT = std::sin(theta);
        return Vector3f(sinT * std::cos(phi), std::cos(theta), sinT * std::sin(phi));
    }

    // ── Sample environment in a given direction ───────────────────────────────

    Vector3f sample(const Vector3f &dir) const {
        if (!loaded) return Vector3f(0);
        Vector2f uv = dirToUV(dir);

        float px = std::max(0.f, uv.x * (width - 1));
        float py = std::max(0.f, uv.y * (height - 1));
        int x0 = (int)px, y0 = (int)py;
        int x1 = std::min(x0 + 1, width - 1);
        int y1 = std::min(y0 + 1, height - 1);
        float fx = px - x0, fy = py - y0;

        auto fetch = [&](int x, int y) { return pixels[y * width + x]; };
        Vector3f top = fetch(x0, y0) + (fetch(x1, y0) - fetch(x0, y0)) * fx;
        Vector3f bottom = fetch(x0, y1) + (fetch(x1, y1) - fetch(x0, y1)) * fx;
        return top + (bottom - top) * fy;
    }

    // PDF of sampling a given direction under importance sampling
    float importanceSamplePdf(const Vector3f &dir) const {
        Vector2f uv = dirToUV(dir);
        int col = std::min((int)(uv.x * width), width - 1);
        int row = std::min((int)(uv.y * height), height - 1);
        float theta = uv.y * M_PI;
        float sinT = std::max(1e-4f, std::sin(theta));
        float lum = luminance(pixels[row * width + col]);
        return (lum / (totalLuminance + 1e-6f)) * ((float)(width * height) / (2.f * M_PI * M_PI * sinT));
    }

    // ── Importance sample — returns a world-space direction and its PDF ────────
    //
    // Draw two uniform random numbers, invert the 2D CDF to get (u,v),
    // convert to direction, compute the solid-angle PDF.

    Vector3f importanceSample(float &pdf) const {
        float r1 = get_random_float();
        float r2 = get_random_float();

        // 1. Sample row via marginal CDF
        int row = (int)(std::lower_bound(marginalCdf.begin(), marginalCdf.end(), r1) - marginalCdf.begin());
        row = std::min(row, height - 1);

        // 2. Sample column via conditional CDF for that row
        auto rowBegin = cdf.begin() + row * width;
        auto rowEnd = rowBegin + width;
        int col = (int)(std::lower_bound(rowBegin, rowEnd, r2) - rowBegin);
        col = std::min(col, width - 1);

        // 3. Convert (col, row) to UV then direction
        float u = (col + 0.5f) / (float)width;
        float v = (row + 0.5f) / (float)height;
        Vector3f dir = uvToDir(u, v);

        // 4. Compute solid-angle PDF
        // p(u,v) = luminance / totalLuminance
        // p(dir) = p(u,v) * (width * height) / (2 * PI * PI * sin(theta))
        float theta = v * M_PI;
        float sinT = std::max(1e-4f, std::sin(theta));
        float lum = luminance(pixels[row * width + col]);
        pdf = (lum / (totalLuminance + 1e-6f)) * ((float)(width * height) / (2.f * M_PI * M_PI * sinT));
        pdf = std::max(1e-6f, pdf);

        return dir;
    }

private:
    // ── CDF construction ──────────────────────────────────────────────────────

    static float luminance(const Vector3f &c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; }

    void buildCDF() {
        cdf.resize(width * height);
        marginalCdf.resize(height);

        std::vector<float> rowSums(height, 0.f);

        for (int row = 0; row < height; row++) {
            // sin(theta) weighting — pixels near poles subtend less solid angle
            float theta = (row + 0.5f) / (float)height * M_PI;
            float sinT = std::sin(theta);

            // Build conditional CDF for this row
            float rowSum = 0.f;
            for (int col = 0; col < width; col++) {
                float lum = luminance(pixels[row * width + col]) * sinT;
                rowSum += lum;
                cdf[row * width + col] = rowSum;
            }

            // Normalise row CDF to [0, 1]
            if (rowSum > 0.f)
                for (int col = 0; col < width; col++)
                    cdf[row * width + col] /= rowSum;

            rowSums[row] = rowSum;
        }

        // Build marginal CDF over rows
        float total = 0.f;
        for (int row = 0; row < height; row++) {
            total += rowSums[row];
            marginalCdf[row] = total;
        }

        totalLuminance = total;

        // Normalise marginal CDF to [0, 1]
        if (total > 0.f)
            for (float &v : marginalCdf)
                v /= total;
    }
};