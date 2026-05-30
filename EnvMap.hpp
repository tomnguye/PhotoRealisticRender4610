#pragma once

#include "Vector.hpp"
#include "global.hpp"
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include "stb_image.h"

// Equirectangular HDR environment map with 2D importance sampling.
// A 2D CDF over pixel luminance is built at load time. At sample time the
// marginal CDF selects a row and the conditional CDF selects a column within
// that row. This concentrates samples on bright regions and avoids wasting
// them on dark sky.

struct EnvMap {
    std::vector<Vector3f> pixels; // Linear HDR pixels, row-major.
    int width = 0;
    int height = 0;
    float totalLuminance = 0.f;

    std::vector<float> cdf;         // Conditional CDF per row, flattened to width * height.
    std::vector<float> marginalCdf; // CDF over row averages, length = height.

    bool loaded = false;

    /**
     * @brief Loads an equirectangular HDR image from disk and builds the importance sampling CDF.
     *
     * @param path Path to the .hdr file.
     * @return true if loading succeeded, false otherwise.
     */
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
        printf("[EnvMap] Loaded %s (%dx%d) totalLuminance=%.2f\n", path.c_str(), w, h,
               totalLuminance);
        return true;
    }

    /**
     * @brief Returns true if no environment map has been loaded.
     */
    bool empty() const {
        return !loaded;
    }

    /**
     * @brief Converts a world space direction to equirectangular UV coordinates.
     *
     * @param dir Normalised world space direction.
     * @return Vector2f UV in [0, 1] x [0, 1].
     */
    static Vector2f dirToUV(const Vector3f &dir) {
        float phi = std::atan2(dir.z, dir.x);
        float theta = std::acos(std::max(-1.f, std::min(1.f, dir.y)));
        return Vector2f((phi + M_PI) / (2.f * M_PI), theta / M_PI);
    }

    /**
     * @brief Converts equirectangular UV coordinates to a world space direction.
     *
     * @param u U coordinate in [0, 1].
     * @param v V coordinate in [0, 1].
     * @return Vector3f Normalised world space direction.
     */
    static Vector3f uvToDir(float u, float v) {
        float phi = u * 2.f * M_PI - M_PI;
        float theta = v * M_PI;
        float sinT = std::sin(theta);
        return Vector3f(sinT * std::cos(phi), std::cos(theta), sinT * std::sin(phi));
    }

    /**
     * @brief Samples the environment map for a given direction using bilinear filtering.
     *
     * @param dir Normalised world space direction.
     * @return Vector3f Linear HDR radiance.
     */
    Vector3f sample(const Vector3f &dir) const {
        if (!loaded)
            return Vector3f(0);

        Vector2f uv = dirToUV(dir);
        float px = std::max(0.f, uv.x * (width - 1));
        float py = std::max(0.f, uv.y * (height - 1));
        int x0 = (int) px, y0 = (int) py;
        int x1 = std::min(x0 + 1, width - 1);
        int y1 = std::min(y0 + 1, height - 1);
        float fx = px - x0, fy = py - y0;

        auto fetch = [&](int x, int y) { return pixels[y * width + x]; };
        Vector3f top = fetch(x0, y0) + (fetch(x1, y0) - fetch(x0, y0)) * fx;
        Vector3f bottom = fetch(x0, y1) + (fetch(x1, y1) - fetch(x0, y1)) * fx;
        return top + (bottom - top) * fy;
    }

    /**
     * @brief Evaluates the solid angle PDF for a given direction under importance sampling.
     *
     * Must be consistent with importanceSample(). Both use luminance / totalLuminance
     * weighted by the equirectangular solid angle correction (1 / sin(theta)).
     *
     * @param dir Normalised world space direction.
     * @return float Solid angle PDF for this direction.
     */
    float importanceSamplePdf(const Vector3f &dir) const {
        Vector2f uv = dirToUV(dir);
        int col = std::min((int) (uv.x * width), width - 1);
        int row = std::min((int) (uv.y * height), height - 1);
        float theta = uv.y * M_PI;
        float sinT = std::max(1e-4f, std::sin(theta));
        float lum = luminance(pixels[row * width + col]);
        return (lum * sinT / (totalLuminance + 1e-6f)) *
               ((float) (width * height) / (2.f * M_PI * M_PI * sinT));
    }

    /**
     * @brief Samples a direction from the environment map using importance sampling.
     *
     * Inverts the 2D CDF to select a row from the marginal CDF then a column
     * from the conditional CDF for that row. The returned PDF matches
     * importanceSamplePdf() evaluated at the returned direction.
     *
     * @param pdf Solid angle PDF of the sampled direction (out).
     * @return Vector3f Sampled world space direction.
     */
    Vector3f importanceSample(float &pdf) const {
        float r1 = get_random_float();
        float r2 = get_random_float();

        int row = (int) (std::lower_bound(marginalCdf.begin(), marginalCdf.end(), r1) -
                         marginalCdf.begin());
        row = std::min(row, height - 1);

        auto rowBegin = cdf.begin() + row * width;
        auto rowEnd = rowBegin + width;
        int col = (int) (std::lower_bound(rowBegin, rowEnd, r2) - rowBegin);
        col = std::min(col, width - 1);

        float u = (col + 0.5f) / (float) width;
        float v = (row + 0.5f) / (float) height;
        Vector3f dir = uvToDir(u, v);

        float theta = v * M_PI;
        float sinT = std::max(1e-4f, std::sin(theta));
        float lum = luminance(pixels[row * width + col]);
        pdf = (lum / (totalLuminance + 1e-6f)) * ((float) (width * height) / (2.f * M_PI * M_PI));
        pdf = std::max(1e-6f, pdf);

        return dir;
    }

  private:
    /**
     * @brief Returns the luminance of a linear RGB colour using Rec. 709 coefficients.
     *
     * @param c Linear RGB colour.
     * @return float Luminance.
     */
    static float luminance(const Vector3f &c) {
        return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
    }

    /**
     * @brief Builds the 2D importance sampling CDF from the loaded pixel data.
     *
     * Each row is weighted by sin(theta) to account for the reduced solid angle
     * near the poles of the equirectangular projection. The marginal CDF is built
     * from the sum of each weighted row.
     */
    void buildCDF() {
        cdf.resize(width * height);
        marginalCdf.resize(height);

        std::vector<float> rowSums(height, 0.f);

        for (int row = 0; row < height; row++) {
            float theta = (row + 0.5f) / (float) height * M_PI;
            float sinT = std::sin(theta);

            float rowSum = 0.f;
            for (int col = 0; col < width; col++) {
                float lum = luminance(pixels[row * width + col]) * sinT;
                rowSum += lum;
                cdf[row * width + col] = rowSum;
            }

            if (rowSum > 0.f)
                for (int col = 0; col < width; col++)
                    cdf[row * width + col] /= rowSum;

            rowSums[row] = rowSum;
        }

        float total = 0.f;
        for (int row = 0; row < height; row++) {
            total += rowSums[row];
            marginalCdf[row] = total;
        }

        totalLuminance = total;

        if (total > 0.f)
            for (float &v : marginalCdf)
                v /= total;
    }
};