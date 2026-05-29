#include "Renderer.hpp"
#include "Integrator.hpp"
#include "Material.hpp"
#include "Scene.hpp"
#include "stb_image_write.h"
#include <atomic>
#include <fstream>
#include <sstream>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

struct Tile
{
    int x, y; // top left pixel
    int w, h; // tile dimensions
};

const int TILE_SIZE = 32;

// 3x3 matrix * vector (row-major coefficients, same convention as the GLSL source)
static Vector3f mat3Mul(const float m[3][3], Vector3f v)
{
    return Vector3f(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
}

// The exact AgX sigmoid curve (analytical, not a polynomial fit)
static Vector3f agxCurve(Vector3f v)
{
    // Piecewise parameters split at the midpoint (threshold = 20/33)
    const float threshold = 0.6060606060606061f;
    const float a_up = 69.86278913545539f, a_down = 59.507875f;
    const float b_up = 13.f / 4.f, b_down = 3.f / 1.f;
    const float c_up = -4.f / 13.f, c_down = -1.f / 3.f;

    auto curve1 = [&](float x) -> float
    {
        bool below = (x <= threshold);
        float a = below ? a_up : a_down;
        float b = below ? b_up : b_down;
        float c = below ? c_up : c_down;
        float d = x - threshold;
        return 0.5f + (2.f * x - 2.f * threshold) * std::pow(1.f + a * std::pow(std::abs(d), b), c);
    };

    return Vector3f(curve1(v.x), curve1(v.y), curve1(v.z));
}
static Vector3f agxPunchyLook(Vector3f c)
{
    // BT.2020 luminance coefficients (Blender uses BT.2020 CIE-2012 luma here).
    float L = 0.2658f * c.x + 0.59846f * c.y + 0.13503f * c.z;
    const float sat = 1.4f;
    const float power = 1.35f;
    c.x = L + sat * (c.x - L);
    c.y = L + sat * (c.y - L);
    c.z = L + sat * (c.z - L);
    c.x = std::pow(std::max(0.f, c.x), power);
    c.y = std::pow(std::max(0.f, c.y), power);
    c.z = std::pow(std::max(0.f, c.z), power);
    return c;
}

static Vector3f agxTonemap(Vector3f ci, bool agxPunchy = true)
{
    const float min_ev = -12.473931188332413f;
    const float max_ev = 4.026068811667588f;
    const float dynamic_range = max_ev - min_ev;

    const float agx_mat[3][3] = {
        {0.544814746488245f, 0.373787398372697f, 0.0813978551390581f},
        {0.140416948464053f, 0.754137554567394f, 0.105445496968552f},
        {0.0888104196149096f, 0.178871756420858f, 0.732317823964232f}};

    const float agx_mat_inv[3][3] = {
        {1.96488741169489f, -0.855988495690215f, -0.108898916004672f},
        {-0.299313364904742f, 1.32639796461980f, -0.0270845997150571f},
        {-0.164352742528393f, -0.238183969428088f, 1.40253671195648f}};

    ci = mat3Mul(agx_mat, ci);

    ci.x = std::max(ci.x, 1e-10f);
    ci.y = std::max(ci.y, 1e-10f);
    ci.z = std::max(ci.z, 1e-10f);

    Vector3f ct;
    ct.x = std::clamp((std::log2(ci.x) - min_ev) / dynamic_range, 0.f, 1.f);
    ct.y = std::clamp((std::log2(ci.y) - min_ev) / dynamic_range, 0.f, 1.f);
    ct.z = std::clamp((std::log2(ci.z) - min_ev) / dynamic_range, 0.f, 1.f);

    Vector3f co = agxCurve(ct);

    // Punchy look operates on the formed AgX Base, before outset + encode.
    if (agxPunchy)
        co = agxPunchyLook(co);

    co = mat3Mul(agx_mat_inv, co);

    co.x = std::clamp(co.x, 0.f, 1.f);
    co.y = std::clamp(co.y, 0.f, 1.f);
    co.z = std::clamp(co.z, 0.f, 1.f);
    return co;
}

void Renderer::Render(const Scene &scene, const Integrator &integrator,
                      const RenderSettings &settings)
{
    const int W = settings.width;
    const int H = settings.height;
    const int totalPixels = W * H;
    const int minSamples = settings.minSPP;
    const int maxSamples = settings.maxSPP;
    const float threshold = settings.varianceThreshold;
    const float exposure = settings.exposure;

    std::vector<Vector3f> framebuffer(totalPixels, Vector3f(0));
    std::vector<int> sampleCount(totalPixels, 0);

    const int tilesX = (W + TILE_SIZE - 1) / TILE_SIZE;
    const int tilesY = (H + TILE_SIZE - 1) / TILE_SIZE;
    const int totalTiles = tilesX * tilesY;

    std::vector<Tile> tiles;
    tiles.reserve(totalTiles);
    for (int ty = 0; ty < tilesY; ty++)
    {
        for (int tx = 0; tx < tilesX; tx++)
        {
            Tile t;
            t.x = tx * TILE_SIZE;
            t.y = ty * TILE_SIZE;
            t.w = std::min(TILE_SIZE, W - t.x);
            t.h = std::min(TILE_SIZE, H - t.y);
            tiles.push_back(t);
        }
    }

    std::cout << "SPP: min=" << minSamples << " max=" << maxSamples << " threshold=" << threshold
              << "\n";
    std::cout << "Tiles: " << totalTiles << " (" << tilesX << "x" << tilesY << " @ " << TILE_SIZE
              << "px)\n";

    std::atomic<int> tilesDone(0);

    // We use tile based sampling to help distribute compute across threads.
#pragma omp parallel for schedule(dynamic)
    for (int tileIdx = 0; tileIdx < totalTiles; tileIdx++)
    {
        const Tile &tile = tiles[tileIdx];

        for (int j = tile.y; j < tile.y + tile.h; j++)
        {
            for (int i = tile.x; i < tile.x + tile.w; i++)
            {
                int m = i + j * W;

                Welford wR, wG, wB;
                int n = 0;

                while (n < maxSamples)
                {
                    float xOffset = get_random_float();
                    float yOffset = get_random_float();

                    Ray ray = scene.camera.generateRay(i, j, W, H, xOffset, yOffset);
                    Vector3f s = integrator.castRay(ray);
                    if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z))
                    {
                        printf("NaN at pixel (%d, %d)\n", i, j);
                        s = Vector3f(0.f);
                    }

                    framebuffer[m] += s;
                    n++;

                    wR.update(s.x);
                    wG.update(s.y);
                    wB.update(s.z);

                    if (n >= minSamples)
                    {
                        if (wR.ci() < threshold && wG.ci() < threshold && wB.ci() < threshold)
                        {
                            break;
                        }
                    }
                }

                framebuffer[m] = framebuffer[m] / (float)n;
                sampleCount[m] = n;
            }
        }

        int done = ++tilesDone;
        if (done % std::max(1, totalTiles / 100) == 0)
        {
#pragma omp critical
            UpdateProgress((float)done / (float)totalTiles);
        }
    }
    UpdateProgress(1.f);

    // Adaptive sampling statistics.
    int totalSamples = 0;
    int minS = maxSamples, maxS = 0;
    for (int s : sampleCount)
    {
        totalSamples += s;
        minS = std::min(minS, s);
        maxS = std::max(maxS, s);
    }
    printf("Adaptive sampling: min=%d max=%d avg=%.1f (budget=%d)\n", minS, maxS,
           (float)totalSamples / (float)totalPixels, maxSamples);

    std::vector<unsigned char> pngBuffer(totalPixels * 3);
    for (int i = 0; i < totalPixels; i++)
    {
        Vector3f raw = framebuffer[i] * settings.exposure;

        if (!std::isfinite(raw.x))
            raw.x = 0.f;
        if (!std::isfinite(raw.y))
            raw.y = 0.f;
        if (!std::isfinite(raw.z))
            raw.z = 0.f;
        raw.x = std::max(0.f, raw.x);
        raw.y = std::max(0.f, raw.y);
        raw.z = std::max(0.f, raw.z);
        auto linearToSRGB = [](float x) -> float
        {
            x = std::clamp(x, 0.f, 1.f);
            return x <= 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1.f / 2.4f) - 0.055f;
        };

        auto encode = [](float x) -> float
        {
            // AgX output is in a 2.2 power space: de-gamma to linear...
            float lin = std::pow(std::clamp(x, 0.f, 1.f), 2.2f);
            // ...then apply the sRGB OETF for the 8-bit PNG.
            return lin <= 0.0031308f ? 12.92f * lin
                                     : 1.055f * std::pow(lin, 1.f / 2.4f) - 0.055f;
        };

        // Vector3f co = Vector3f(linearToSRGB(raw.x), linearToSRGB(raw.y), linearToSRGB(raw.z));
        // float r = linearToSRGB(raw.x), g = linearToSRGB(raw.y), b = linearToSRGB(raw.z);

        Vector3f co = agxTonemap(raw, true); // AgX curve result, 2.2-encoded space
        float r = encode(co.x), g = encode(co.y), b = encode(co.z);

        pngBuffer[i * 3 + 0] = (unsigned char)(std::clamp(r, 0.f, 1.f) * 255.f + 0.5f);
        pngBuffer[i * 3 + 1] = (unsigned char)(std::clamp(g, 0.f, 1.f) * 255.f + 0.5f);
        pngBuffer[i * 3 + 2] = (unsigned char)(std::clamp(b, 0.f, 1.f) * 255.f + 0.5f);
    }

    float avgR = 0, avgG = 0, avgB = 0;
    float maxR = 0, maxG = 0, maxB = 0;
    for (int i = 0; i < totalPixels; i++)
    {
        avgR += framebuffer[i].x;
        avgG += framebuffer[i].y;
        avgB += framebuffer[i].z;
        maxR = std::max(maxR, framebuffer[i].x);
        maxG = std::max(maxG, framebuffer[i].y);
        maxB = std::max(maxB, framebuffer[i].z);
    }
    avgR /= totalPixels;
    avgG /= totalPixels;
    avgB /= totalPixels;
    printf("Avg radiance: (%.4f, %.4f, %.4f)\n", avgR, avgG, avgB);
    printf("Max radiance: (%.4f, %.4f, %.4f)\n", maxR, maxG, maxB);

    // PPM
    FILE *fp = fopen("output.ppm", "wb");
    fprintf(fp, "P6\n%d %d\n255\n", W, H);
    for (int i = 0; i < totalPixels; i++)
        fwrite(&pngBuffer[i * 3], 1, 3, fp);
    fclose(fp);

    // PNG
    stbi_write_png("output.png", W, H, 3, pngBuffer.data(), W * 3);

    // Adaptive sampling visualisation
    std::vector<unsigned char> sampleBuffer(totalPixels * 3);
    for (int i = 0; i < totalPixels; i++)
    {
        unsigned char v = (unsigned char)(255 * (float)sampleCount[i] / (float)maxSamples);
        sampleBuffer[i * 3 + 0] = v;
        sampleBuffer[i * 3 + 1] = v;
        sampleBuffer[i * 3 + 2] = v;
    }
    FILE *fp2 = fopen("sample_count.ppm", "wb");
    fprintf(fp2, "P6\n%d %d\n255\n", W, H);
    for (int i = 0; i < totalPixels; i++)
        fwrite(&sampleBuffer[i * 3], 1, 3, fp2);
    fclose(fp2);
    stbi_write_png("sample_count.png", W, H, 3, sampleBuffer.data(), W * 3);
}
