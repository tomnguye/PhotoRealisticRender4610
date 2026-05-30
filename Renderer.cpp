#include "Renderer.hpp"
#include "Integrator.hpp"
#include "Material.hpp"
#include "Scene.hpp"
#include "ToneMapping.hpp"
#include "stb_image_write.h"
#include <atomic>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

struct Tile {
    int x, y; // top left pixel
    int w, h; // tile dimensions
};

const int TILE_SIZE = 32;

void Renderer::Render(const Scene &scene, const Integrator &integrator,
                      const RenderSettings &settings) {
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
    for (int ty = 0; ty < tilesY; ty++) {
        for (int tx = 0; tx < tilesX; tx++) {
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
    std::cout << "Tone mapper: " << tonemap::toString(settings.toneMapper) << "\n";

    std::atomic<int> tilesDone(0);

    // We use tile based sampling to help distribute compute across threads.
#pragma omp parallel for schedule(dynamic)
    for (int tileIdx = 0; tileIdx < totalTiles; tileIdx++) {
        const Tile &tile = tiles[tileIdx];

        for (int j = tile.y; j < tile.y + tile.h; j++) {
            for (int i = tile.x; i < tile.x + tile.w; i++) {
                int m = i + j * W;

                Welford wR, wG, wB;
                int n = 0;

                while (n < maxSamples) {
                    float xOffset = get_random_float();
                    float yOffset = get_random_float();

                    Ray ray = scene.camera.generateRay(i, j, W, H, xOffset, yOffset);
                    Vector3f s = integrator.castRay(ray);
                    if (std::isnan(s.x) || std::isnan(s.y) || std::isnan(s.z)) {
                        printf("NaN at pixel (%d, %d)\n", i, j);
                        s = Vector3f(0.f);
                    }

                    framebuffer[m] += s;
                    n++;

                    wR.update(s.x);
                    wG.update(s.y);
                    wB.update(s.z);

                    if (n >= minSamples) {
                        if (wR.ci() < threshold && wG.ci() < threshold && wB.ci() < threshold) {
                            break;
                        }
                    }
                }

                framebuffer[m] = framebuffer[m] / (float) n;
                sampleCount[m] = n;
            }
        }

        int done = ++tilesDone;
        if (done % std::max(1, totalTiles / 100) == 0) {
#pragma omp critical
            UpdateProgress((float) done / (float) totalTiles);
        }
    }
    UpdateProgress(1.f);

    // Adaptive sampling statistics.
    int totalSamples = 0;
    int minS = maxSamples, maxS = 0;
    for (int s : sampleCount) {
        totalSamples += s;
        minS = std::min(minS, s);
        maxS = std::max(maxS, s);
    }
    printf("Adaptive sampling: min=%d max=%d avg=%.1f (budget=%d)\n", minS, maxS,
           (float) totalSamples / (float) totalPixels, maxSamples);

    float avgR = 0, avgG = 0, avgB = 0;
    float maxR = 0, maxG = 0, maxB = 0;
    for (int i = 0; i < totalPixels; i++) {
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

    // -------------------------------------------------------------------------
    // Tone mapping + output.
    //
    // The selected operator is written to output.{png,ppm}. In addition, unless
    // the selected operator is already Raw, a true-Raw image (scene-linear, no
    // display transform) is written alongside it (suffixed "_raw"). Note Raw is
    // a diagnostic passthrough and will look dark on a normal display.
    // -------------------------------------------------------------------------

    // Apply exposure (in linear) and sanitise a single pixel's radiance.
    auto exposeLinear = [&](int i) -> Vector3f {
        Vector3f c = framebuffer[i] * exposure;
        if (!std::isfinite(c.x))
            c.x = 0.f;
        if (!std::isfinite(c.y))
            c.y = 0.f;
        if (!std::isfinite(c.z))
            c.z = 0.f;
        c.x = std::max(0.f, c.x);
        c.y = std::max(0.f, c.y);
        c.z = std::max(0.f, c.z);
        return c;
    };

    // Tone map the whole framebuffer with a given operator into an 8-bit buffer,
    // then write it as both PNG and PPM under the given base filename.
    auto renderAndWrite = [&](tonemap::ToneMapper mapper, const std::string &baseName) {
        std::vector<unsigned char> buf(totalPixels * 3);
        for (int i = 0; i < totalPixels; i++) {
            Vector3f co = tonemap::apply(mapper, exposeLinear(i));
            buf[i * 3 + 0] = (unsigned char) (std::clamp(co.x, 0.f, 1.f) * 255.f + 0.5f);
            buf[i * 3 + 1] = (unsigned char) (std::clamp(co.y, 0.f, 1.f) * 255.f + 0.5f);
            buf[i * 3 + 2] = (unsigned char) (std::clamp(co.z, 0.f, 1.f) * 255.f + 0.5f);
        }

        const std::string ppmName = baseName + ".ppm";
        const std::string pngName = baseName + ".png";

        FILE *fp = fopen(ppmName.c_str(), "wb");
        if (fp) {
            fprintf(fp, "P6\n%d %d\n255\n", W, H);
            for (int i = 0; i < totalPixels; i++)
                fwrite(&buf[i * 3], 1, 3, fp);
            fclose(fp);
        }
        stbi_write_png(pngName.c_str(), W, H, 3, buf.data(), W * 3);
    };

    // Primary output: the selected operator.
    renderAndWrite(settings.toneMapper, "output");

    // Companion Raw baseline, unless the selection is already Raw.
    if (settings.toneMapper != tonemap::ToneMapper::Raw)
        renderAndWrite(tonemap::ToneMapper::Raw, "output_raw");

    // -------------------------------------------------------------------------
    // Adaptive sampling visualisation
    // -------------------------------------------------------------------------
    std::vector<unsigned char> sampleBuffer(totalPixels * 3);
    for (int i = 0; i < totalPixels; i++) {
        unsigned char v = (unsigned char) (255 * (float) sampleCount[i] / (float) maxSamples);
        sampleBuffer[i * 3 + 0] = v;
        sampleBuffer[i * 3 + 1] = v;
        sampleBuffer[i * 3 + 2] = v;
    }
    FILE *fp2 = fopen("sample_count.ppm", "wb");
    if (fp2) {
        fprintf(fp2, "P6\n%d %d\n255\n", W, H);
        for (int i = 0; i < totalPixels; i++)
            fwrite(&sampleBuffer[i * 3], 1, 3, fp2);
        fclose(fp2);
    }
    stbi_write_png("sample_count.png", W, H, 3, sampleBuffer.data(), W * 3);
}