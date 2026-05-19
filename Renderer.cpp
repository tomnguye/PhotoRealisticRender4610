#include "Renderer.hpp"
#include "Material.hpp"
#include "Scene.hpp"
#include <atomic>
#include <fstream>
#include <sstream>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

const float EPSILON = 1e-2;

// ─── Welford online variance tracker ─────────────────────────────────────────

struct Welford {
    int n = 0;
    float mean = 0.f;
    float M2 = 0.f;

    void update(float x) {
        n++;
        float delta = x - mean;
        mean += delta / (float)n;
        M2 += delta * (x - mean);
    }

    float variance() const { return n < 2 ? 0.f : M2 / (float)(n - 1); }

    // 95% confidence interval as fraction of mean — stop when below threshold
    float ci() const {
        if (n < 2 || std::abs(mean) < 1e-6f) return 1.f;
        return 1.96f * std::sqrt(variance() / (float)n) / std::abs(mean);
    }
};

// ─── Tile ─────────────────────────────────────────────────────────────────────

struct Tile {
    int x, y; // top-left pixel
    int w, h; // tile dimensions (may be smaller at edges)
};

// ─── Render ───────────────────────────────────────────────────────────────────

void Renderer::Render(const Scene &scene) {
    const int W = scene.width;
    const int H = scene.height;
    const int totalPixels = W * H;

    std::vector<Vector3f> framebuffer(totalPixels, Vector3f(0));
    std::vector<int> sampleCount(totalPixels, 0);

    // ── Adaptive sampling parameters ──────────────────────────────────────────
    const int minSamples = 64;
    const int maxSamples = scene.spp;
    const float threshold = scene.adaptiveThreshold;

    // ── Tile setup ────────────────────────────────────────────────────────────
    // 32x32 tiles — large enough to amortise overhead, small enough for
    // good load balancing across threads. Adjust if needed.
    const int TILE_SIZE = 32;
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

    std::cout << "SPP: min=" << minSamples << " max=" << maxSamples << " threshold=" << threshold << "\n";
    std::cout << "Tiles: " << totalTiles << " (" << tilesX << "x" << tilesY << " @ " << TILE_SIZE << "px)\n";

    std::atomic<int> tilesDone(0);

    // ── Render — one tile per thread iteration ────────────────────────────────
    // schedule(dynamic,1): tiles handed to threads on demand.
    // This is the key load balancing improvement — a thread that finishes a
    // cheap tile (sky, background) immediately picks up the next one rather
    // than waiting for a slow thread to finish a long scanline.

#pragma omp parallel for schedule(dynamic, 1) num_threads(20)
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
                    Vector3f s = scene.castRay(ray, 0);
                    float lum = 0.2126f * s.x + 0.7152f * s.y + 0.0722f * s.z;
                    if (lum > 20.f) s = s * (20.f / lum);
                    framebuffer[m] += s;

                    framebuffer[m] += s;
                    n++;

                    wR.update(s.x);
                    wG.update(s.y);
                    wB.update(s.z);

                    if (n >= minSamples) {
                        if (wR.ci() < threshold && wG.ci() < threshold && wB.ci() < threshold) break;
                    }
                }

                framebuffer[m] = framebuffer[m] / (float)n;
                sampleCount[m] = n;
            }
        }

        // Progress — updated per tile, not per pixel (much less contention)
        int done = ++tilesDone;
        if (done % std::max(1, totalTiles / 100) == 0) UpdateProgress((float)done / (float)totalTiles);
    }
    UpdateProgress(1.f);

    // ── Sample count stats ────────────────────────────────────────────────────
    int totalSamples = 0;
    int minS = maxSamples, maxS = 0;
    for (int s : sampleCount) {
        totalSamples += s;
        minS = std::min(minS, s);
        maxS = std::max(maxS, s);
    }
    printf("Adaptive sampling: min=%d max=%d avg=%.1f (budget=%d)\n", minS, maxS, (float)totalSamples / (float)totalPixels, maxSamples);

    // ── Tone mapping ──────────────────────────────────────────────────────────

    auto ACESFilm = [](float x) -> float {
        const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
        return std::clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.f, 1.f);
    };

    auto ACESToneMap = [&](Vector3f col) -> Vector3f {
        float r = col.x * 0.59719f + col.y * 0.35458f + col.z * 0.04823f;
        float g = col.x * 0.07600f + col.y * 0.90834f + col.z * 0.01566f;
        float b = col.x * 0.02840f + col.y * 0.13383f + col.z * 0.83777f;
        r = ACESFilm(r);
        g = ACESFilm(g);
        b = ACESFilm(b);
        float ro = r * 1.60475f + g * -0.53108f + b * -0.07367f;
        float go = r * -0.10208f + g * 1.10813f + b * -0.00605f;
        float bo = r * -0.00327f + g * -0.07276f + b * 1.07602f;
        return Vector3f(std::clamp(ro, 0.f, 1.f), std::clamp(go, 0.f, 1.f), std::clamp(bo, 0.f, 1.f));
    };

    auto linearToSRGB = [](float x) -> float {
        x = std::max(0.f, x);
        return x <= 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1.f / 2.4f) - 0.055f;
    };

    // ── Write render output ───────────────────────────────────────────────────

    std::stringstream ss;
    ss << "binary_task" << TASK_N << ".ppm";
    FILE *fp = fopen(ss.str().c_str(), "wb");
    fprintf(fp, "P6\n%d %d\n255\n", W, H);
    for (int i = 0; i < totalPixels; i++) {
        Vector3f c = ACESToneMap(framebuffer[i] * scene.exposure);
        unsigned char px[3] = {(unsigned char)(255 * linearToSRGB(c.x)), (unsigned char)(255 * linearToSRGB(c.y)),
                               (unsigned char)(255 * linearToSRGB(c.z))};
        fwrite(px, 1, 3, fp);
    }
    fclose(fp);

    // ── Write sample count visualisation (white = max samples = noisiest) ─────
    FILE *fp2 = fopen("sample_count.ppm", "wb");
    fprintf(fp2, "P6\n%d %d\n255\n", W, H);
    for (int i = 0; i < totalPixels; i++) {
        unsigned char v = (unsigned char)(255 * (float)sampleCount[i] / (float)maxSamples);
        unsigned char px[3] = {v, v, v};
        fwrite(px, 1, 3, fp2);
    }
    fclose(fp2);
}