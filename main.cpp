#include "Integrator.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include "ToneMapping.hpp"
#include "scenes/museum_final.cpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <ostream>
#include <string>

int main(int argc, char **argv) {
    RenderSettings settings;
    settings.width = 1920 / 2;
    settings.height = 1080 / 2;
    settings.minSPP = 1;
    settings.maxSPP = 4096 / 64;
    settings.russianRoulette = 0.95f;
    settings.maxDepth = 12;
    settings.varianceThreshold = 0.01f;
    settings.exposure = 1.0f;
    settings.toneMapper = tonemap::ToneMapper::AgX;

    settings.shadows = true;

    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--no-shadows") {
            settings.shadows = false;
            fprintf(stderr, "[main] SHADOWS OFF\n");
        } else if (arg == "--light-brightness" && i + 1 < argc) {
            settings.lightBrightness = std::atof(argv[++i]);
            fprintf(stderr, "[main] LIGHT BRIGHTNESS SET TO %f\n", settings.lightBrightness);

        } else if (arg == "--env-dark") {
            settings.envMap = "dark";
            fprintf(stderr, "[main] ENV MAP SET TO DARK\n");
        }
    }

    Scene scene = buildScene();

    // scene.camera.aperture = 0.003;
    scene.camera.init(settings.width, settings.height);

    // Integrator integrator = Integrator(scene, settings.maxDepth, settings.russianRoulette);
    Integrator integrator =
        Integrator(scene, settings.maxDepth, settings.russianRoulette, settings.shadows);

    auto start = std::chrono::system_clock::now();

    scene.buildBVH();
    // scene.buildPhotonMaps(1e6);

    Renderer r;

    r.Render(scene, integrator, settings);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: "
              << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : "
              << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count()
              << " minutes\n";
    std::cout << "          : "
              << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count()
              << " seconds\n";

    return 0;
}