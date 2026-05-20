#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include "scenes/cornellbox.cpp"
#include <chrono>

int main(int argc, char **argv) {
    RenderSettings settings;
    settings.width = 960;
    settings.height = 540;
    settings.minSPP = 64;
    settings.maxSPP = 64;
    settings.russianRoulette = 0.95f;
    settings.maxDepth = 30;
    settings.varianceThreshold = 0.05f;
    settings.exposure = 0.18f;

    Scene scene = buildCornellBox();
    scene.camera.aperture = 0.05f;
    scene.camera.focusDistance = 9.2981;
    scene.camera.init(settings.width, settings.height);

    Integrator integrator = Integrator(scene, settings.maxDepth, settings.russianRoulette);

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