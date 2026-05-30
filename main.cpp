#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
// #include "scenes/cornellbox.cpp"
// #include "scenes/test.cpp"
#include "scenes/museum_final.cpp"
#include <chrono>

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

    Scene scene = buildScene();

    // scene.camera.aperture = 0.003;
    scene.camera.init(settings.width, settings.height);
    printf("forward: (%.3f, %.3f, %.3f)\n", scene.camera.forward.x, scene.camera.forward.y,
           scene.camera.forward.z);
    printf("right:   (%.3f, %.3f, %.3f)\n", scene.camera.right.x, scene.camera.right.y,
           scene.camera.right.z);
    printf("camUp:   (%.3f, %.3f, %.3f)\n", scene.camera.camUp.x, scene.camera.camUp.y,
           scene.camera.camUp.z);

    // scene.backgroundColor = Vector3f(3.0f);
    scene.medium.sigma_t = -1;
    // scene.medium = Medium::dust();
    // scene.medium.bounded = true;
    // scene.medium.bounds = Bounds3(Vector3f(-10.0f, -5.0f, -10.0f), Vector3f(10.0f, 5.0f, 10.0f));

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