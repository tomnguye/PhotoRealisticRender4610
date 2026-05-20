#include "Renderer.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

float TASK_N = 2; // 1.1, 1.2, 1.3, 2

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char **argv) {
    if (argc >= 2)
        TASK_N = (float)atof(argv[1]);

    RenderSettings settings;
    settings.width = 960;
    settings.height = 540;
    settings.minSPP = 64;
    settings.maxSPP = 64;
    settings.russianRoulette = 0.95f;
    settings.maxDepth = 30;
    settings.varianceThreshold = 0.05f;
    settings.exposure = 0.18f;

    // Camera camera =  Camera::fromBlender(Vector3f(4.754f, 6.913f, 4.007f), Vector3f(2.015f,
    // -2.374f, 1.508f), 22.895f);
    Camera camera = Camera::create(Vector3f(2.78f, 2.73f, -8.0f), Vector3f(2.78f, 2.73f, 1),
                                   Vector3f(0, 1, 0), 40.0f);
    camera.aperture = 0.05f; // bigger = more blur
    camera.focusDistance = 9.2981;
    camera.init(settings.width, settings.height);

    Scene scene;
    scene.camera = camera;
    scene.envMap.load("../hdri/qwantani_dusk_2_puresky_4k.hdr");
    scene.backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);

    Integrator integrator = Integrator(scene, settings.maxDepth, settings.russianRoulette);

    Material *pink = new Material(DIFFUSE, Vector3f(0.75f, 0.42f, 0.42f));
    Material *blue = new Material(DIFFUSE, Vector3f(0.50f, 0.45f, 0.70f));
    Material *purple = new Material(DIFFUSE, Vector3f(0.73f, 0.33f, 0.83f));
    Material *green = new Material(DIFFUSE, Vector3f(0.35f, 0.85f, 0.35f));
    Material *white = new Material(DIFFUSE, Vector3f(0.78f, 0.78f, 0.78f));
    Material *light = new Material(EMIT, Vector3f(1));
    light->m_emission = Vector3f(10.0f);

    // MeshTriangle floor("../models/cornellbox/floor.obj", Vector3f(0), white);
    // MeshTriangle shortbox("../models/cornellbox/shortbox.obj", Vector3f(0), new Material(MIRROR,
    // Vector3f(1))); MeshTriangle tallbox("../models/cornellbox/tallbox.obj", Vector3f(0), new
    // Material(MIRROR, Vector3f(1))); MeshTriangle left("../models/cornellbox/left.obj",
    // Vector3f(0), pink); MeshTriangle right("../models/cornellbox/right.obj", Vector3f(0), blue);
    // MeshTriangle light_("../models/cornellbox/light.obj", Vector3f(0.0f, 0.0f, -10000.0f),
    // light); MeshTriangle light_back("../models/cornellbox/light.obj", Vector3f(0, -5, -500),
    // light);

    MeshTriangle floor("../models/gltf/floor.glb", white);
    Material *roughMirror = new Material(GLASS, Vector3f(0.95f));
    roughMirror->roughness = 0.05f;
    roughMirror->metallic = 1.0f; // full metallic — no diffuse lobe, Fresnel from F0
    MeshTriangle shortbox("../models/gltf/shortbox.glb", white);
    MeshTriangle tallbox("../models/gltf/tallbox.glb", white);
    MeshTriangle left("../models/gltf/left.glb", pink);
    MeshTriangle right("../models/gltf/right.glb", blue);
    MeshTriangle light_("../models/gltf/light.glb", light);
    // MeshTriangle light_back("../models/cornellbox/light.obj", light);

    MeshTriangle toolbox("../models/gltf/metal_toolbox_4k.glb");

    scene.Add(&toolbox);
    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    // scene.Add(&light_back);

    // scene.Add(new MeshTriangle("../models/spot/spot.obj", Vector3f(0), new Material(GLASS,
    // Vector3f(1))));

    scene.Add(new Sphere(Vector3f(4.50f, 1.0f, 1.00f), 1.0f, new Material(MIRROR, Vector3f(1))));

    // Vector3f verts[4] = {{0, 0, 0}, {552.8, 0, 0}, {549.6, 0, 559.2}, {0, 0, 559.2}};
    // Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    // uint32_t vertIndex[6] = {0, 2, 1, 2, 0, 3};
    // Material *mfloor = new Material(DIFFUSE, Vector3f(0));
    // mfloor->textured = true;
    // scene.Add(new MeshTriangle(verts, vertIndex, 2, st, mfloor));

    scene.buildBVH();

    auto start = std::chrono::system_clock::now();
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