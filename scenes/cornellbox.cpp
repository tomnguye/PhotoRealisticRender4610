#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"

Scene buildCornellBox() {
    Camera camera = Camera::create(Vector3f(2.78f, 2.73f, -8.0f), Vector3f(2.78f, 2.73f, 1),
                                   Vector3f(0, 1, 0), 40.0f);
    Scene scene;
    scene.camera = camera;
    scene.envMap.load("../assets/hdri/qwantani_dusk_2_puresky_4k.hdr");
    scene.backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);

    auto *pink = new DiffuseMaterial();
    pink->baseColor = Vector3f(0.75f, 0.42f, 0.42f);

    auto *blue = new DiffuseMaterial();
    blue->baseColor = Vector3f(0.50f, 0.45f, 0.70f);

    auto *white = new DiffuseMaterial();
    white->baseColor = Vector3f(0.78f, 0.78f, 0.78f);

    auto *light = new EmissiveMaterial();
    light->m_emission = Vector3f(10.0f);

    auto *mirror = new MirrorMaterial();
    mirror->baseColor = Vector3f(1.0f);

    auto *glass = new GlassMaterial();
    glass->baseColor = Vector3f(1.0f);

    // for (auto *mesh : GLTFLoader::load("../assets/models/floor.glb", white))
    //     scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/models/shortbox.glb", mirror))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/models/tallbox.glb", white))
        scene.Add(mesh);
    // for (auto *mesh : GLTFLoader::load("../assets/models/left.glb", pink))
    //     scene.Add(mesh);
    // for (auto *mesh : GLTFLoader::load("../assets/models/right.glb", blue))
    //     scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/models/light.glb", light))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/models/metal_toolbox_4k.glb"))
        scene.Add(mesh);

    scene.Add(new Sphere(Vector3f(4.50f, 1.0f, 1.00f), 1.0f, glass));

    return scene;
}