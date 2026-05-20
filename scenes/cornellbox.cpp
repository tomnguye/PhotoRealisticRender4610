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

    Material *pink = new Material(DIFFUSE, Vector3f(0.75f, 0.42f, 0.42f));
    Material *blue = new Material(DIFFUSE, Vector3f(0.50f, 0.45f, 0.70f));
    Material *white = new Material(DIFFUSE, Vector3f(0.78f, 0.78f, 0.78f));
    Material *light = new Material(EMIT, Vector3f(1));
    light->m_emission = Vector3f(10.0f);

    for (auto *mesh : GLTFLoader::load("../assets/models/floor.glb", white))
        scene.Add(mesh);

    for (auto *mesh :
         GLTFLoader::load("../assets/models/shortbox.glb", new Material(MIRROR, Vector3f(1))))
        scene.Add(mesh);

    for (auto *mesh : GLTFLoader::load("../assets/models/tallbox.glb", white))
        scene.Add(mesh);

    for (auto *mesh : GLTFLoader::load("../assets/models/left.glb", pink))
        scene.Add(mesh);

    for (auto *mesh : GLTFLoader::load("../assets/models/right.glb", blue))
        scene.Add(mesh);

    for (auto *mesh : GLTFLoader::load("../assets/models/light.glb", light))
        scene.Add(mesh);

    for (auto *mesh : GLTFLoader::load("../assets/models/metal_toolbox_4k.glb"))
        scene.Add(mesh);

    scene.Add(new Sphere(Vector3f(4.50f, 1.0f, 1.00f), 1.0f, new Material(GLASS, Vector3f(1))));

    return scene;
}