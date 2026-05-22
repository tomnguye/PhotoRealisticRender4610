
#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include <Camera.hpp>

static Scene buildHouse() {
    Camera::fromBlender(Vector3f(-4.399f, 0.437f, 2.257f), Vector3f(2.539f, -6.761f, 2.017f),
                        53.702f);

    Scene scene;
    scene.camera = camera;
    scene.envMap.load("../assets/hdri/lonely_road_afternoon_puresky_8k.hdr");
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

    for (auto *mesh :
         GLTFLoader::load("../models/gltf/house_shortbox.glb", MaterialType::Diffuse, white))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../models/gltf/house_tallbox.glb", MaterialType::Glass, glass))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../models/gltf/house.glb", MaterialType::Diffuse, white))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../models/gltf/floor.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../models/gltf/house_toolbox.glb", MaterialType::Diffuse))
        scene.Add(mesh);

    return scene;
}
