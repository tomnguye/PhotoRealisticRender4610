#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include <Camera.hpp>

static Scene buildScene() {
    auto camera =
        Camera::fromBlender(Vector3f(-4.3986f, 0.4373f, 2.2567f), // eye    (Blender coords)
                            Vector3f(2.5388f, -6.7610f, 2.0174f), // target (Blender coords)
                            Vector3f(0.0166f, -0.0172f, 0.9997f), // up     (Blender coords)
                            83.9744f,                             // fov_h (degrees)
                            53.7016f,                             // fov_v (degrees)
                            4.9630f // focusDistance (from '5_amber_lr')
        );

    Scene scene;
    scene.camera = camera;

    scene.envMap.load("../assets/hdri/lonely_road_afternoon_puresky_8k.hdr");

    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/2_mosquito_lr_original_o_material_0_0.glb",
                          MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/5_amber_lr_PBR_0.glb", MaterialType::Glass)) {
        mesh->m->roughness = 0.05;
        mesh->m->ior = 1.439;
        mesh->m->metallic = 0;
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/6_eclats_eclats_0.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/blender_models/Cube.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/blender_models/floor.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/glass_case.glb", MaterialType::Glass))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/Object_10.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/Object_8.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/Object_9.glb", MaterialType::Mirror)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/blender_models/window.glb", MaterialType::Glass))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/window_001.glb", MaterialType::Glass))
        scene.Add(mesh);

    return scene;
}
