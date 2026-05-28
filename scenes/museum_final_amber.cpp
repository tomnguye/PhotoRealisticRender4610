#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include <Camera.hpp>

static Scene buildScene() {
    auto camera =
        Camera::fromBlender(Vector3f(-1.5831f, 6.3097f, -2.9150f),  // eye    (Blender coords)
                            Vector3f(-1.2663f, -3.5621f, -1.3507f), // target (Blender coords)
                            Vector3f(-0.0050f, 0.1564f, 0.9877f),   // up     (Blender coords)
                            83.9744f,                               // fov_h (degrees)
                            53.7016f,                               // fov_v (degrees)
                            7.1285f                                 // focusDistance (from 'amber')
        );

    Scene scene;
    scene.camera = camera;

    scene.envMap.load("../assets/hdri/lonely_road_afternoon_puresky_8k.hdr");

    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/amber.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/barrier.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/base.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/base_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/Cube.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/dinosaur.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/fern_02_a.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/fern_02_b.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/fern_02_b_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/fern_02_c.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/fern_02_c_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/fern_02_d.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/floor.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/glass_case.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/glass_case_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/mosquito.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/Object_2.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/pedestal.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/pedestal_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/pillars.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/plaque.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/plaque_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/plateosaurus.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/platform.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/platform_002.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/platform2.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/puddle.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock07.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load(
             "../assets/museum_final_amber/rock_moss_set_02_rock07_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock08.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock09.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load(
             "../assets/museum_final_amber/rock_moss_set_02_rock09_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock10.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock11.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock12.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/rock_moss_set_02_rock13.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/sphere_gltf.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final_amber/tree_small_02_LOD0.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final_amber/wet_floor_sign.glb", MaterialType::Diffuse))
        scene.Add(mesh);

    return scene;
}
