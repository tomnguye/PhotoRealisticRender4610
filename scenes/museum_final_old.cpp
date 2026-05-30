#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include <Camera.hpp>

static Scene buildScene() {
    auto camera =
        Camera::fromBlender(Vector3f(12.6561f, 4.8860f, -3.1668f), // eye    (Blender coords)
                            Vector3f(3.5889f, 0.7598f, -2.2953f),  // target (Blender coords)
                            Vector3f(0.0793f, 0.0361f, 0.9962f),   // up     (Blender coords)
                            61.9275f,                              // fov_h (degrees)
                            37.2991f,                              // fov_v (degrees)
                            13.9312f // focusDistance (from 'dinosaur')
        );

    // auto camera =
    //     Camera::fromBlender(Vector3f(-1.5831f, 6.3097f, -2.9150f),  // eye    (Blender coords)
    //                         Vector3f(-1.2663f, -3.5621f, -1.3507f), // target (Blender coords)
    //                         Vector3f(-0.0050f, 0.1564f, 0.9877f),   // up     (Blender coords)
    //                         83.9744f,                               // fov_h (degrees)
    //                         53.7016f,                               // fov_v (degrees)
    //                         7.1285f                                 // focusDistance (from
    //                         'amber')
    //     );

    Scene scene;
    scene.camera = camera;

    scene.envMap.load("../assets/hdri/lonely_road_afternoon_puresky_8k.hdr");

    for (auto *mesh : GLTFLoader::load("../assets/museum_final/amber.glb", MaterialType::Glass)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/barrier.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/base.glb", MaterialType::Mirror))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/base_001.glb", MaterialType::Mirror))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/Cube.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    // for (auto *mesh :
    //      GLTFLoader::load("../assets/museum_final/dinosaur.glb", MaterialType::Diffuse))
    //     scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_a.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_b.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_b_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_c.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_c_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_d.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/floor.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/glass_case.glb", MaterialType::Glass))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/glass_case_001.glb", MaterialType::Glass))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/mosquito.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/Object_2.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/pedestal.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/pedestal_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/pillars.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/plaque.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/plaque_001.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/plateosaurus.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/platform.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/platform_002.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/platform2.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/puddle.glb", MaterialType::Glass)) {
        mesh->m->ior = 1.3;
        mesh->m->roughness = 0.02;
        mesh->m->metallic = 0;
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock07.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock07_001.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock08.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock09.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock09_001.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock10.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock11.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock12.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock13.glb",
                                       MaterialType::Diffuse))
        scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/sphere_gltf.glb", MaterialType::Diffuse))
        scene.Add(mesh);
    // for (auto *mesh :
    //      GLTFLoader::load("../assets/museum_final/tree_small_02_LOD0.glb",
    //      MaterialType::Diffuse))
    //     scene.Add(mesh);
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/wet_floor_sign.glb", MaterialType::Diffuse))
        scene.Add(mesh);

    return scene;
}
