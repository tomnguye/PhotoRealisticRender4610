#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include <Camera.hpp>

static Scene buildScene() {

    // Cameras
    auto rexCamera =
        Camera::fromBlender(Vector3f(12.7000f, 4.5060f, -3.1668f), // eye    (Blender coords)
                            Vector3f(3.5993f, 0.4541f, -2.2953f),  // target (Blender coords)
                            Vector3f(0.0796f, 0.0354f, 0.9962f),   // up     (Blender coords)
                            73.7398f,                              // fov_h (degrees)
                            45.7473f,                              // fov_v (degrees)
                            17.1680f // focusDistance (from 'dinosaur')
        );

    auto amberCamera =
        Camera::fromBlender(Vector3f(-1.5831f, 6.3097f, -2.9150f),  // eye    (Blender coords)
                            Vector3f(-1.2663f, -3.5621f, -1.3507f), // target (Blender coords)
                            Vector3f(-0.0050f, 0.1564f, 0.9877f),   // up     (Blender coords)
                            83.9744f,                               // fov_h (degrees)
                            53.7016f,                               // fov_v (degrees)
                            0.4735f                                 // focusDistance (from 'amber')
        );

    auto wfsCamera =
        Camera::fromBlender(Vector3f(3.1671f, 3.2395f, -3.8770f),  // eye    (Blender coords)
                            Vector3f(-6.6850f, 3.8421f, -2.2733f), // target (Blender coords)
                            Vector3f(0.1601f, -0.0098f, 0.9871f),  // up     (Blender coords)
                            61.9275f,                              // fov_h (degrees)
                            37.2991f,                              // fov_v (degrees)
                            1.2005f // focusDistance (from 'wet_floor_sign')
        );

    // Set active camera
    Scene scene;
    scene.camera = rexCamera;

    // scene.envMap.load("../assets/hdri/lonely_road_afternoon_puresky_8k.hdr");
    scene.envMap.load("../assets/hdri/kloppenheim_02_puresky_4k.hdr");

    // Meshes
    // amber_exhibit
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/amber.glb", MaterialType::Glass)) {
        mesh->m->ior = 1.539;
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/base.glb", MaterialType::Mirror)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/glass_case.glb", MaterialType::Glass)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/mosquito.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/pedestal.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/plaque.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }

    // building
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/Cube.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/floor.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/pillars.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }

    // egg_exhibit
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/base_001.glb", MaterialType::Mirror)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/dinosaur_egg.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/glass_case_001.glb", MaterialType::Glass)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/pedestal_001.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/plaque_001.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }

    // extras
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/puddle.glb", MaterialType::Glass)) {
        mesh->m->ior = 1.3;
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/wet_floor_sign.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }

    // lights
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/spotlight.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/spotlight_001.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/spotlight_002.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/spotlight_light.glb", MaterialType::Emit)) {
        mesh->m->m_emission = 1000 * Vector3f(0.642f, 0.0f, 1.0f);
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/spotlight_light_001.glb", MaterialType::Emit)) {
        mesh->m->m_emission = 1000 * Vector3f(0.009f, 0.0f, 1.0f);
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/spotlight_light_002.glb", MaterialType::Emit)) {
        mesh->m->m_emission = 1000 * Vector3f(0.336f, 1.0f, 0.315f);
        scene.Add(mesh);
    }

    // plateo
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_a.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_b.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_b_001.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_c.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_c_001.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/fern_02_d.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/plateosaurus.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/platform2.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock07_001.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock09_001.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/sphere_gltf.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/tree_small_02_LOD0.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }

    // trex_exhibit
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/barrier.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/dinosaur.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/platform.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh :
         GLTFLoader::load("../assets/museum_final/platform_002.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock07.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock08.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock09.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock10.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock11.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock12.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }
    for (auto *mesh : GLTFLoader::load("../assets/museum_final/rock_moss_set_02_rock13.glb",
                                       MaterialType::Diffuse)) {
        scene.Add(mesh);
    }

    return scene;
}
