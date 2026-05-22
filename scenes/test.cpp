#include "GLTFLoader.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include <Camera.hpp>

static Scene buildScene() {
    auto camera =
        Camera::fromBlender(Vector3f(4.6301f, 8.5258f, 0.1142f),   // eye    (Blender coords)
                            Vector3f(-0.2530f, -0.2008f, 0.1142f), // target (Blender coords)
                            Vector3f(-0.0000f, 0.0000f, 1.0000f),  // up     (Blender coords)
                            53.1301f,                              // fov_h (degrees)
                            31.4173f,                              // fov_v (degrees)
                            9.7026f                                // focusDistance (from 'furnace')
        );

    Scene scene;
    scene.camera = camera;

    scene.envMap.load("../assets/hdri/lonely_road_afternoon_puresky_8k.hdr");

    for (auto *mesh :
         GLTFLoader::load("../assets/blender_models/furnace.glb", MaterialType::Diffuse)) {
        scene.Add(mesh);
        mesh->m->baseColor = Vector3f(0.8f, 0.8f, 1.0f);
        std::cout << mesh->m->baseColor << "\n";
    }

    return scene;
}
