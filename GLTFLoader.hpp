#pragma once
#include "tiny_gltf.h"
#include <cstring>
#include <string>
#include <vector>

// Forward declarations — avoids pulling Triangle.hpp into every TU that includes this.
class MeshTriangle;
class Material;

class GLTFLoader {
public:
    /**
     * @brief Loads a glTF/glb file and returns one MeshTriangle per primitive.
     *
     * @param filename   Path to the .gltf or .glb file.
     * @param overrideMat  If non-null, all primitives use this material and
     *                     glTF material data is ignored.
     * @return Vector of heap-allocated MeshTriangle objects. Caller owns them.
     */
    static std::vector<MeshTriangle *> load(const std::string &filename,
                                            Material *overrideMat = nullptr);

private:
    /**
     * @brief Reads PBR textures and factors from a glTF material into a Material.
     */
    static void loadMaterial(const tinygltf::Model &model, const tinygltf::Material &gm,
                             Material *out);
};