#pragma once
#include "tiny_gltf.h"
#include <string>
#include <vector>

class MeshTriangle;
class Material;
class DiffuseMaterial;

enum class MaterialType { Diffuse, Glass, Mirror, Emit };

class GLTFLoader {
  public:
    /**
     * @brief Loads a gltf/glb file and returns one MeshTriangle per primitive.
     *
     * @param filename Path to the .gltf or .glb file.
     * @param overrideMaterial If non-null all primitives use this material and glTF material data
     * is ignored.
     * @return Vector of heap-allocated MeshTriangle objects. Caller owns them.
     */
    static std::vector<MeshTriangle *> load(const std::string &filename, MaterialType materialType,
                                            Material *overrideMaterial = nullptr);

  private:
    /**
     * @brief Reads PBR textures and factors from a gltf material into a DiffuseMaterial.
     */
    static void loadMaterial(const tinygltf::Model &model, const tinygltf::Material &gm,
                             DiffuseMaterial *out);
};