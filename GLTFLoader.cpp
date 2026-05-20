#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "GLTFLoader.hpp"
#include "Material.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"

#include <cstdio>
#include <limits>
#include <numeric>

inline std::vector<float> tg_readFloats(const tinygltf::Model &m, int idx) {
    auto &acc = m.accessors[idx];
    auto &bv = m.bufferViews[acc.bufferView];
    auto &buf = m.buffers[bv.buffer];
    int nc = tinygltf::GetNumComponentsInType(acc.type);
    int cs = tinygltf::GetComponentSizeInBytes(acc.componentType);
    size_t stride = bv.byteStride ? bv.byteStride : nc * cs;
    const unsigned char *base = buf.data.data() + bv.byteOffset + acc.byteOffset;

    std::vector<float> out;
    out.reserve(acc.count * nc);
    for (size_t i = 0; i < acc.count; ++i) {
        const unsigned char *e = base + i * stride;
        for (int c = 0; c < nc; ++c) {
            float v = 0;
            if (acc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
                memcpy(&v, e + c * 4, 4);
            else if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
                uint16_t u;
                memcpy(&u, e + c * 2, 2);
                v = u / 65535.f;
            } else if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE)
                v = e[c] / 255.f;
            out.push_back(v);
        }
    }
    return out;
}

inline std::vector<uint32_t> tg_readIndices(const tinygltf::Model &m, int idx) {
    auto &acc = m.accessors[idx];
    auto &bv = m.bufferViews[acc.bufferView];
    const unsigned char *base = m.buffers[bv.buffer].data.data() + bv.byteOffset + acc.byteOffset;

    std::vector<uint32_t> out(acc.count);
    for (size_t i = 0; i < acc.count; ++i) {
        if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
            uint32_t v;
            memcpy(&v, base + i * 4, 4);
            out[i] = v;
        } else if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
            uint16_t v;
            memcpy(&v, base + i * 2, 2);
            out[i] = v;
        } else {
            out[i] = base[i];
        }
    }
    return out;
}

void GLTFLoader::loadMaterial(const tinygltf::Model &model, const tinygltf::Material &gm,
                              Material *out) {
    auto &pbr = gm.pbrMetallicRoughness;

    // Scalar/vector factors.
    auto &bc = pbr.baseColorFactor;
    out->baseColor = Vector3f(bc[0], bc[1], bc[2]);
    out->metallic = (float)pbr.metallicFactor;
    out->roughness = (float)pbr.roughnessFactor;
    out->m_emission = Vector3f(gm.emissiveFactor[0], gm.emissiveFactor[1], gm.emissiveFactor[2]);

    // Base colour texture
    int ti = pbr.baseColorTexture.index;
    if (ti >= 0 && model.textures[ti].source >= 0) {
        auto &img = model.images[model.textures[ti].source];
        out->baseColorTex.data = img.image;
        out->baseColorTex.width = img.width;
        out->baseColorTex.height = img.height;
    }

    // Metallic roughness texture
    int mri = pbr.metallicRoughnessTexture.index;
    if (mri >= 0 && model.textures[mri].source >= 0) {
        auto &img = model.images[model.textures[mri].source];
        out->metallicRoughnessTex.data = img.image;
        out->metallicRoughnessTex.width = img.width;
        out->metallicRoughnessTex.height = img.height;
    }

    // Normal map.
    int ni = gm.normalTexture.index;
    if (ni >= 0 && model.textures[ni].source >= 0) {
        auto &img = model.images[model.textures[ni].source];
        out->normalTex.data = img.image;
        out->normalTex.width = img.width;
        out->normalTex.height = img.height;
    }

    // Emissive texture.
    int ei = gm.emissiveTexture.index;
    if (ei >= 0 && model.textures[ei].source >= 0) {
        auto &img = model.images[model.textures[ei].source];
        out->emissiveTex.data = img.image;
        out->emissiveTex.width = img.width;
        out->emissiveTex.height = img.height;
    }
}

std::vector<MeshTriangle *> GLTFLoader::load(const std::string &filename, Material *overrideMat) {
    printf("[GLTFLoader] Loading: %s\n", filename.c_str());

    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;

    bool ok = (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".glb")
                  ? loader.LoadBinaryFromFile(&model, &err, &warn, filename)
                  : loader.LoadASCIIFromFile(&model, &err, &warn, filename);

    if (!ok) {
        fprintf(stderr, "[GLTFLoader] Error: %s\n", err.c_str());
        return {};
    }
    if (!warn.empty())
        fprintf(stderr, "[GLTFLoader] Warning: %s\n", warn.c_str());

    std::vector<MeshTriangle *> result;

    for (auto &mesh : model.meshes) {
        for (auto &prim : mesh.primitives) {
            if (prim.mode != TINYGLTF_MODE_TRIANGLES)
                continue;

            auto pos = tg_readFloats(model, prim.attributes.at("POSITION"));
            auto nrm = prim.attributes.count("NORMAL")
                           ? tg_readFloats(model, prim.attributes.at("NORMAL"))
                           : std::vector<float>{};
            auto uvs = prim.attributes.count("TEXCOORD_0")
                           ? tg_readFloats(model, prim.attributes.at("TEXCOORD_0"))
                           : std::vector<float>{};
            auto tan = prim.attributes.count("TANGENT")
                           ? tg_readFloats(model, prim.attributes.at("TANGENT"))
                           : std::vector<float>{};
            auto idx = prim.indices >= 0 ? tg_readIndices(model, prim.indices) : [&] {
                std::vector<uint32_t> v(pos.size() / 3);
                std::iota(v.begin(), v.end(), 0);
                return v;
            }();

            Material *primMat = overrideMat ? overrideMat : new Material();
            if (!overrideMat && prim.material >= 0)
                loadMaterial(model, model.materials[prim.material], primMat);

            printf("[GLTFLoader] Primitive — roughness=%.2f metallic=%.2f "
                   "hasColorTex=%d hasMRTex=%d hasNormalTex=%d\n",
                   primMat->roughness, primMat->metallic, !primMat->baseColorTex.empty(),
                   !primMat->metallicRoughnessTex.empty(), !primMat->normalTex.empty());

            std::vector<Triangle> triangles;
            triangles.reserve(idx.size() / 3);

            Vector3f minVert(std::numeric_limits<float>::infinity());
            Vector3f maxVert(-std::numeric_limits<float>::infinity());

            for (size_t i = 0; i + 2 < idx.size(); i += 3) {
                auto [i0, i1, i2] = std::tie(idx[i], idx[i + 1], idx[i + 2]);

                Vector3f p0(pos[i0 * 3], pos[i0 * 3 + 1], pos[i0 * 3 + 2]);
                Vector3f p1(pos[i1 * 3], pos[i1 * 3 + 1], pos[i1 * 3 + 2]);
                Vector3f p2(pos[i2 * 3], pos[i2 * 3 + 1], pos[i2 * 3 + 2]);

                for (auto &p : {p0, p1, p2}) {
                    minVert = Vector3f(std::min(minVert.x, p.x), std::min(minVert.y, p.y),
                                       std::min(minVert.z, p.z));
                    maxVert = Vector3f(std::max(maxVert.x, p.x), std::max(maxVert.y, p.y),
                                       std::max(maxVert.z, p.z));
                }

                Triangle tri(p0, p1, p2, primMat);

                if (!nrm.empty()) {
                    tri.n0 = normalize(Vector3f(nrm[i0 * 3], nrm[i0 * 3 + 1], nrm[i0 * 3 + 2]));
                    tri.n1 = normalize(Vector3f(nrm[i1 * 3], nrm[i1 * 3 + 1], nrm[i1 * 3 + 2]));
                    tri.n2 = normalize(Vector3f(nrm[i2 * 3], nrm[i2 * 3 + 1], nrm[i2 * 3 + 2]));
                    tri.hasSmoothNormals = true;
                    tri.normal = normalize(tri.n0 + tri.n1 + tri.n2);
                }

                if (!uvs.empty()) {
                    tri.t0 = Vector2f(uvs[i0 * 2], uvs[i0 * 2 + 1]);
                    tri.t1 = Vector2f(uvs[i1 * 2], uvs[i1 * 2 + 1]);
                    tri.t2 = Vector2f(uvs[i2 * 2], uvs[i2 * 2 + 1]);
                }

                if (!tan.empty()) {
                    tri.tan0 = Vector3f(tan[i0 * 4], tan[i0 * 4 + 1], tan[i0 * 4 + 2]);
                    tri.tan1 = Vector3f(tan[i1 * 4], tan[i1 * 4 + 1], tan[i1 * 4 + 2]);
                    tri.tan2 = Vector3f(tan[i2 * 4], tan[i2 * 4 + 1], tan[i2 * 4 + 2]);
                    tri.tangentW0 = tan[i0 * 4 + 3];
                    tri.tangentW1 = tan[i1 * 4 + 3];
                    tri.tangentW2 = tan[i2 * 4 + 3];
                    tri.hasTangents = true;
                }

                triangles.push_back(tri);
            }

            result.push_back(
                new MeshTriangle(std::move(triangles), Bounds3(minVert, maxVert), primMat));
        }
    }

    return result;
}