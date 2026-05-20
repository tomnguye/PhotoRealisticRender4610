#pragma once
#include "tiny_gltf.h"
#include <cstring>
#include <vector>

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
        } else
            out[i] = base[i];
    }
    return out;
}