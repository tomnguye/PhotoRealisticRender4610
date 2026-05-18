#pragma once

#include <vector>

// ─── Texture ──────────────────────────────────────────────────────────────────
//
// Plain data owner. Holds decoded pixel bytes from a glTF image.
// Channels is always 4 (RGBA) for glTF images loaded via tinygltf.
// Nothing else lives here — sampling logic is in TextureUtils.hpp.

struct Texture {
    std::vector<unsigned char> data;
    int width = 0;
    int height = 0;
    int channels = 4; // glTF images are always RGBA via tinygltf

    bool empty() const { return data.empty(); }
};