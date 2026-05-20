#pragma once

#include "BVH.hpp"
#include "GLTFLoader.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include <array>
#include <cassert>
#include <cstring>

class Triangle : public Object {
public:
    Vector3f v0, v1, v2;       // vertices A, B, C counter-clockwise order
    Vector3f e1, e2;           // edges v1-v0, v2-v0
    Vector2f t0, t1, t2;       // per-vertex texture coords
    Vector3f tan0, tan1, tan2; // per-vertex tangents (xyz only)
    float tangentW0 = 1.f, tangentW1 = 1.f,
          tangentW2 = 1.f; // glTF tangent handedness (w component)
    bool hasTangents = false;
    Vector3f n0, n1, n2; // per-vertex smooth normals
    bool hasSmoothNormals = false;
    Vector3f normal;
    float area;
    Material *m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        area = crossProduct(e1, e2).norm() * 0.5f;
    }

    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index,
                              const Vector2f &uv, Vector3f &N, Vector2f &st) const override {
        N = normal;
    }
    Bounds3 getBounds() override;
    void Sample(Intersection &pos, float &pdf) override {
        float x = std::sqrt(get_random_float()), y = get_random_float();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = this->normal;
        pdf = 1.0f / area;
    }
    float getArea() override { return area; }
    bool hasEmit() override { return m->hasEmission(); }
};

class MeshTriangle : public Object {
public:
    MeshTriangle(const Vector3f *verts, const uint32_t *vertsIndex, const uint32_t &numTris,
                 const Vector2f *st, Material *mt = new Material()) {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
        m = mt;

        Vector3f min_vert =
            Vector3f{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < numTris; i++) {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(verts[vertsIndex[i * 3 + j]].x, verts[vertsIndex[i * 3 + j]].y,
                                     verts[vertsIndex[i * 3 + j]].z);
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x), std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x), std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }
            Triangle *tri = new Triangle(face_vertices[0], face_vertices[1], face_vertices[2], mt);
            tri->t0 = st[vertsIndex[i * 3]];
            tri->t1 = st[vertsIndex[i * 3 + 1]];
            tri->t2 = st[vertsIndex[i * 3 + 2]];

            triangles.push_back(*tri);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri : triangles) {
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = new BVHAccel(ptrs);
    }

    MeshTriangle(const std::string &filename, Vector3f offset, Material *mt = new Material()) {
        objl::Loader loader;
        loader.LoadFile(filename);
        area = 0;
        m = mt;
        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert =
            Vector3f{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X + offset.x,
                                     mesh.Vertices[i + j].Position.Y + offset.y,
                                     mesh.Vertices[i + j].Position.Z + offset.z);
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x), std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x), std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            triangles.emplace_back(face_vertices[0], face_vertices[1], face_vertices[2], mt);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri : triangles) {
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = new BVHAccel(ptrs);
    }

    // ── glTF constructor — loads material from glTF file ─────────────────────
    MeshTriangle(const std::string &filename) {
        area = 0;
        m = new Material();
        loadGLTF(filename, true);
    }

    // ── glTF constructor — uses provided material, ignores glTF material ──────
    MeshTriangle(const std::string &filename, Material *mt) {
        area = 0;
        m = mt;
        loadGLTF(filename, false);
    }

private:
    void loadGLTF(const std::string &filename, bool loadMaterial) {
        printf("Loading: %s\n", filename.c_str());

        tinygltf::Model model;
        tinygltf::TinyGLTF loader;
        std::string err, warn;
        bool ok = (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".glb")
                      ? loader.LoadBinaryFromFile(&model, &err, &warn, filename)
                      : loader.LoadASCIIFromFile(&model, &err, &warn, filename);
        if (!ok) {
            fprintf(stderr, "[glTF] %s\n", err.c_str());
            return;
        }

        Vector3f min_vert(std::numeric_limits<float>::infinity());
        Vector3f max_vert(-std::numeric_limits<float>::infinity());

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

                Material *primMat = new Material(*m);

                if (loadMaterial && prim.material >= 0) {
                    auto &gm = model.materials[prim.material];
                    auto &pbr = gm.pbrMetallicRoughness;

                    // PBR factors
                    auto &bc = pbr.baseColorFactor;
                    primMat->baseColor = Vector3f(bc[0], bc[1], bc[2]);
                    primMat->metallic = (float)pbr.metallicFactor;
                    primMat->roughness = (float)pbr.roughnessFactor;
                    primMat->m_emission =
                        Vector3f(gm.emissiveFactor[0], gm.emissiveFactor[1], gm.emissiveFactor[2]);

                    // base colour texture (sRGB — decoded at sample time in TextureUtils)
                    int ti = pbr.baseColorTexture.index;
                    if (ti >= 0 && model.textures[ti].source >= 0) {
                        auto &img = model.images[model.textures[ti].source];
                        primMat->baseColorTex.data = img.image;
                        primMat->baseColorTex.width = img.width;
                        primMat->baseColorTex.height = img.height;
                    }

                    // metallic-roughness texture (linear, G=roughness B=metallic)
                    int mri = pbr.metallicRoughnessTexture.index;
                    if (mri >= 0 && model.textures[mri].source >= 0) {
                        auto &img = model.images[model.textures[mri].source];
                        primMat->metallicRoughnessTex.data = img.image;
                        primMat->metallicRoughnessTex.width = img.width;
                        primMat->metallicRoughnessTex.height = img.height;
                    }

                    // normal map (linear, tangent space)
                    int ni = gm.normalTexture.index;
                    if (ni >= 0 && model.textures[ni].source >= 0) {
                        auto &img = model.images[model.textures[ni].source];
                        primMat->normalTex.data = img.image;
                        primMat->normalTex.width = img.width;
                        primMat->normalTex.height = img.height;
                    }

                    // emissive texture (sRGB — decoded at sample time in TextureUtils)
                    int ei = gm.emissiveTexture.index;
                    if (ei >= 0 && model.textures[ei].source >= 0) {
                        auto &img = model.images[model.textures[ei].source];
                        primMat->emissiveTex.data = img.image;
                        primMat->emissiveTex.width = img.width;
                        primMat->emissiveTex.height = img.height;
                    }
                }

                printf("mat: roughness=%.2f metallic=%.2f hasColorTex=%d hasMRTex=%d "
                       "hasNormalTex=%d\n",
                       primMat->roughness, primMat->metallic, !primMat->baseColorTex.empty(),
                       !primMat->metallicRoughnessTex.empty(), !primMat->normalTex.empty());

                for (size_t i = 0; i + 2 < idx.size(); i += 3) {
                    auto [i0, i1, i2] = std::tie(idx[i], idx[i + 1], idx[i + 2]);

                    Vector3f p0(pos[i0 * 3], pos[i0 * 3 + 1], pos[i0 * 3 + 2]);
                    Vector3f p1(pos[i1 * 3], pos[i1 * 3 + 1], pos[i1 * 3 + 2]);
                    Vector3f p2(pos[i2 * 3], pos[i2 * 3 + 1], pos[i2 * 3 + 2]);

                    for (auto &p : {p0, p1, p2}) {
                        min_vert = Vector3f(std::min(min_vert.x, p.x), std::min(min_vert.y, p.y),
                                            std::min(min_vert.z, p.z));
                        max_vert = Vector3f(std::max(max_vert.x, p.x), std::max(max_vert.y, p.y),
                                            std::max(max_vert.z, p.z));
                    }

                    Triangle tri(p0, p1, p2, primMat);

                    // per-vertex smooth normals
                    if (!nrm.empty()) {
                        tri.n0 = normalize(Vector3f(nrm[i0 * 3], nrm[i0 * 3 + 1], nrm[i0 * 3 + 2]));
                        tri.n1 = normalize(Vector3f(nrm[i1 * 3], nrm[i1 * 3 + 1], nrm[i1 * 3 + 2]));
                        tri.n2 = normalize(Vector3f(nrm[i2 * 3], nrm[i2 * 3 + 1], nrm[i2 * 3 + 2]));
                        tri.hasSmoothNormals = true;
                        tri.normal = normalize(tri.n0 + tri.n1 + tri.n2);
                    }

                    // per-vertex UVs
                    if (!uvs.empty()) {
                        tri.t0 = Vector2f(uvs[i0 * 2], uvs[i0 * 2 + 1]);
                        tri.t1 = Vector2f(uvs[i1 * 2], uvs[i1 * 2 + 1]);
                        tri.t2 = Vector2f(uvs[i2 * 2], uvs[i2 * 2 + 1]);
                    }

                    // per-vertex tangents — store xyz and handedness (w) separately
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
                    area += tri.area;
                }
            }
        }

        bounding_box = Bounds3(min_vert, max_vert);
        std::vector<Object *> ptrs;
        for (auto &tri : triangles)
            ptrs.push_back(&tri);
        bvh = new BVHAccel(ptrs);
    }

public:
    Bounds3 getBounds() { return bounding_box; }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index,
                              const Vector2f &uv, Vector3f &N, Vector2f &st) const {
        const Vector3f &v0 = vertices[vertexIndex[index * 3]];
        const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Intersection getIntersection(Ray ray) {
        Intersection intersec;
        if (bvh)
            intersec = bvh->Intersect(ray);
        return intersec;
    }

    void Sample(Intersection &pos, float &pdf) {
        bvh->Sample(pos, pdf);
        pos.obj = this;
        pos.material = this->m;
    }
    float getArea() { return area; }
    bool hasEmit() { return m->hasEmission(); }

    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel *bvh;
    float area;

    Material *m;
};

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray) {
    Vector3f P = crossProduct(ray.direction, e2);
    float PdotE1 = dotProduct(P, e1);
    if (fabs(PdotE1) < 1e-6)
        return Intersection();

    Vector3f T = ray.origin - v0;
    Vector3f Q = crossProduct(T, e1);

    float u = dotProduct(P, T) / PdotE1;
    if (u < 0)
        return Intersection();

    float v = dotProduct(Q, ray.direction) / PdotE1;
    if (v < 0 || u + v > 1)
        return Intersection();

    float t = dotProduct(Q, e2) / PdotE1;
    if (t <= 1e-3f)
        return Intersection();

    Intersection intersection;
    intersection.happened = true;
    intersection.coords = ray.origin + t * ray.direction;
    intersection.tnear = t;
    intersection.obj = this;
    intersection.material = m;

    // interpolate UVs from barycentric coords
    float w0 = 1.f - u - v;
    intersection.tcoords = t0 * w0 + t1 * u + t2 * v;

    // smooth shading: interpolate per-vertex normals if available, else use face normal
    if (hasSmoothNormals)
        intersection.normal = normalize(n0 * w0 + n1 * u + n2 * v);
    else
        intersection.normal = this->normal;

    // interpolate tangent and apply glTF handedness (w component)
    if (hasTangents) {
        Vector3f interpTan = normalize(tan0 * w0 + tan1 * u + tan2 * v);
        // interpolate handedness — in practice all three are the same sign, but lerp is safe
        float w = tangentW0 * w0 + tangentW1 * u + tangentW2 * v;
        intersection.tangent = interpTan;
        intersection.tangentHandedness = w >= 0.f ? 1.f : -1.f;
        intersection.hasTangent = true;
    }

    return intersection;
}