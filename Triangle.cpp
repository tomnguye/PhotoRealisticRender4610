#include "Triangle.hpp"
#include "OBJ_Loader.hpp"

MeshTriangle::MeshTriangle(const std::string &filename, Vector3f offset, Material *mt) {
    objl::Loader loader;
    loader.LoadFile(filename);
    area = 0;
    m = mt;
    assert(loader.LoadedMeshes.size() == 1);
    auto mesh = loader.LoadedMeshes[0];

    Vector3f minVert(std::numeric_limits<float>::infinity());
    Vector3f maxVert(-std::numeric_limits<float>::infinity());

    for (int i = 0; i < (int) mesh.Vertices.size(); i += 3) {
        std::array<Vector3f, 3> face;
        for (int j = 0; j < 3; j++) {
            auto vert = Vector3f(mesh.Vertices[i + j].Position.X + offset.x,
                                 mesh.Vertices[i + j].Position.Y + offset.y,
                                 mesh.Vertices[i + j].Position.Z + offset.z);
            face[j] = vert;
            minVert = Vector3f(std::min(minVert.x, vert.x), std::min(minVert.y, vert.y),
                               std::min(minVert.z, vert.z));
            maxVert = Vector3f(std::max(maxVert.x, vert.x), std::max(maxVert.y, vert.y),
                               std::max(maxVert.z, vert.z));
        }
        triangles.emplace_back(face[0], face[1], face[2], mt);
    }

    bounding_box = Bounds3(minVert, maxVert);
    buildBVH();
}

Intersection Triangle::finalize(const Ray &ray, float t, float u, float v) const {
    Intersection inter;
    inter.happened = true;
    inter.coords = ray.origin + t * ray.direction;
    inter.tnear = t;
    inter.obj = const_cast<Triangle *>(this);
    inter.material = m;

    float w0 = 1.f - u - v;
    inter.tcoords = t0 * w0 + t1 * u + t2 * v;

    if (hasSmoothNormals)
        inter.normal = normalize(n0 * w0 + n1 * u + n2 * v);
    else
        inter.normal = this->normal;

    if (hasTangents) {
        Vector3f interpTan = normalize(tan0 * w0 + tan1 * u + tan2 * v);
        float w = tangentW0 * w0 + tangentW1 * u + tangentW2 * v;
        inter.tangent = interpTan;
        inter.tangentHandedness = w >= 0.f ? 1.f : -1.f;
        inter.hasTangent = true;
    }

    return inter;
}

Intersection Triangle::getIntersection(Ray ray) {
    // Thin wrapper: cheap test, then finalize the hit once. Kept for any caller
    // that wants a full Intersection directly; the hot BVH loop calls hitTest()
    // and finalize() separately so it only finalizes the closest triangle.
    TriHit h = hitTest(ray);
    if (!h.happened)
        return Intersection();
    return finalize(ray, h.t, h.u, h.v);
}