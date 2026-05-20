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

    for (int i = 0; i < (int)mesh.Vertices.size(); i += 3) {
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

Intersection Triangle::getIntersection(Ray ray) {
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

    Intersection inter;
    inter.happened = true;
    inter.coords = ray.origin + t * ray.direction;
    inter.tnear = t;
    inter.obj = this;
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