#include "Photon.hpp"
#include "BRDFUtils.hpp"
#include "Scene.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

Vector3f PhotonGrid::worldToCell(Vector3f pos) const {
    return Vector3f((int)((pos.x - bounds_min.x) / cell_size.x),
                    (int)((pos.y - bounds_min.y) / cell_size.y),
                    (int)((pos.z - bounds_min.z) / cell_size.z));
}

int PhotonGrid::cellToIndex(int cx, int cy, int cz) const {
    return cx + cy * resolution + cz * resolution * resolution;
}

int PhotonGrid::photonToIndex(Vector3f pos) const {
    Vector3f cell = worldToCell(pos);
    return cellToIndex((int)cell.x, (int)cell.y, (int)cell.z);
}

void PhotonGrid::build(std::vector<Photon> &photons, float radius) {
    if (photons.empty())
        return;

    bounds_min = photons[0].position;
    bounds_max = photons[0].position;

    for (auto &p : photons) {
        bounds_min.x = std::min(bounds_min.x, p.position.x);
        bounds_min.y = std::min(bounds_min.y, p.position.y);
        bounds_min.z = std::min(bounds_min.z, p.position.z);
        bounds_max.x = std::max(bounds_max.x, p.position.x);
        bounds_max.y = std::max(bounds_max.y, p.position.y);
        bounds_max.z = std::max(bounds_max.z, p.position.z);
    }

    bounds_min = bounds_min - Vector3f(0.01f, 0.01f, 0.01f);
    bounds_max = bounds_max + Vector3f(0.01f, 0.01f, 0.01f);

    Vector3f scene_extent = bounds_max - bounds_min;
    resolution = (int)(scene_extent.norm() / radius);
    resolution = std::max(10, std::min(200, resolution));

    cell_size = (bounds_max - bounds_min) / (float)resolution;

    for (auto &p : photons) {
        int idx = photonToIndex(p.position);
        cells[idx].push_back(p);
    }
}

void PhotonGrid::query(Vector3f pos, float radius, std::vector<const Photon *> &result) const {
    result.clear();

    int cx_min = (int)((pos.x - radius - bounds_min.x) / cell_size.x);
    int cy_min = (int)((pos.y - radius - bounds_min.y) / cell_size.y);
    int cz_min = (int)((pos.z - radius - bounds_min.z) / cell_size.z);
    int cx_max = (int)((pos.x + radius - bounds_min.x) / cell_size.x);
    int cy_max = (int)((pos.y + radius - bounds_min.y) / cell_size.y);
    int cz_max = (int)((pos.z + radius - bounds_min.z) / cell_size.z);

    cx_min = std::max(0, cx_min);
    cx_max = std::min(resolution - 1, cx_max);
    cy_min = std::max(0, cy_min);
    cy_max = std::min(resolution - 1, cy_max);
    cz_min = std::max(0, cz_min);
    cz_max = std::min(resolution - 1, cz_max);

    for (int cx = cx_min; cx <= cx_max; cx++)
        for (int cy = cy_min; cy <= cy_max; cy++)
            for (int cz = cz_min; cz <= cz_max; cz++) {
                auto it = cells.find(cellToIndex(cx, cy, cz));
                if (it != cells.end())
                    for (auto &p : it->second)
                        if ((p.position - pos).norm() <= radius)
                            result.push_back(&p);
            }
}
Vector3f PhotonMap::estimateIrradiance(Vector3f pos, Vector3f normal) const {
    thread_local std::vector<const Photon*> nearby;
    caustic_grid.query(pos, photon_radius, nearby);
    if (nearby.empty()) return {};

    float area = M_PI * photon_radius * photon_radius;
    Vector3f irradiance = {};
    for (auto& p : nearby) {
        Vector3f power = p->power;
        irradiance += power;
    }
    return irradiance / area;
}

void PhotonMap::trace(Photon p, int depth, std::vector<Photon> &t_caustic, const Scene &scene) {
    if (depth > maxDepth)
        return;

    auto hit = scene.intersect(Ray(p.position, p.direction));
    if (!hit.happened || hit.material->isEmissive())
        return;

    Material *mat = hit.material;
    Vector3f geoN = dotProduct(p.direction, hit.normal) < 0.0f ? hit.normal : -hit.normal;

    if (mat->isDelta()) {
        ShadingData sd;
        sd.Ng = geoN;
        sd.N = geoN;

        Vector3f wo = -p.direction;
        BSDFSample bsdf = mat->sample(wo, sd);

        p.is_caustic = true;
        p.power = p.power * bsdf.f;
        p.direction = bsdf.wi;
        p.position = hit.coords + (dotProduct(bsdf.wi, geoN) > 0.0f ? geoN : -geoN) * EPSILON;
        trace(p, depth + 1, t_caustic, scene);

    } else {
        // diffuse — store caustic photon if eligible
        if (depth > 0 && p.is_caustic) {
            t_caustic.push_back({hit.coords, p.direction, p.power, p.is_caustic});
        }

        float survival = std::min(1.0f, std::max({p.power.x, p.power.y, p.power.z}));
        if (get_random_float() < survival && depth < maxDepth) {
            auto *dm = static_cast<DiffuseMaterial *>(mat);

            ShadingData sd = hit.hasTangent
                                 ? dm->buildShadingData(hit.tcoords, geoN, normalize(hit.tangent),
                                                        hit.tangentHandedness)
                                 : dm->buildShadingData(hit.tcoords, geoN);

            Vector3f wo = -p.direction;
            BSDFSample bsdf = dm->sample(wo, sd);
            if (bsdf.pdf < 1e-6f)
                return;

            float cosTheta = std::max(0.0f, dotProduct(bsdf.wi, geoN));
            Vector3f weight = bsdf.f * cosTheta / bsdf.pdf;
            p.power = p.power * weight;

            // then RR based on new power
            float survival = std::min(1.0f, std::max({p.power.x, p.power.y, p.power.z}));
            if (get_random_float() > survival) return;
            p.power = p.power / survival;
            p.position = hit.coords + geoN * EPSILON;
            p.direction = bsdf.wi;
            trace(p, depth + 1, t_caustic, scene);
        }
    }
}

void PhotonMap::emit(int num_photons, const Scene &scene) {
    const int num_threads = omp_get_max_threads() - 1;
    std::vector<std::vector<Photon>> thread_caustic(num_threads);
    for (auto &v : thread_caustic)
        v.reserve(num_photons / num_threads / 10);

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 100)
    for (int i = 0; i < num_photons; i++) {
        int tid = omp_get_thread_num();

        Intersection lightPoint;
        float light_pdf;
        scene.sampleLight(lightPoint, light_pdf);

        float area = 1.0f / light_pdf;
        Vector3f dir = sampleCosineHemisphere(lightPoint.normal);
        Vector3f power = lightPoint.material->m_emission * area / num_photons;
        Photon p;
        p.position = lightPoint.coords + lightPoint.normal * EPSILON;
        p.direction = dir;
        p.power = power;
        p.is_caustic = false;

        trace(p, 0, thread_caustic[tid], scene);
    }

    size_t total = 0;
    for (auto &v : thread_caustic)
        total += v.size();
    caustic_map.reserve(total);
    for (auto &v : thread_caustic)
        caustic_map.insert(caustic_map.end(), v.begin(), v.end());
}

void PhotonMap::build() { caustic_grid.build(caustic_map, photon_radius); }