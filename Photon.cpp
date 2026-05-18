#include "Photon.hpp"

Vector3f PhotonGrid::worldToCell(Vector3f pos) const {
    return Vector3f(
        (int)((pos.x - bounds_min.x) / cell_size.x),
        (int)((pos.y - bounds_min.y) / cell_size.y),
        (int)((pos.z - bounds_min.z) / cell_size.z)
    );
}

int PhotonGrid::cellToIndex(int cx, int cy, int cz) const {
    // flatten 3D cell coords to single int key
    return cx + cy * resolution + cz * resolution * resolution;
}

int PhotonGrid::photonToIndex(Vector3f pos) const {
    Vector3f cell = worldToCell(pos);
    return cellToIndex((int)cell.x, (int)cell.y, (int)cell.z);
}

void PhotonGrid::build(std::vector<Photon>& photons, float radius) {
    if (photons.empty()) return;

    // find bounds
    bounds_min = photons[0].position;
    bounds_max = photons[0].position;
    
    for (auto& p : photons) {
        bounds_min.x = std::min(bounds_min.x, p.position.x);
        bounds_min.y = std::min(bounds_min.y, p.position.y);
        bounds_min.z = std::min(bounds_min.z, p.position.z);
        bounds_max.x = std::max(bounds_max.x, p.position.x);
        bounds_max.y = std::max(bounds_max.y, p.position.y);
        bounds_max.z = std::max(bounds_max.z, p.position.z);
    }
    
    // add small padding
    bounds_min = bounds_min - Vector3f(0.01f, 0.01f, 0.01f);
    bounds_max = bounds_max + Vector3f(0.01f, 0.01f, 0.01f);


    Vector3f scene_extent = bounds_max - bounds_min;
    resolution = (int)(scene_extent.norm() / radius);
    resolution = std::max(10, std::min(200, resolution));
    
    // compute cell size
    cell_size = (bounds_max - bounds_min) / (float)resolution;
    
    // insert photons into cells
    for (auto& p : photons) {
        int idx = photonToIndex(p.position);
        cells[idx].push_back(p);
    }
}

std::vector<Photon> PhotonGrid::query(Vector3f pos, float radius) const {
    std::vector<Photon> result;
    
    // find cell range to search
    int cx_min = (int)((pos.x - radius - bounds_min.x) / cell_size.x);
    int cy_min = (int)((pos.y - radius - bounds_min.y) / cell_size.y);
    int cz_min = (int)((pos.z - radius - bounds_min.z) / cell_size.z);
    
    int cx_max = (int)((pos.x + radius - bounds_min.x) / cell_size.x);
    int cy_max = (int)((pos.y + radius - bounds_min.y) / cell_size.y);
    int cz_max = (int)((pos.z + radius - bounds_min.z) / cell_size.z);
    
    // clamp to grid bounds
    cx_min = std::max(0, cx_min); cx_max = std::min(resolution-1, cx_max);
    cy_min = std::max(0, cy_min); cy_max = std::min(resolution-1, cy_max);
    cz_min = std::max(0, cz_min); cz_max = std::min(resolution-1, cz_max);
    
    // collect all photons in range
    for (int cx = cx_min; cx <= cx_max; cx++)
    for (int cy = cy_min; cy <= cy_max; cy++)
    for (int cz = cz_min; cz <= cz_max; cz++) {
        int idx = cellToIndex(cx, cy, cz);
        auto it = cells.find(idx);
        if (it != cells.end()) {
            for (auto& p : it->second) {
                float dist = (p.position - pos).norm();
                if (dist <= radius)
                    result.push_back(p);
            }
        }
    }
    
    return result;
}


Vector3f PhotonMap::estimateIrradiance(Vector3f pos, Vector3f normal, float initial_radius) const
{
    const int targetPhotons = 8;
    
    // initial query
    auto nearby = caustic_grid.query(pos, initial_radius);
    float radius = initial_radius;

    // if not enough, predict radius needed based on photon density
    if ((int)nearby.size() < targetPhotons && !nearby.empty()) {
        // density = photons / area
        // to get targetPhotons we need area = targetPhotons / density
        // radius = sqrt(targetPhotons / density / pi)
        float density = nearby.size() / (M_PI * initial_radius * initial_radius);
        float predicted_radius = std::sqrt(targetPhotons / (density * M_PI));
        predicted_radius = std::min(predicted_radius, initial_radius * 8.0f); // cap growth
        nearby = caustic_grid.query(pos, predicted_radius);
        radius = predicted_radius;
    } else if (nearby.empty()) {
        return {};
    }

    if (nearby.empty()) return {};

    float area = M_PI * radius * radius;
    if (area < EPSILON) return {};

    Vector3f irradiance = {};
    for (auto& p : nearby) {
        float cos_theta = dotProduct(-p.direction, normal);
        if (cos_theta > 0)
            irradiance += p.power * cos_theta;
    }
    return irradiance / area;
}
// ─────────────────────────────────────────────
//  Photon tracing
// ─────────────────────────────────────────────

void PhotonMap::trace(Photon p, int depth, std::vector<Photon>& t_caustic, const Scene& scene)
{
    if (depth > scene.maxDepth) return;

    auto hit = scene.intersect(Ray(p.position, p.direction));
    if (!hit.happened || hit.material->hasEmission()) return;

    if (hit.material->m_type == MIRROR) {
        p.is_caustic = true;
        p.direction  = scene.reflect(p.direction, hit.normal).normalized();
        p.position   = hit.coords + hit.normal * EPSILON;
        trace(p, depth + 1, t_caustic, scene);
    }
    else if (hit.material->m_type == GLASS) {
        float     kr        = scene.fresnel(p.direction, hit.normal, hit.material->ior);
        Vector3f  refracted = scene.refract(p.direction, hit.normal, hit.material->ior);
        bool      tir       = (refracted.norm() < EPSILON);

        if (get_random_float() < kr || tir) {
            p.power    = p.power * kr;
            p.direction = scene.reflect(p.direction, hit.normal).normalized();
            p.position  = hit.coords + hit.normal * EPSILON;
        } else {
            p.is_caustic = true;
            p.power     = p.power * (1.0f - kr);
            p.direction  = refracted.normalized();
            p.position   = hit.coords - hit.normal * EPSILON;
        }
        trace(p, depth + 1, t_caustic, scene);
    }
    else { // diffuse
        if (depth > 0 && p.is_caustic) {
            t_caustic.push_back({ hit.coords, p.direction, p.power, p.is_caustic });
        }

        float survival = std::min(1.0f, std::max({p.power.x, p.power.y, p.power.z}));
        if (get_random_float() < survival && depth < scene.maxDepth) {
            p.power     = p.power * hit.material->getColor() / survival;
            p.position  = hit.coords + hit.normal * EPSILON;
            p.direction = hit.material->sample(p.direction, hit.normal);
            trace(p, depth + 1, t_caustic, scene);
        }
    }
}

void PhotonMap::emit(int num_photons, const Scene& scene)
{
    const int num_threads = 8;
    std::vector<std::vector<Photon>> thread_caustic(num_threads);
    for (auto& v : thread_caustic)
        v.reserve(num_photons / num_threads / 10);

    #pragma omp parallel for num_threads(num_threads) schedule(dynamic, 100)
    for (int i = 0; i < num_photons; i++) {
        int tid = omp_get_thread_num();

        Intersection lightPoint;
        float light_pdf;
        scene.sampleLight(lightPoint, light_pdf);

        float    area  = 1.0f / light_pdf;
        Vector3f dir   = lightPoint.material->sample(0, lightPoint.normal);
        Vector3f power = lightPoint.material->getEmission() * area / num_photons;

        Photon p;
        p.position   = lightPoint.coords + lightPoint.normal * EPSILON;
        p.direction  = dir;
        p.power      = power;
        p.is_caustic = false;

        trace(p, 0, thread_caustic[tid], scene);
    }

    size_t total = 0;
    for (auto& v : thread_caustic) total += v.size();
    caustic_map.reserve(total);
    for (auto& v : thread_caustic)
        caustic_map.insert(caustic_map.end(), v.begin(), v.end());
}