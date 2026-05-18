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

