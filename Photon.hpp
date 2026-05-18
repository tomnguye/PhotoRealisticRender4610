#pragma once
#ifndef PHOTON_H
#define PHOTON_H

#include "Scene.hpp"
#include "Vector.hpp"
#include <vector>
#include <unordered_map>

struct Photon {
    Vector3f position;
    Vector3f direction;
    Vector3f power;
    bool is_caustic; // did this photon pass through a specular surface
};

Photon findNearestPhoton(Vector3f hitPoint);

struct PhotonGrid {
    Vector3f bounds_max;
    Vector3f bounds_min;
    Vector3f cell_size;
    int resolution; 
    std::unordered_map<int, std::vector<Photon>> cells;

    Vector3f worldToCell(Vector3f pos) const;
    int cellToIndex(int cx, int cy, int cz) const;
    int photonToIndex(Vector3f pos) const;
    void build(std::vector<Photon>& photons, float radius);
    std::vector<Photon> query(Vector3f pos, float radius) const;
};

class PhotonMap {
public:
    std::vector<Photon> caustic_map;
    PhotonGrid          caustic_grid;
    float               photon_radius;

    void emit(int num_photons, const Scene& scene);
    void build();
    Vector3f estimateIrradiance(Vector3f pos, Vector3f normal, float initial_radius) const;

private:
    void trace(Photon p, int depth, std::vector<Photon>& t_caustic, const Scene& scene);
};

#endif