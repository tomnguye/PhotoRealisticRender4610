#pragma once
#ifndef PHOTON_H
#define PHOTON_H

#include "Vector.hpp"
#include <unordered_map>
#include <vector>

class Scene;

struct Photon {
    Vector3f position;
    Vector3f direction;
    Vector3f power;
    bool is_caustic; // did this photon pass through a specular surface
};

struct PhotonGrid {
    Vector3f bounds_max;
    Vector3f bounds_min;
    Vector3f cell_size;
    int resolution;
    std::unordered_map<int, std::vector<Photon>> cells;

    Vector3f worldToCell(Vector3f pos) const;
    int cellToIndex(int cx, int cy, int cz) const;
    int photonToIndex(Vector3f pos) const;
    void build(std::vector<Photon> &photons, float radius);
    void query(Vector3f pos, float radius, std::vector<const Photon *> &result) const;
};

class PhotonMap {
public:
    std::vector<Photon> caustic_map;
    PhotonGrid caustic_grid;
    float photon_radius = 0.15f;

    void emit(int num_photons, const Scene &scene);
    void build();
    Vector3f estimateIrradiance(Vector3f pos, Vector3f normal) const;

private:
    void trace(Photon p, int depth, std::vector<Photon> &t_caustic, const Scene &scene);
};

#endif