#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <iostream>
#include <cstdlib> 
#include <ctime>
#include <chrono>

#undef M_PI
#define M_PI 3.141592653589793f

extern const float  EPSILON;
extern float TASK_N;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline  bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ?
                  -0.5 * (b + sqrt(discr)) :
                  -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) std::swap(x0, x1);
    return true;
}

inline float get_random_float() {
    // PCG random number generator
    static thread_local uint64_t state = 
        std::chrono::high_resolution_clock::now().time_since_epoch().count();
    
    // advance state
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    
    // generate output
    uint32_t xorshifted = ((state >> 18u) ^ state) >> 27u;
    uint32_t rot = state >> 59u;
    uint32_t result = (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    
    return (float)result / (float)UINT32_MAX;
}

inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};
