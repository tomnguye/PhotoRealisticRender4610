//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <sstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "Material.hpp"
#ifdef _OPENMP
    #include <omp.h>
#endif


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 1e-2;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    std::cout << "SPP: " << scene.spp << "\n";

    float progress = 0.0f;

#pragma omp parallel for num_threads(8) // use multi-threading for speedup if openmp is available
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {

            int m = i + j * scene.width;  // pixel index
            if(scene.spp==1){
                // TODO: task 1.1 pixel projection
                float x = (1.0f - 2.0f * (i + 0.5f) / scene.width) * scale * imageAspectRatio;
                float y = (1.0f - 2.0f * (j + 0.5f) / scene.height) * scale;
                Ray ray = {eye_pos, normalize(Vector3f(x, y, 1))};
                framebuffer[m] = Vector3f::Min(scene.castRay(ray , 0), 1);
            }else {
                // TODO: task 2 multi-sampling (anti-aliasing)
                // framebuffer[m] = Vector3f();
                // for (int k = 0; k < scene.spp; k ++) {
                //     float x = (1.0f - 2.0f * (i + get_random_float()) / scene.width) * scale * imageAspectRatio;
                //     float y = (1.0f - 2.0f * (j + get_random_float()) / scene.height) * scale;
                //     Ray ray = {eye_pos, normalize(Vector3f(x, y, 1))};
                //     // framebuffer[m] += Vector3f::Min(scene.castRay(ray , 0), 1);
                //     framebuffer[m] += scene.castRay(ray , 0);
                // }
                // framebuffer[m] = framebuffer[m]/ scene.spp;
                int min_samples = sqrt(scene.spp);
                int samples = 0;
                Vector3f color = {0, 0, 0};
                Vector3f prev_avg = {0, 0, 0};

                while (samples < scene.spp) {
                    float x = (1.0f - 2.0f * (i + get_random_float()) / scene.width) * scale * imageAspectRatio;
                    float y = (1.0f - 2.0f * (j + get_random_float()) / scene.height) * scale;
                    Ray ray = {eye_pos, normalize(Vector3f(x, y, 1))};
                    
                    color += scene.castRay(ray, 0);
                    samples++;
                    
                    if (samples >= min_samples) {
                        Vector3f current_avg = color / samples;
                        float change = (current_avg - prev_avg).norm();
                        if (change < 0.001f) break;
                        prev_avg = current_avg;
                    }
                }

                framebuffer[m] = color / samples;
            }
        }
        progress += 1.0f / (float)scene.height;
        UpdateProgress(progress);
    }
    UpdateProgress(1.f);

    // // Debug
    // for (auto& photon : scene.global_map) {
    //     // transform to camera space relative to eye
    //     Vector3f dir = (photon.position - eye_pos).normalized();
        
    //     // only project photons in front of camera
    //     if (dir.z <= 0) continue;
        
    //     // invert the ray construction
    //     // x = (1 - 2*i/width) * scale * aspect  →  i = (1 - x/(scale*aspect)) * width/2
    //     // y = (1 - 2*j/height) * scale           →  j = (1 - y/scale) * height/2
    //     float px = dir.x / dir.z;
    //     float py = dir.y / dir.z;
        
    //     int img_x = (int)((1.0f - px / (scale * imageAspectRatio)) * scene.width / 2.0f);
    //     int img_y = (int)((1.0f - py / scale) * scene.height / 2.0f);
        
    //     if (img_x >= 0 && img_x < scene.width && img_y >= 0 && img_y < scene.height) {
    //         framebuffer[img_y * scene.width + img_x] = Vector3f(0, 1, 0);
    //     }
    // }
    
    // for (auto& photon : scene.caustic_map) {
    //     Vector3f dir = (photon.position - eye_pos).normalized();
    //     if (dir.z <= 0) continue;
        
    //     float px = dir.x / dir.z;
    //     float py = dir.y / dir.z;
        
    //     int img_x = (int)((1.0f - px / (scale * imageAspectRatio)) * scene.width / 2.0f);
    //     int img_y = (int)((1.0f - py / scale) * scene.height / 2.0f);
        
    //     if (img_x >= 0 && img_x < scene.width && img_y >= 0 && img_y < scene.height) {
    //         framebuffer[img_y * scene.width + img_x] = Vector3f(1, 0, 1);
    //     }
    // }// Debug

    // save framebuffer to file
    std::stringstream ss;
    ss << "binary_task" << TASK_N<<".ppm";
    std::string str = ss.str();
    const char* file_name = str.c_str();
    FILE* fp = fopen(file_name, "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
