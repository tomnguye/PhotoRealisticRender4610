cmake_minimum_required(VERSION 3.10)
project(RayTracing)

## Enable C++14
set(CMAKE_CXX_STANDARD 17)

# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)

    add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
    Scene.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
    Renderer.cpp Renderer.hpp Photon.hpp Photon.cpp)

    target_link_libraries(RayTracing PUBLIC OpenMP::OpenMP_CXX)

    message(STATUS "OpenMP found. Compiling with OpenMP support.")

else()
    add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
    Scene.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
    Renderer.cpp Renderer.hpp Photon.hpp Photon.cpp)

    message(STATUS "OpenMP not found. Compiling without OpenMP support.")

endif()

