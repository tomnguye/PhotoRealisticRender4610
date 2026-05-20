cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -ffast-math")
set(CMAKE_BUILD_PARALLEL_LEVEL 8)

# Use ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    message(STATUS "ccache found. Using ccache.")
endif()

set(SOURCES
    main.cpp
    GLTFLoader.hpp
    GLTFLoader.cpp
    Scene.cpp
    BVH.cpp
    Renderer.cpp
    Object.hpp
    Vector.hpp
    Sphere.hpp
    global.hpp
    Triangle.hpp
    Scene.hpp
    BVH.hpp
    Bounds3.hpp
    Ray.hpp
    Material.hpp
    Intersection.hpp
    Renderer.hpp
    Texture.hpp
    TextureUtils.hpp
    BRDFUtils.hpp
    Photon.cpp
    Photon.hpp
    Integrator.hpp
    Integrator.cpp
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    add_executable(RayTracing ${SOURCES})
    target_link_libraries(RayTracing PUBLIC OpenMP::OpenMP_CXX)
    message(STATUS "OpenMP found. Compiling with OpenMP support.")
else()
    add_executable(RayTracing ${SOURCES})
    message(STATUS "OpenMP not found. Compiling without OpenMP support.")
endif()

target_precompile_headers(RayTracing PRIVATE global.hpp Vector.hpp)
target_include_directories(RayTracing PRIVATE thirdparty/tinygltf ${CMAKE_SOURCE_DIR})