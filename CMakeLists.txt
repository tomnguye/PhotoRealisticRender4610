cmake_minimum_required(VERSION 3.10)
project(RayTracing)
set(CMAKE_CXX_STANDARD 17)

if(MSVC)
    set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
else()
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/Ox /Oi /Ot /Ob2 /GS- /Gy /arch:AVX2 /fp:fast /GL")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /OPT:REF /OPT:ICF")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -ffast-math")
endif()

if(NOT MSVC)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
        message(STATUS "ccache found. Using ccache.")
    endif()
endif()

set(SOURCES
    main.cpp
    GLTFLoader.cpp
    Scene.cpp
    BVH.cpp
    Renderer.cpp
    Triangle.cpp
    Material.cpp
    Photon.cpp
    Integrator.cpp
    Volume.cpp
)

add_executable(RayTracing ${SOURCES})

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(RayTracing PRIVATE OpenMP::OpenMP_CXX)
    message(STATUS "OpenMP found. Compiling with OpenMP support.")
else()
    message(STATUS "OpenMP not found. Compiling without OpenMP support.")
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported)
if(ipo_supported)
    set_property(TARGET RayTracing PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    message(STATUS "LTO enabled.")
endif()

if(MSVC)
    target_compile_options(RayTracing PRIVATE /MP)
endif()

target_precompile_headers(RayTracing PRIVATE global.hpp Vector.hpp)
target_include_directories(RayTracing PRIVATE thirdparty/tinygltf ${CMAKE_SOURCE_DIR})