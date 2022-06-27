cmake_minimum_required(VERSION 3.10)

# set the project name
project(simple-c-implementation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# find boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)


add_library(ParticleFilter SHARED particle-filter.h particle-filter.c)
target_link_libraries(ParticleFilter m)

# add the executable
add_executable(test_particle_filter test-particle-filter.c)
target_link_libraries(test_particle_filter ParticleFilter)

# add the executable
add_executable(particle-filter-consumer ParticleFilterConsumer.hpp ParticleFilterConsumer.cpp)
target_link_libraries(particle-filter-consumer ParticleFilter Boost::boost)
