cmake_minimum_required(VERSION 3.10)

# set the project name
project(simple-c-implementation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# find boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  message("Found OpenMP")
endif()

set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")
set(CMAKE_PREFIX_PATH="/opt/homebrew/opt/llvm")  

# -DCMAKE_BUILD_TYPE=ASAN -DCMAKE_CXX_COMPILER=/usr/local/opt/llvm/bin/clang++


add_library(BeliefPropagation SHARED particle-belief-propagation.h particle-belief-propagation.c util.h util.c)
target_link_libraries(BeliefPropagation m OpenMP::OpenMP_CXX)

# add the executable
add_executable(test-particle-filter test-particle-filter.c)
target_link_libraries(test-particle-filter BeliefPropagation)

# add the executable
add_executable(particle-filter-consumer ParticleFilterConsumer.hpp ParticleFilterConsumer.cpp)
target_link_libraries(particle-filter-consumer BeliefPropagation Boost::boost)


if(UNIX AND NOT APPLE)
  target_link_libraries(particle-filter-consumer rt pthread)
  target_link_libraries(BeliefPropagation rt pthread)  
endif()  
