cmake_minimum_required(VERSION 3.15)

project(opencl_cmake VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

FIND_PACKAGE(PythonInterp)

ENABLE_TESTING()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system python3 REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)


message(STATUS "PYTHON_LIBRARIES = ${Python_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${Python_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${Python_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

find_package(OpenCL REQUIRED)

add_library(particle_filter_opencl SHARED src/ParticleFilterOpenCL.cpp)
add_library(particle_filter SHARED src/ParticleFilterPythonInterface.cpp)

# add_executable(test_opencl_filter src/main.cpp)
# target_compile_features(particle_filter_opencl PRIVATE cxx_auto_type)


target_include_directories(particle_filter PUBLIC ${Python_INCLUDE_DIRS})

target_link_libraries(particle_filter_opencl OpenCL::OpenCL)
target_link_libraries(particle_filter Python3::Module Boost::python3 particle_filter_opencl)

set_target_properties(particle_filter PROPERTIES SUFFIX .so)
set_target_properties(particle_filter PROPERTIES PREFIX "")

# PYTHON_ADD_MODULE(particlefilter src/ParticleFilterOpenCL.cpp)
# FILE(COPY particlefilter.py DESTINATION .)
