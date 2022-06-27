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

find_package(PythonLibs REQUIRED)

add_library(SimpleConsumer simple-consumer.c)
target_link_libraries(SimpleConsumer Boost::boost ${PYTHON_LIBRARIES})
target_include_directories(SimpleConsumer PUBLIC ${PYTHON_INCLUDE_DIRS})



