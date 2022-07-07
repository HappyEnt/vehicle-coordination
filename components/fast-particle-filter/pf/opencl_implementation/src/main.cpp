#ifdef __APPLE__
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#include "opencl.hpp"
#else
#include <CL/cl.hpp>
#endif

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

// #define LENGTH (1024)
// #define TOL (0.001)

// pick up device type from compiler command line or from the default type
#ifndef DEVICE
#define DEVICE CL_DEVICE_TYPE_DEFAULT
#endif

float gen_random() { return rand() / (float)RAND_MAX; }

std::string load_program(std::string input) {
  std::ifstream stream(input.c_str());
  if (!stream.is_open()) {
    std::cout << "Cannot open file: " << input << std::endl;
    exit(1);
  }
  return std::string(std::istreambuf_iterator<char>(stream),
                     (std::istreambuf_iterator<char>()));
}

int main() {
  // declare host containers
  std::vector<double2> particles(LENGTH);

  // // declare device containers
  // cl::Buffer d_a;
  // cl::Buffer d_b;
  // cl::Buffer d_c;

  cl::Buffer d_particles;

  // fill host containers with random numbers
  // std::generate(h_a.begin(), h_a.end(), gen_random);
  // std::generate(h_b.begin(), h_b.end(), gen_random);

  // create a context
  cl::Context context(DEVICE);

  // load in kernel source, creating a program object for the context
  cl::Program program(context, load_program("src/importance_sampling.cl"), true);

  // get the command queue
  cl::CommandQueue queue(context);

 
  // create the kernel functor
  // auto vadd =
  //   cl::compatibility::make_kernel<cl::Buffer, cl::Buffer, cl::Buffer, int>(program, "vadd");

  // copy data to device
  // d_a = cl::Buffer(context, begin(h_a), end(h_a), true);
  // d_b = cl::Buffer(context, begin(h_b), end(h_b), true);


  // allocate results container
  // d_c = cl::Buffer(context, CL_MEM_WRITE_ONLY, sizeof(float) * LENGTH);

  // run calculations
  // vadd(cl::EnqueueArgs(queue, cl::NDRange(LENGTH)), d_a, d_b, d_c, LENGTH);

  // queue.finish();

  // cl::copy(queue, d_c, begin(h_c), end(h_c));

  // test the results
}
