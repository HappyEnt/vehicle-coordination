#ifdef __APPLE__
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#include "opencl.hpp"
#else
#include <CL/cl.hpp>
#endif

#ifndef DEVICE
#define DEVICE CL_DEVICE_TYPE_DEFAULT
#endif


#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>


typedef std::vector<cl_double2> ParticleList;

struct Particle {
  double pos_x;
  double pos_y;
};

struct Measurement {
  double sensor_distance;
  ParticleList particles;
};

class ParticleFilterOpenCL
{
public:
  ParticleFilterOpenCL();
  virtual ~ParticleFilterOpenCL();

  void correct(Measurement measurement);
  void predict();
  void set_particles();
  void get_particles();

private:
  std::vector<cl_double2> particles_local;

  // // declare device containers
  cl::Buffer d_particles_local;
  
  cl::Program program_vadd;
  cl::Program importance_sampling_program;
  cl::Program sum_program;
  cl::Program resample_program;

  cl::Context context;

  void resample();
  void low_variance_resampling(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length);
  
  std::vector<double> run_importance_sampling_program(std::vector<cl_double2> particles_other, double measurement);
  void run_low_variance_resampling_program();

  void normalize();
  void calculate_likelihood();
};
