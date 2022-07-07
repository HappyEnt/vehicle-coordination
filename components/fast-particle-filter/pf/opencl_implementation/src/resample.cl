#pragma OPENCL EXTENSION cl_khr_fp64 : enable 

// TODO is there already a known good way to parallize this?
// if we want to do low variance sampling we cant really parallelize.

__kernel void resample(
                       __global double2 *resampled_particles,
                       const int amount,
                       const double3 *weighted_particles,
                       const unsigned int weight_amount)
{
  int i = get_global_id(0);

  if(i < amount)  {
    double U, r, c;
    size_t i;

    r = (((double) rand())/(RAND_MAX)) * (1.0/length);
    U = 0;
    c = weighted_particles[0].weight;

    i = 0;
    
    U = r + ((double) m)/length;
    while (U > c) {
      i++;
      c += weighted_particles[i].weight;
    }

    resampled_particles[m] = weighted_particles[i].particle;
  }
}
