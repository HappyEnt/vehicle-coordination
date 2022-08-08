#pragma OPENCL EXTENSION cl_khr_fp64 : enable

__kernel void calculate_likelihood(

   __global double* weights,
   __global double2* particles,
   __global double2* particles_other,
   const double measurement,
   const unsigned int amount,
   const unsigned int amount_other)
{
   double mean = 0;
   double std_dev = 0.1;
   int i = get_global_id(0);

   if(i < amount) {
    weights[i] = 1.0/amount;
    double weight_factor = 0.0;

    for (unsigned int j = 0; j < amount_other; ++j) {
      double dist = distance(particles[i], particles_other[j]);
      double likelihood = M_SQRT1_2*(M_2_SQRTPI/2.0) * 1.0/std_dev * exp(-0.5 * ((mean - dist)*(mean - dist))/(std_dev*std_dev));
      
       
      /* // dev_t and use precision prefactor */
      weight_factor += likelihood;
      weights[i] *= weight_factor;
   }

    // total_weight += weighted_particles[i].weight; calculate in own module
   }
}
