#pragma OPENCL EXTENSION cl_khr_fp64 : enable 

__kernel void calculate_likelihood(

   __global double* weights,
   __global double total_weight,
   
   /* const  double * */
   
   const double2* particles,
   const double2* particles_other,   
   const unsigned int amount,
   const unsigned int amount_other)
{
   int i = get_global_id(0);

   if(i < amount) {
    weighted_particles[i].particle = particles[i];
    weighted_particles[i].weight = 1.0/amount;
    double weight_factor = 0.0;

    for (size_t j = 0; j < amount_other; ++j) {
      double dist = distance(partices[i], particles_other[j]);
      double likelihood = M_SQRT1_2*(M_2_SQRTPI/2.0) * 1.0/std_dev * exp(-0.5 * ((mean - dist)*(mean - dist))/(std_dev*std_double));
      
       
      // dev_t and use precision prefactor
      weight_factor += likelihood;
      weighted_particles[i].weight *= weight_factor;
   }

    total_weight += weighted_particles[i].weight;
   }
}
