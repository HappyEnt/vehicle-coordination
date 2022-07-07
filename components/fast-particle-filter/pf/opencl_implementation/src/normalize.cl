#pragma OPENCL EXTENSION cl_khr_fp64 : enable 

__kernel void calculate_likelihood(
   __global double* weights,
   __global double* normalized_weights,   
   __global double total_weight,
   const unsigned int weight_amount)
{
   int i = get_global_id(0);

   if(i < weight_amount)  {
        normalized_weights[i] = weights[i]/total_weight;
   }
}
