#include "floating-point-particle-filter.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <memory.h>

#define SIGN(x) ((x > 0) - (x < 0))

// proto
double distance1(double x, double y);
double distance2(struct particle p1, struct particle p2);

static double from_normal(double x) {
  double sqpi = sqrt(2 * M_PI);  
  return (1.0 / sqpi) * exp(-(x*x)/2);
}

struct normal_distribution *generate_normal_distribution(
                                  double mean,
                                  double std_dev,
                                  bool cache_histogram) { // the standard deviation has also to be a integer
  static unsigned int buckets = 1000; // TODO calculate

  struct normal_distribution *result_distribution =
    malloc(sizeof(struct normal_distribution));
  
  if(cache_histogram) {
    // because of the symmetry of the normal distribution, we only generate half of the distribution
    double bucket_size = 4.0/buckets;
    
    result_distribution->cached_distribution = malloc(sizeof(double) * buckets);
    result_distribution->bucket_size = bucket_size;    
  } else {
    result_distribution->cached_distribution = NULL; 
  }


  result_distribution->std_dev = std_dev;
  result_distribution->mean = mean;

  if(cache_histogram) {
    double *cached_distribution = result_distribution->cached_distribution;
  
    for (size_t i = 0; i < buckets; ++i) {
      double x = i * result_distribution->bucket_size;
      cached_distribution[i] = from_normal(x);
    }
  }  

  return result_distribution;
}

void destroy_normal_distribution(struct normal_distribution *distribution) {
  free(distribution->cached_distribution);
  free(distribution);
}

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      double x) {

  if(distribution->cached_distribution == NULL) {
    return from_normal(distance1(x,distribution->mean)/distribution->std_dev)/distribution->std_dev;
  }
  
  if (x > 4 * distribution->std_dev) {
    return 0.0;
  }

  size_t tx = (distance1(distribution->mean, x) / distribution->std_dev) / distribution->bucket_size;
  return distribution->cached_distribution[tx] / distribution->std_dev; // TODO maybe interpolate
}

double distance1(double x, double y) {
  if (x > y) {
    return x - y;
  }
  return y - x;
}

double distance2(struct particle p1, struct particle p2) {
  double dx = 0;
  double dy = 0;

  dx = distance1(p1.x_pos, p2.x_pos);
  dy = distance1(p1.y_pos, p2.y_pos);

  return sqrt(dx * dx + dy * dy);
}

void low_variance_resampling(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length) {
  double U, r, c;
  size_t i;
  
  r = rand() * 1.0/length;
  U = 0;
  c = weighted_particles[0].weight;
    i = 0;
  for (size_t m = 0; m < length; ++m) {
    U = r + (double) (m - 1)/length;
    while (U > c) {
      i++;
      c = c + weighted_particles[i].weight;
    }
    
    resampled_particles[m] = weighted_particles[i].particle;
  }
}

void resample(struct weighted_particle *weighted_particles, struct particle *resampled_particles, size_t length) {
  low_variance_resampling(weighted_particles, resampled_particles, length);
}

void calculate_likelihood(double measurement, struct particle *particles,
                          struct particle *particles_other, size_t amount,
                          size_t amount_other,
                          struct weighted_particle *weighted_particles) {

  struct normal_distribution *norm;
  norm = generate_normal_distribution(0, 1000, false);

  for (size_t i = 0; i < amount; ++i) {
    weighted_particles[i].particle = particles[i];
    for (size_t j = 0; j < amount_other; ++j) {
      double likelihood = value_from_normal_distribution(
                                                         norm,
                                                         distance1(
                                                                   measurement,
                                                                   distance2(
                                                                             particles[i],
                                                                             particles[j]))); // if more precision is needed raise type to
      // uint64_t and use precision prefactor
      weighted_particles[i].weight += likelihood;
    }
  }
}

// _____Implementation of public particle filter interface_____

// TODO pass sensor parameters through structure
void create_particle_filter_instance(struct particle_filter_instance **pf_inst) {
  struct particle_filter_instance *ret_inst = malloc(sizeof(struct particle_filter_instance));

  ret_inst->local_particles = NULL;
  ret_inst->uwb_error_likelihood = generate_normal_distribution(0, 0.1, 1);

  *pf_inst = ret_inst;
}

void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst) {
  /* if(pf_inst->local_particles != NULL) */
  /*   free(pf_inst->local_particles); */

  free(pf_inst);
}

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length) {
  struct particle *pt_cpy = malloc(sizeof(struct particle) * length);

  memcpy(pt_cpy, particles, length);
  
  pf_inst->local_particles = pt_cpy;
  pf_inst->local_particles_length = length;
}


// on success returns length of particles, negative numbers indicate error
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles) {
  if(pf_inst->local_particles == NULL) {
    return -1;
  }

  *particles = pf_inst->local_particles;
  
  return pf_inst->local_particles_length;
}

void correct(struct particle_filter_instance *pf_inst, struct measurement *m) {
  struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * pf_inst->local_particles_length);

  calculate_likelihood(m->measured_distance, pf_inst->local_particles, m->foreign_particles, pf_inst->local_particles_length, m->foreign_particles_length, wp);

  resample(wp, pf_inst->local_particles, pf_inst->local_particles_length);

  free(wp);
}

void predict(struct particle_filter_instance *pf_inst, int action) {
  // TODO implement me
}
