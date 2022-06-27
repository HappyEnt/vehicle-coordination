#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#define PARTICLES 100
#define MEASUREMENT_STDEV 0.1

struct particle {
  uint32_t x_pos;
  uint32_t y_pos;
};

struct normal_distribution {
  uint32_t mean;
  uint32_t std_dev;

  size_t buckets;
  uint32_t bucket_size;

  double *cached_distribution;
};

/**
 * @brief      generates n-buckets approximating normal distribution
 *
 * @details    evaluates and caches normal distribution for given mean and
 * variance until third standard deviation.
 *
 * @param      param
 *
 * @return     return type
 */
struct normal_distribution *generate_normal_distribution(
                                  size_t buckets, uint32_t mean,
                                  double std_dev);

void destroy_normal_distribution(struct normal_distribution *distribution);

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      uint32_t x);

void initialize_particles(struct particle *particles, size_t amount);

double **calculate_likelihood(uint32_t measurement, struct particle *particles,
                              struct particle *particles_other, size_t amount,
                              size_t amount_other);



#endif /* PARTICLE_FILTER_H */
