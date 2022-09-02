#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define SIGN(x) ((x > 0) - (x < 0))

struct particle {
  double x_pos; // [x_pos] = m
  double y_pos; // [y_pos] = m
  double weight;
};

struct normal_distribution {
  double mean;
  double std_dev;

  size_t buckets;
  double bucket_size;

  double *cached_distribution;
};

double distance1(double x, double y);
double distance2(struct particle p1, struct particle p2);

void sample_particles_from_gaussian(struct particle mean , double std_dev, struct particle *ps, size_t amount);
void sample_particles_from_unit_gaussian(struct particle *ps, size_t amount);

struct normal_distribution *generate_normal_distribution(
                                                         double mean,
                                                         double std_dev,
                                                         bool cache_histogram);

void destroy_normal_distribution(struct normal_distribution *distribution);

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      double x);

void sample_from_2d_uniform(struct particle *target_particles, size_t amount, double lower_x, double upper_x, double lower_y, double upper_y);

double value_from_independent_2D_distribution_(struct particle x, struct particle mean, double std_dev);

#endif /* UTIL_H */
