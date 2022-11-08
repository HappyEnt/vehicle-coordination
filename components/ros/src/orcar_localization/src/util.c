#include "util.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include "debug.h"

#include <gsl/gsl_randist.h>

static double from_normal(double x) {
  double sqpi = sqrt(2 * M_PI);
  double expc = exp(-(x*x)/2);

  return (1.0 / sqpi) * expc;
}


// see https://peterroelants.github.io/posts/multivariate-normal-primer/ for method to sample from normal distributions
// with arbitrary covariance matrix. For our cases it is enough to sample from a 2-dimensional gaussian with identity covariance matrix
// and zero mean vector
void sample_particles_from_gaussian(struct particle mean , double std_dev, struct particle *ps, size_t amount) {
  // because each dimension is independent (identiy covariance matrix) we can just  sample from each variable seperately

  for (size_t p = 0; p < amount; p++) {
    double s1 = (((double) rand() + 1)/(double)((unsigned)RAND_MAX + 2));
    double s2 = (((double) rand() + 1)/(double)((unsigned)RAND_MAX + 2));

    double mag = std_dev * sqrt(-2 * log(s1));

    ps[p].x_pos   = mag * cos(2 * M_PI * s2) + mean.x_pos;
    ps[p].y_pos   = mag * sin(2 * M_PI * s2) + mean.y_pos;

    if(ps[p].x_pos != ps[p].x_pos || ps[p].y_pos != ps[p].y_pos) {
      log_err("error in sample gaussian %f %f with input %f, %f", ps[p].x_pos, ps[p].y_pos, mean.x_pos, mean.y_pos);
    }

  }
}

// see https://peterroelants.github.io/posts/multivariate-normal-primer/ for method to sample from normal distributions
// with arbitrary covariance matrix. For our cases it is enough to sample from a 2-dimensional gaussian with identity covariance matrix
// and zero mean vector
void sample_particles_from_unit_gaussian(struct particle *ps, size_t amount) {
  // because each dimension is independent (identiy covariance matrix) we can just  sample from each variable seperately
  struct particle unit_gaus_mean;

  unit_gaus_mean.x_pos = 0;
  unit_gaus_mean.y_pos = 0;

  sample_particles_from_gaussian(unit_gaus_mean , 1, ps, amount);
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

// evaluate two dimensional normal distribution with independent variables x,y and identical variance sigma
double value_from_independent_2D_distribution_(struct particle x, struct particle mean, double std_dev) {
  double prob_x, prob_y;
  struct particle diff;

  diff.x_pos = distance1(x.x_pos, mean.x_pos);
  diff.y_pos = distance1(x.y_pos, mean.y_pos);
  prob_x = from_normal(diff.x_pos/std_dev);
  prob_y = from_normal(diff.y_pos/std_dev);
  return (prob_x * prob_y) / (std_dev*std_dev);
}

void sample_from_2d_uniform(struct particle *target_particles, size_t amount, double lower_x, double upper_x, double lower_y, double upper_y) {
  for (size_t p = 0; p < amount; p++) {
    double x = lower_x + (((double) rand())/(RAND_MAX)) * (upper_x - lower_x);
    double y = lower_y + (((double) rand())/(RAND_MAX)) * (upper_y - lower_y);

    target_particles[p].x_pos = x;
    target_particles[p].y_pos = y;
    target_particles[p].weight = 1.0/amount;
  }
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


