#ifndef FLOATING_POINT_PARTICLE_FILTER_H
#define FLOATING_POINT_PARTICLE_FILTER_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

#define PARTICLES 100
#define MEASUREMENT_STDEV 0.1


struct particle {
  double x_pos; // [x_pos] = m
  double y_pos; // [y_pos] = m
};

struct weighted_particle {
  struct particle particle; 
  double weight;
};

enum actions {
  CONTROL_ACTION, // for mobility model p(x_t | x_t-1)
  TDOA_MEASUREMENT, // Time difference of Arrival -> 3 Sets of particles
  TWR_MEASUREMENT,  // Two Way Ranging -> 2 Sets of particles
  SELF_MEASUREMENT, // z_self for example pedometer measurement
};

struct action {
  enum actions type;
  union {
    double distance;
    double delta_pos[2];
  } value;
};

typedef double measurement_t;
typedef double action_t[2];

struct particle_filter_instance {
  // the minimum we have to store is the particle set 
  struct particle *local_particles;
  size_t local_particles_length;
  // Any other data that is usefull
  struct normal_distribution *uwb_error_likelihood;
};

struct normal_distribution {
  double mean;
  double std_dev;

  size_t buckets;
  double bucket_size;

  double *cached_distribution;
};

struct measurement {
  double measured_distance;
  struct particle *foreign_particles;
  size_t foreign_particles_length;
};

// Public Interface

// Use create and destroy to create instance, do not free memory yourself.
void create_particle_filter_instance(struct particle_filter_instance **pf_inst);
void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst);

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length);
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles);

// TODO find out what kind of actions will exist 
void predict(struct particle_filter_instance *pf_inst, int action);
void correct(struct particle_filter_instance *pf_inst, struct measurement m);


// Test Interface
struct normal_distribution *generate_normal_distribution(
                                  double mean,
                                  double std_dev,
                                  bool cache_histogram);

void destroy_normal_distribution(struct normal_distribution *distribution);

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      double x);

void calculate_likelihood(double measurement, struct particle *particles,
                          struct particle *particles_other, size_t amount,
                          size_t amount_other,
                          struct weighted_particle *weighted_particles);

void resample(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length);

#endif /* FLOATING_POINT_PARTICLE_FILTER_H */
