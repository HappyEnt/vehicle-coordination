#ifndef FLOATING_POINT_PARTICLE_FILTER_H
#define FLOATING_POINT_PARTICLE_FILTER_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

enum actions {
  CONTROL_ACTION, // for mobility model p(x_t | x_t-1)
  TDOA_MEASUREMENT, // Time difference of Arrival -> 3 Sets of particles
  TWR_MEASUREMENT,  // Two Way Ranging -> 2 Sets of particles
  SELF_MEASUREMENT, // z_self for example pedometer measurement
};

struct message {
  double measured_distance;
  struct particle *particles;
  size_t particles_length;
};

struct message_stack {
  struct message item;
  struct message_stack *next;
};

struct particle_filter_instance {
  // the minimum we have to store is the particle set
  struct particle *local_particles;
  size_t local_particles_length;
  // Any other data that is usefull
  struct normal_distribution *uwb_error_likelihood;

  struct message_stack *mstack;
};


// _____Public Interface_____

// Use create and destroy to create instance, do not free memory yourself.
void create_particle_filter_instance(struct particle_filter_instance **pf_inst);
void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst);

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length);
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles);

void add_message(struct particle_filter_instance *pf_inst, struct message m);

// TODO find out what kind of actions will exist
void predict(struct particle_filter_instance *pf_inst, double moved_distance);
void iterate(struct particle_filter_instance *pf_inst);

// _____Test Interface_____
/* void calculate_belief(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles); */
/* void resample(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length); */

#endif /* FLOATING_POINT_PARTICLE_FILTER_H */
