#ifndef FLOATING_POINT_PARTICLE_FILTER_H
#define FLOATING_POINT_PARTICLE_FILTER_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

struct message {
  double measured_distance;

  struct particle *particles;
  size_t particles_length;

  enum {
    DENSITY_ESTIMATION, // determines how we sample from the message
    DUMB_PARTICLES,
  } type;

  double h_opt; // only used in case of non parametric belief propagation
  double variance;
};

enum filter_type {
  POST_REGULARIZATION,
  PRE_REGULARIZATION,
  PROGRESSIVE_POST_REGULARIZATION,
  PROGRESSIVE_PRE_REGULARIZATION,
  NON_PARAMETRIC_BP,
  RESAMPLE_MOVE,
  SIR,
  SIR_ROUGHENING,
};

struct particle_filter_instance;


// _____Public Interface_____

// Use create and destroy to create instance, do not free memory yourself.
void create_particle_filter_instance(struct particle_filter_instance **pf_inst);
void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst);

void set_filter_type(struct particle_filter_instance *pf_inst, enum filter_type type);
void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length);
void set_particle_amount(struct particle_filter_instance *pf_inst, size_t amount);
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles);


void add_belief(struct particle_filter_instance *pf_inst, struct message m);

// TODO find out what kind of actions will exist
void predict_dist(struct particle_filter_instance *pf_inst, double moved_distance);
void predict_dist_2D(struct particle_filter_instance *pf_inst, double moved_x, double moved_y);
void iterate(struct particle_filter_instance *pf_inst);

// _____Test Interface_____
/* void calculate_belief(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles); */
/* void resample(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length); */

#endif /* FLOATING_POINT_PARTICLE_FILTER_H */
