#include "util.h"
#include "particle-belief-propagation.h"

#include "debug.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <memory.h>
#include <assert.h>

#include <omp.h>

#define USE_CACHE 0

#define DIM 2

void push_message(struct message_stack **ms, struct message m) {
  if(*ms == NULL)  {
    struct message_stack *new_ms = malloc(sizeof(struct message_stack));

    new_ms->item = m;
    new_ms->next = NULL;
  
    *ms = new_ms;    
  } else {
    push_message(&(*ms)->next, m);
  }
}

size_t message_stack_len(struct message_stack *ms) {
  if (ms==NULL) return 0;
  return message_stack_len(ms->next) + 1;
}

int pop_message(struct message_stack **ms, struct message *m) {
  if(*ms != NULL) {
    if((*ms)->next == NULL) {
      struct message item;

      item = (*ms)->item;
      
      free(*ms);
      *ms = NULL;

      if(m != NULL) {
        *m = item;
      }

      return 1;
    } else {
      return pop_message(&(*ms)->next, m);
    }
  }
  return 0;
}

void clear_message_stack(struct message_stack **ms) {
  while(pop_message(ms, NULL) != 0);
}


void low_variance_resampling(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t input_length, size_t target_length) {
  double U, r, c;
  size_t i;

  r = (((double) rand())/(RAND_MAX)) * (1.0/target_length);
  U = 0;
  c = weighted_particles[0].weight;

  i = 0;
  for (size_t m = 0; m < target_length; ++m) {
    U = r + ((double) m)/target_length;
    while (U > c) {
      i++;
      c += weighted_particles[i].weight;
    }

    resampled_particles[m] = weighted_particles[i].particle;
  }
}


double message_prob(struct particle_filter_instance *pf, struct particle sample, struct message m) {
  double weight_factor = 0.0;

  size_t samples_other = m.particles_length;
  double measurement = m.measured_distance;
  struct particle *particles_other = m.particles;

  for (size_t j = 0; j < samples_other; ++j) {        
    double likelihood = value_from_normal_distribution(
                                                       pf->uwb_error_likelihood,
                                                       distance1(
                                                                 measurement,
                                                                 distance2(
                                                                           sample,
                                                                           particles_other[j])));
    if(likelihood != likelihood) {
      log_info("%f,%f,%f,%f", sample.x_pos, sample.y_pos, particles_other[j].x_pos, particles_other[j].y_pos);
    }
    
    assert(likelihood == likelihood);

    weight_factor += likelihood;

    //add each threads partial sum to the total sum
  }
  return weight_factor / m.particles_length;      
}

double message_stack_prob(struct particle_filter_instance *pf, struct particle sample) {
  struct message_stack *current_message = pf->mstack;
  double weight_product = 1.0;

 
  while(current_message != NULL) {
    weight_product *= message_prob(pf, sample, current_message->item);
    
    current_message = current_message->next;
  }

  return weight_product;
}

void resample(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles, size_t input_length, size_t target_length) {
  /* pf_inst->local_particles, pf_inst->local_particles_length, */
  struct particle *resampled_particles = malloc(sizeof(struct particle) * target_length);

  low_variance_resampling(weighted_particles, resampled_particles, input_length, target_length);

  pf->local_particles = resampled_particles;
  pf->local_particles_length = target_length;
}


// calculates new particle set for belief
void correct(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles) {
  double total_weight = 0.0; // implicitly shared by being definde outside the following parallel block

  struct particle *particles = pf->local_particles;
  size_t samples = pf->local_particles_length;
  
  for (size_t i = 0; i < samples; ++i) {  
    weighted_particles[i].particle = particles[i];
    weighted_particles[i].weight = 1.0/samples;
  }
  
  for (size_t i = 0; i < samples; ++i) {
    weighted_particles[i].weight *= message_stack_prob(pf, weighted_particles[i].particle);
  }

  for (size_t i = 0; i < samples; ++i) {  
    total_weight += weighted_particles[i].weight; // last message, i.e. weight for each particle converged to final value
    /* printf("[i] = %f\n", weighted_particles[i].weight); */
  } // printf("\n");

  for (size_t i = 0; i < samples; ++i) {
    weighted_particles[i].weight /= total_weight;
  }
}

double max_empirical_variance(struct particle *particles, size_t components) {
  // todo has to be calculated from message
  double variance_x = 0, variance_y = 0;
  struct particle mean;

  mean.x_pos = 0;
  mean.y_pos = 0;  
  
  for(size_t i = 0; i < components; i++) {
    mean.x_pos += particles[i].x_pos;
    mean.y_pos += particles[i].y_pos;    
  }

  mean.x_pos /= components;
  mean.y_pos /= components;

  for(size_t i = 0; i < components; i++) {
    variance_x += (particles[i].x_pos - mean.x_pos)*(particles[i].x_pos - mean.x_pos);
    variance_y += (particles[i].y_pos - mean.y_pos)*(particles[i].y_pos - mean.y_pos);    
  }

  variance_x /= components;
  variance_y /= components;

  double variance = variance_x > variance_y ? variance_x : variance_y;

  return variance;
}

double max_empirical_weighted_variance(struct weighted_particle *particles, size_t components) {
  // todo has to be calculated from message
  double variance_x = 0, variance_y = 0;
  struct particle mean;

  mean.x_pos = 0;
  mean.y_pos = 0;  
  
  for(size_t i = 0; i < components; i++) {
    mean.x_pos += particles[i].weight * particles[i].particle.x_pos;
    mean.y_pos += particles[i].weight * particles[i].particle.y_pos;    
  }

  for(size_t i = 0; i < components; i++) {
    variance_x += particles[i].weight * (particles[i].particle.x_pos - mean.x_pos)*(particles[i].particle.x_pos - mean.x_pos);
    variance_y += particles[i].weight * (particles[i].particle.y_pos - mean.y_pos)*(particles[i].particle.y_pos - mean.y_pos);    
  }

  double variance = variance_x > variance_y ? variance_x : variance_y;

  return variance;
}


double opt_unit_gaussian_bandwidth(size_t components, size_t dimensions) {
  return pow(components, - 1.0 / (dimensions + 4)) * pow((4.0/(dimensions + 2)), 1.0 / (dimensions + 4)) * 0.5;
}

// for now we generate exactly the same amoutn of new samples as we had in our previous belief estimate
void regularized_reject_correct(struct particle_filter_instance *pf) {
  struct particle *old_particles = pf->local_particles;
  size_t components = pf->local_particles_length;
  
  struct particle *new_particles = malloc(components * sizeof(struct particle));

  double supremum = value_from_normal_distribution(pf->uwb_error_likelihood, 0) * message_stack_len(pf->mstack);
  log_info("supremum %f", supremum);

  double variance = max_empirical_variance(old_particles, components);
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);
#pragma omp parallel shared(new_particles)
  {
#pragma omp for
    {
      for(size_t c = 0; c < components; c++) {
        while(true) {
          size_t I = (((double) rand())/(RAND_MAX)) * (components);
          double U = (((double) rand())/(RAND_MAX));
          struct particle sample;
          sample_particles_from_unit_gaussian(&sample, 1);
      
          struct particle next_sample;
          next_sample.x_pos = old_particles[I].x_pos + sample.x_pos * h_opt * sqrt(variance);
          next_sample.y_pos = old_particles[I].y_pos + sample.y_pos * h_opt * sqrt(variance);

          double accept = message_stack_prob(pf, next_sample);

          if(accept > U * supremum) {
            new_particles[c] = next_sample;
            break;
          }
        }
      }
    }
  }

  pf->local_particles = new_particles;
  pf->local_particles_length = components;

  free(old_particles);  
}
// See chapter 12 douce et al.
void progressive_weight_correction(struct particle_filter_instance *pf) {
  // a factorization of the likelihood function results naturally from the belief propagation graph
  // the following works on the assumption of gaussian error noise, but can be extended to other
  // models as well see douce et al.

  // TODO
}

// Warning: has yet to be tested
void sample_from_multinomial_direct(struct weighted_particle *wp, size_t *sample, size_t components, size_t repetetions) {
  for(size_t i = 0; i < components; ++i) {
    sample[i] = 0;
  }
  
  for (size_t i = 0; i < repetetions; ++i) {
    double alpha = (double) rand()/(RAND_MAX);
    size_t index = 0;

    for (double agg = -alpha; agg < 0 && index < repetetions; agg += wp[index].weight) {
      index++;
    }

    sample[index] += 1;
  }
}

void resample_kde(struct particle_filter_instance *pf, struct weighted_particle *wp, size_t target_samples) {
  struct particle *old_particles = pf->local_particles;
  size_t components = pf->local_particles_length;
  
  // from each message draw k * M particles
  struct particle *new_particles = malloc(target_samples * sizeof(struct particle));
  
  // todo has to be calculated from message
  double variance = max_empirical_variance(old_particles, components);
  /* double variance = max_empirical_weighted_variance(wp, components);   */

  // chapter 12 douce et al. optimal bandwidth assuming unit gaussian distribution of underlying distribution
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);

  log_info("h_opt %f", h_opt);

  size_t component_freq[components];
  sample_from_multinomial_direct(wp, component_freq, components, target_samples);

  size_t sample_cnt = 0;
  for(size_t c = 0; c < components; c++) {
    for (size_t s = 0; s < component_freq[c]; s++) {
      struct particle sample;
      
      sample_particles_from_unit_gaussian(&sample, 1);

      new_particles[sample_cnt].x_pos = wp[c].particle.x_pos + sample.x_pos*h_opt*sqrt(variance);
      new_particles[sample_cnt].y_pos = wp[c].particle.y_pos + sample.y_pos*h_opt*sqrt(variance);        

      sample_cnt++;
    }
  }

  pf->local_particles = new_particles;
  pf->local_particles_length = target_samples;

  free(old_particles);
}


void sample_metropolis(struct particle_filter_instance *pf, size_t samples, size_t burn_in) {
  struct particle *old_particles = pf->local_particles;
  size_t components = pf->local_particles_length;
  
  // from each message draw k * M particles
  struct particle *new_particles = malloc(samples * sizeof(struct particle));

  struct particle current_sample = old_particles[0];

  for(size_t i = 0; i < samples + burn_in; i++) {
    struct particle next_sample;
    
    sample_particles_from_gaussian(current_sample, 0.5, &next_sample, 1);

    double prob_cur = message_stack_prob(pf, current_sample);
    double prob_next = message_stack_prob(pf, next_sample);
    double accept = prob_next / prob_cur;
    double alpha = (((double) rand())/(RAND_MAX));

    /* log_info("probs: %f / %f", prob_next, prob_cur); */

    if (alpha < accept || prob_cur < 1e-12) {
      current_sample = next_sample;
    } else {
      /* log_err("%f %f", prob_cur, prob_next); */
    }

    if(i >= burn_in) {
      new_particles[i - burn_in] = current_sample;      
    }

  }
  
  pf->local_particles = new_particles;
  pf->local_particles_length = samples;

  free(old_particles);
}

// _____Implementation of public particle filter interface_____

// TODO pass sensor parameters through structure
void create_particle_filter_instance(struct particle_filter_instance **pf_inst) {
  struct particle_filter_instance *ret_inst = malloc(sizeof(struct particle_filter_instance));

  ret_inst->local_particles = NULL;
  ret_inst->uwb_error_likelihood = generate_normal_distribution(0, 0.1, USE_CACHE);
  ret_inst->mstack = NULL;

  srand(time(0));

  *pf_inst = ret_inst;
}

void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst) {
  /* if(pf_inst->local_particles != NULL) */
  /*   free(pf_inst->local_particles); */

  free(pf_inst);
}

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length) {
  struct particle *pt_cpy = malloc(sizeof(struct particle) * length);

  memcpy(pt_cpy, particles, sizeof(struct particle) * length);

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

void add_message(struct particle_filter_instance *pf_inst, struct message m) {
  push_message(&pf_inst->mstack, m);
}

// Notes
// regularisation (approximation of belief) using KDE -> doucet et al.

// pre regularization belief propagation.
// see chapter 12 douce et al.
// This is also not yet the version that autonomously finds the best message order.
void pre_regularisation_bp(struct particle_filter_instance *pf_inst) {
  size_t M = pf_inst->local_particles_length;
  
  struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * M);

  // First correct
  correct(pf_inst, wp);

  // than perform resampling through density estimation
  resample_kde(pf_inst, wp, M);

  correct(pf_inst, wp);  

  resample_kde(pf_inst, wp, M);
    
  clear_message_stack(&pf_inst->mstack);
}

void post_regularisation_bp(struct particle_filter_instance *pf_inst) {
  size_t M = pf_inst->local_particles_length;
  
  struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * M);

  // First correct
  regularized_reject_correct(pf_inst);
  
  clear_message_stack(&pf_inst->mstack);
}


void iterate(struct particle_filter_instance *pf_inst) {
  /* pre_regularisation_bp(pf_inst); */

  post_regularisation_bp(pf_inst);
}

void predict(struct particle_filter_instance *pf_inst, int action) {
  // TODO implement me
}
