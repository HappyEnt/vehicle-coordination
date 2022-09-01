#include "util.h"
#include "particle-belief-propagation.h"

#include "debug.h"

/* #include <exception.h> */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <memory.h>
#include <assert.h>
#include <float.h>
#include <gsl/gsl_randist.h>

/* #include <omp.h> */

#define USE_CACHE 0
#define DIM 2

// structure definitions
struct message_stack {
  struct message item;
  struct message_stack *next;
};

struct particle_filter_instance {
  // the minimum we have to store is the particle set
  struct particle *local_particles;
  size_t local_particles_length;
  bool has_prior;

  enum filter_type type;
  struct message_stack *mstack;

  struct normal_distribution *uwb_error_likelihood;
  gsl_rng *r;

};

// protos
void belief_to_message(struct particle_filter_instance *pf, struct message *m, double std_dev);
void sample_kde(struct particle_filter_instance *pf, struct message m, struct particle *target_particles, size_t target_samples);
void upsample_message(struct particle_filter_instance *pf, struct message *m, size_t target_amount);
void low_variance_resample_local_particles_kde(
    struct particle_filter_instance *pf, struct particle *input_particles,
    struct particle *resampled_particles, size_t input_length,
    size_t target_length);
void __sample_from_unit_gaussian(struct particle_filter_instance *pf, struct particle *ps, size_t amount);
void generate_messages_from_beliefs(struct particle_filter_instance *pf_inst, struct message_stack *mstack);

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
  struct message curr;
  while(pop_message(ms, &curr) != 0) {
    free(curr.particles);
  };
}

void normalize_weights(struct particle *particles, size_t amount) {
  // normalize weights
  double total_weight = 0;
  for (size_t i = 0; i < amount; ++i) {
    total_weight += particles[i].weight;
  }

  for (size_t i = 0; i < amount; ++i) {
    particles[i].weight /= total_weight;
  }
}


void low_variance_resampling(struct particle_filter_instance *pf, struct particle *input_particles, struct particle* resampled_particles, size_t input_length, size_t target_length) {
  double U, r, c;
  size_t i;

  r =  gsl_rng_uniform(pf->r) * (1.0/target_length);
  U = 0;
  c = input_particles[0].weight;

  i = 0;
  for (size_t m = 0; m < target_length; ++m) {
    U = r + ((double) m)/target_length;
    while (U > c) {
      i++;
      c += input_particles[i].weight;
    }

    resampled_particles[m] = input_particles[i];
    resampled_particles[m].weight = 1.0/target_length;
  }
}

void multinomial_resampling(struct particle_filter_instance *pf, struct particle *input_particles, struct particle *resampled_particles, size_t input_length, size_t target_length) {
  double weights[input_length];
  unsigned int sample[input_length];

  for(size_t w = 0; w < input_length; ++w) {
    weights[w] = input_particles[w].weight;
  }

  gsl_ran_multinomial(pf->r, input_length, target_length, weights, sample);

  size_t sampled = 0;
  for(size_t s = 0; s < input_length; ++s) {
    for(size_t r = 0; r < sample[s]; ++r) {
      resampled_particles[sampled] = input_particles[s];
      sampled++;
    }
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
      /* log_info("%f,%f,%f,%f", sample.x_pos, sample.y_pos, particles_other[j].x_pos, particles_other[j].y_pos); */
    }

    assert(likelihood == likelihood);

    weight_factor += likelihood / particles_other->weight;

    //add each threads partial sum to the total sum
  }
  return weight_factor / m.particles_length;
}

double message_stack_prob(struct particle_filter_instance *pf, struct message_stack *mstack ,struct particle sample, double lambda) {
  struct message_stack *current_message = mstack;
  double weight_product = 1.0;

  while(current_message != NULL) {
    weight_product *= message_prob(pf, sample, current_message->item);

    current_message = current_message->next;
  }

  return pow(weight_product, 1.0/lambda);
}

void resample_local_particles(struct particle_filter_instance *pf, size_t target_length) {
  struct particle *new_particles = malloc(target_length * sizeof(struct particle));

  low_variance_resample_local_particles_kde(pf, pf->local_particles, new_particles, pf->local_particles_length, target_length);
  //  // multinomial_resampling(pf, pf->local_particles, new_particles, pf->local_particles_length, target_length);

  pf->local_particles = new_particles;
  pf->local_particles_length = target_length;
}

void resample_local_particles_with_replacement (struct particle_filter_instance *pf, size_t target_length) {
  struct particle *new_particles = malloc(target_length * sizeof(struct particle));

  low_variance_resampling(pf, pf->local_particles, new_particles, pf->local_particles_length, target_length);

  pf->local_particles = new_particles;
  pf->local_particles_length = target_length;
}


// calculates new particle set for belief
void correct(struct particle_filter_instance *pf, double lambda) {
  double total_weight = 0.0; // implicitly shared by being definde outside the following parallel block

  struct particle *particles = pf->local_particles;
  size_t samples = pf->local_particles_length;

  for (size_t i = 0; i < samples; ++i) {
    particles[i].weight *= message_stack_prob(pf, pf->mstack, particles[i], lambda);
  }

  normalize_weights(particles, samples);
}

double max_empirical_variance(struct particle *particles, size_t components) {
  // todo has to be calculated from message
  double variance_x = 0, variance_y = 0;
  struct particle mean;

  mean.x_pos = 0;
  mean.y_pos = 0;

  for(size_t i = 0; i < components; i++) {
    mean.x_pos += particles[i].weight * particles[i].x_pos;
    mean.y_pos += particles[i].weight * particles[i].y_pos;
  }

  for(size_t i = 0; i < components; i++) {
    variance_x += particles[i].weight * (particles[i].x_pos - mean.x_pos)*(particles[i].x_pos - mean.x_pos);
    variance_y += particles[i].weight * (particles[i].y_pos - mean.y_pos)*(particles[i].y_pos - mean.y_pos);
  }

  double variance = variance_x > variance_y ? variance_x : variance_y;

  return variance;
}


double opt_unit_gaussian_bandwidth(size_t components, size_t dimensions) {
  return pow(components, - 1.0 / (dimensions + 4)) * pow((4.0/(dimensions + 2)), 1.0 / (dimensions + 4)) * 0.5;
    /* return pow (components, -1.0/3.0) * 0.2; */
}

// we want to use the random number generator stored inside our particle filter instance.
void __sample_from_unit_gaussian(struct particle_filter_instance *pf, struct particle *ps, size_t amount) {
  gsl_vector *mean = gsl_vector_calloc(2); // calloc already initializes the elements with 0
  gsl_vector *result = gsl_vector_calloc(2);
  gsl_matrix *L = gsl_matrix_calloc(2, 2);

  gsl_matrix_set(L, 0, 0, 1);
  gsl_matrix_set(L, 1, 1, 1);

  gsl_ran_multivariate_gaussian(pf->r, mean, L, result);

  ps->x_pos = gsl_vector_get(result, 0);
  ps->y_pos = gsl_vector_get(result, 1);

  gsl_vector_free(mean);
  gsl_vector_free(result);
  gsl_matrix_free(L);
}

// for now we generate exactly the same amoutn of new samples as we had in our previous belief estimate
void regularized_reject_correct(struct particle_filter_instance *pf, double lambda) {
  struct particle *old_particles = pf->local_particles;
  size_t components = pf->local_particles_length;

  struct particle *new_particles = malloc(components * sizeof(struct particle));

  double supremum = value_from_normal_distribution(pf->uwb_error_likelihood, 0) * message_stack_len(pf->mstack);

  double variance = max_empirical_variance(old_particles, components);
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);

  log_info("standard dev: %f", sqrt(variance));
/* #pragma omp parallel shared(new_particles) */
/*   { */
/* #pragma omp for */
    {
      for(size_t c = 0; c < components; c++) {
        while(true) {
          size_t I =  gsl_rng_uniform(pf->r) * (components);
          double U = gsl_rng_uniform(pf->r);
          struct particle sample;
          __sample_from_unit_gaussian(pf, &sample, 1);


          struct particle next_sample;
          next_sample.x_pos = old_particles[I].x_pos + sample.x_pos * h_opt * sqrt(variance);
          next_sample.y_pos = old_particles[I].y_pos + sample.y_pos * h_opt * sqrt(variance);
          next_sample.weight = 1.0 / components;

          double accept = message_stack_prob(pf, pf->mstack, next_sample, lambda);

          if(accept > U * supremum) {
            new_particles[c] = next_sample;
            break;
          }
        }
      }
    }
  /* } */

  pf->local_particles = new_particles;
  pf->local_particles_length = components;

  free(old_particles);
}

void sample_from_multinomial(struct particle *particles, unsigned int *sample, size_t components, size_t repetetions) {
  double weights[components];

  for(size_t c = 0; c < components; c++) {
    weights[c] = particles[c].weight;
  }

  gsl_rng * r = gsl_rng_alloc (gsl_rng_taus);
  gsl_ran_multinomial(r, components, components, weights, sample);
}


// takes a weighted set of particles and resamples by performing kernel density estimation
// samples from multinomial distribution. In practice the low-variance version below works better.
void resample_local_particles_kde(struct particle_filter_instance *pf, size_t target_samples) {
  struct particle *particles = pf->local_particles;
  size_t components = pf->local_particles_length;

  struct particle *new_particles = malloc(target_samples * sizeof(struct particle));

  // todo has to be calculated from message
  /* double variance = max_empirical_variance(old_particles, components); */
  double variance = max_empirical_variance(particles, components);

  // chapter 12 douce et al. optimal bandwidth assuming unit gaussian distribution of underlying distribution
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);

  unsigned int component_freq[components];
  sample_from_multinomial(particles, component_freq, components, target_samples);



  size_t sample_cnt = 0;
  for(size_t c = 0; c < components; c++) {
    for (size_t s = 0; s < component_freq[c]; s++) {
      struct particle sample;

      __sample_from_unit_gaussian(pf, &sample, 1);

      new_particles[sample_cnt].x_pos = particles[c].x_pos + sample.x_pos*h_opt*sqrt(variance);
      new_particles[sample_cnt].y_pos = particles[c].y_pos + sample.y_pos*h_opt*sqrt(variance);
      new_particles[sample_cnt].weight = 1.0/target_samples;

      assert(new_particles[sample_cnt].y_pos == new_particles[sample_cnt].y_pos);
      assert(new_particles[sample_cnt].x_pos == new_particles[sample_cnt].x_pos);

      sample_cnt++;
    }
  }

  assert(sample_cnt == target_samples);

  pf->local_particles = new_particles;
  pf->local_particles_length = target_samples;

  free(particles);
}

void low_variance_resample_local_particles_kde(
    struct particle_filter_instance *pf, struct particle *input_particles,
    struct particle *resampled_particles, size_t input_length,
    size_t target_length) {

  struct particle *particles = pf->local_particles;
  size_t components = pf->local_particles_length;



  // todo has to be calculated from message
  /* double variance = max_empirical_variance(old_particles, components); */
  double variance = max_empirical_variance(particles, components);

  // chapter 12 douce et al. optimal bandwidth assuming unit gaussian distribution of underlying distribution
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);

  double U, r, c;
  size_t i;

  r =  gsl_rng_uniform(pf->r) * (1.0/target_length);
  U = 0;
  c = input_particles[0].weight;

  i = 0;
  for (size_t m = 0; m < target_length; ++m) {
    U = r + ((double) m)/target_length;
    while (U > c) {
      i++;
      c += input_particles[i].weight;
    }

    struct particle sample;

    __sample_from_unit_gaussian(pf, &sample, 1);

    resampled_particles[m].x_pos = particles[i].x_pos + sample.x_pos*h_opt*sqrt(variance);
    resampled_particles[m].y_pos = particles[i].y_pos + sample.y_pos*h_opt*sqrt(variance);
    resampled_particles[m].weight = 1.0/target_length;
  }

  pf->local_particles = resampled_particles;
  pf->local_particles_length = target_length;

  free(particles);
}

void sample_kde(struct particle_filter_instance *pf, struct message m, struct particle *target_particles, size_t target_samples) {
  if(m.type != DENSITY_ESTIMATION) {
    log_err("can only sample from message with kernel density estimation");
  }

  for(size_t s = 0; s < target_samples; s++) {
    size_t c = gsl_rng_uniform(pf->r) * m.particles_length;

    struct particle sample;
    __sample_from_unit_gaussian(pf, &sample, 1);

    target_particles[s].x_pos = m.particles[c].x_pos + sample.x_pos * m.h_opt * sqrt(m.variance);
    target_particles[s].y_pos = m.particles[c].y_pos + sample.y_pos * m.h_opt * sqrt(m.variance);
    target_particles[s].weight = 1.0/target_samples;
  }
}

// this is similar to message_prob, but message prob implements whats described in particle belief propagation [ihler09]
// while this implements the scheme similar to NBP [ihler05]
double evaluate_message_at(struct message m, struct particle pos) {
  size_t components = m.particles_length;
  struct particle *particles = m.particles;

  double acc = 0.0;

  for (size_t c = 0; c < components; ++c) {
    // for  now only consider normal distributons with covariate matrices of the form Sigma = (sigma * I)
    acc += particles[c].weight * value_from_independent_2D_distribution_(pos, particles[c], (sqrt(m.variance) * m.h_opt));
  }

  return acc / components;
}

double evaluate_message_stack_at(struct message_stack *ms, struct particle pos) {
  struct message_stack *current_message = ms;

  double acc = 1.0;
  while (current_message != NULL) {
    acc *= evaluate_message_at(current_message->item, pos);

    current_message = current_message->next;
  }

  return acc;
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

    double prob_cur = message_stack_prob(pf, pf->mstack, current_sample, 1.0);
    double prob_next = message_stack_prob(pf, pf->mstack, next_sample, 1.0);
    double accept = prob_next / prob_cur;
    double alpha = gsl_rng_uniform(pf->r);

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

void move_metropolis(struct particle_filter_instance *pf, size_t burn_in_per_sample) {
  struct particle *particles = pf->local_particles;
  size_t components = pf->local_particles_length;

  for(size_t i = 0; i < components; i++) {
    struct particle current_sample = particles[0];

    for(size_t s = 0; s < burn_in_per_sample; s++) {
      struct particle next_sample;


      sample_particles_from_gaussian(current_sample, 4, &next_sample, 1);

      double prob_cur = message_stack_prob(pf, pf->mstack, current_sample, 1.0);
      double prob_next = message_stack_prob(pf, pf->mstack, next_sample, 1.0);
      double accept = prob_next / prob_cur;

      accept = accept < 1.0 ? accept : 1.0;

      double alpha = gsl_rng_uniform(pf->r);

      /* log_info("probs: %f / %f", prob_next, prob_cur); */

      if (alpha < accept || prob_cur < 1e-12) {
        current_sample = next_sample;
      }
    }
    particles[i] = current_sample;
  }
}


double calculate_progressive_factor(struct particle_filter_instance *pf, double sigma_max) {
  struct message_stack *current_message = pf->mstack;
  struct particle *current_particles = pf->local_particles;
  size_t current_particles_length = pf->local_particles_length;

  double weight_product = 1.0;
  // set to smallest double
  double max_prob = -DBL_MAX;

  for (size_t i = 0; i < current_particles_length; ++i) {
    double prob = -1.0 *  log(message_stack_prob(pf, pf->mstack, current_particles[i], 1.0));
    if (prob > max_prob && !isinf(prob)) {
      max_prob = prob;
    }
  }

  return max_prob / log(sigma_max);
  /* return max_prob; */
}


// _____Implementation of public particle filter interface_____

// TODO pass sensor parameters through structure
void create_particle_filter_instance(struct particle_filter_instance **pf_inst) {
  struct particle_filter_instance *ret_inst = malloc(sizeof(struct particle_filter_instance));

  ret_inst->local_particles = NULL;
  ret_inst->uwb_error_likelihood = generate_normal_distribution(0, 0.1, USE_CACHE);
  ret_inst->mstack = NULL;
  ret_inst->has_prior = false;

  ret_inst->r = gsl_rng_alloc (gsl_rng_taus);
  gsl_rng_set(ret_inst->r, time(0));

  ret_inst->type = POST_REGULARIZATION;

  *pf_inst = ret_inst;
}

void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst) {
  /* if(pf_inst->local_particles != NULL) */
  /*   free(pf_inst->local_particles); */

  free(pf_inst);
}

void set_filter_type(struct particle_filter_instance *pf_inst, enum filter_type type) {
  pf_inst->type = type;
}

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length) {
  struct particle *pt_cpy = malloc(sizeof(struct particle) * length);

  memcpy(pt_cpy, particles, sizeof(struct particle) * length);

  pf_inst->local_particles = pt_cpy;
  pf_inst->local_particles_length = length;
  pf_inst->has_prior = true;
}

void set_particle_amount(struct particle_filter_instance *pf_inst, size_t amount) {
  pf_inst->local_particles_length = amount;
}


// on success returns length of particles, negative numbers indicate error
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles) {
  if(pf_inst->local_particles == NULL) {
    return -1;
  }

  *particles = pf_inst->local_particles;

  return pf_inst->local_particles_length;
}

void add_belief(struct particle_filter_instance *pf_inst, struct message m) {
  // take ownership of data
  struct particle *particles = malloc(sizeof(struct particle) * m.particles_length);
  memcpy(particles, m.particles, sizeof(struct particle) * m.particles_length);

  struct message m_cpy = {
    .measured_distance = m.measured_distance,
    .particles = particles,
    .particles_length = m.particles_length,
    .h_opt = m.h_opt,
    .variance = m.variance,
    .type = m.type
  };

  push_message(&pf_inst->mstack, m_cpy);
}

void deep_copy_message(struct message *dest_message, struct message source_message) {
  struct particle *particle_cpy = malloc(sizeof(struct particle) * source_message.particles_length);
  *dest_message = source_message;
  dest_message->particles = particle_cpy;

  memcpy(particle_cpy, source_message.particles, sizeof(struct particle) * source_message.particles_length);
}

void redistribute_particles(struct particle_filter_instance *pf_inst, struct message proposal_distribution, double fraction) {
  size_t start_index, amount;
  struct message m_cp = proposal_distribution;
  struct particle *current_particles = malloc(sizeof(struct particle) * pf_inst->local_particles_length);
  memcpy(current_particles, pf_inst->local_particles, sizeof(struct particle) * pf_inst->local_particles_length);

  if (proposal_distribution.type != DENSITY_ESTIMATION) {
    struct particle *p_cp = malloc(sizeof(struct particle) * m_cp.particles_length);
    memcpy(p_cp, m_cp.particles, sizeof(struct particle) * m_cp.particles_length);
    m_cp.particles = p_cp;

    upsample_message(pf_inst, &m_cp, pf_inst->local_particles_length);

    belief_to_message(pf_inst, &m_cp, pf_inst->uwb_error_likelihood->std_dev);
  }

  amount = pf_inst->local_particles_length * fraction;
  start_index = pf_inst->local_particles_length - amount;

  assert (amount + start_index <= pf_inst->local_particles_length);

  sample_kde(pf_inst, m_cp, pf_inst->local_particles + start_index, amount);

  // create copy of message stack
  struct message_stack *stack_cpy = NULL;
  struct message_stack *current_entry;

  current_entry = pf_inst->mstack;
  while(current_entry != NULL) {
    struct message m;

    deep_copy_message(&m, current_entry->item);

    upsample_message(pf_inst, &m, pf_inst->local_particles_length);

    push_message(&stack_cpy, m);

    current_entry = current_entry->next;
  }

  generate_messages_from_beliefs(pf_inst, stack_cpy);

  // if a current belief already exists, take it into consideration
  if(pf_inst->has_prior && false) {
    struct message own_belief_m;
    own_belief_m.particles = current_particles;
    own_belief_m.particles_length = pf_inst->local_particles_length;
    own_belief_m.h_opt = opt_unit_gaussian_bandwidth(own_belief_m.particles_length, DIM);
    own_belief_m.variance = max_empirical_variance(own_belief_m.particles, own_belief_m.particles_length);
    own_belief_m.type = DENSITY_ESTIMATION;

    push_message(&stack_cpy, own_belief_m);
  }

  /* for(size_t p = 0; p < amount; p++) { */
  /*   pf_inst->local_particles[start_index + p].weight = message_stack_prob(pf_inst, pf_inst->mstack, pf_inst->local_particles[start_index + p], 1.0); */
  /* } */

  /* for(size_t p = 0; p < amount; p++) { */
  /*   pf_inst->local_particles[start_index + p].weight = evaluate_message_stack_at(stack_cpy, pf_inst->local_particles[start_index + p]); */
  /* } */

  // set uniform weight
  for(size_t p = 0; p < pf_inst->local_particles_length; p++) {
    pf_inst->local_particles[p].weight = 1.0/pf_inst->local_particles_length;
  }

  for(size_t p = 0; p < pf_inst->local_particles_length; p++) {
    pf_inst->local_particles[p].weight *= evaluate_message_stack_at(stack_cpy, pf_inst->local_particles[p]);
  }

  normalize_weights(pf_inst->local_particles, pf_inst->local_particles_length);

  clear_message_stack(&stack_cpy);
  free(m_cp.particles);
}

double effective_sample_size_estimator(struct particle_filter_instance *pf_inst) {
  double sum = 0.0;
  for (size_t i = 0; i < pf_inst->local_particles_length; ++i) {
    sum += pf_inst->local_particles[i].weight*pf_inst->local_particles[i].weight;
  }

  return 1.0/sum;
}


// Notes
// regularisation (approximation of belief) using KDE -> doucet et al.

// pre regularization belief propagation.
// see chapter 12 douce et al.
// This is also not yet the version that autonomously finds the best message order.
void pre_regularisation_bp(struct particle_filter_instance *pf_inst) {
  size_t M = pf_inst->local_particles_length;

  /* redistribute_particles(pf_inst, pf_inst->mstack->item, 0.04); // redistribute */

  // First correct
  correct(pf_inst, 1.0);

  double ess = effective_sample_size_estimator(pf_inst);

  log_info("ESS: %f", ess);

  // than perform resampling through density estimation
  if(ess < (double) M / 2) {
    resample_local_particles(pf_inst, M);
  }

  clear_message_stack(&pf_inst->mstack);
}

void post_regularisation_bp(struct particle_filter_instance *pf_inst) {
  size_t M = pf_inst->local_particles_length;

  /* redistribute_particles(pf_inst, pf_inst->mstack->item, 0.1); // redistribute */

  regularized_reject_correct(pf_inst, 1.0);

  clear_message_stack(&pf_inst->mstack);
}

void progressive_post_regularisation_bp(struct particle_filter_instance *pf_inst, double sigma_max, int n_max) {
  int n;
  double lambda;


  // initial conditions
  n = 0;
  /* lambda = 2; */
  // start with high lambda to start of
  lambda = calculate_progressive_factor(pf_inst, sigma_max);

  // add prediction message
  /* struct message own_belief_m; */
  /* own_belief_m.particles = pf_inst->local_particles; */
  /* own_belief_m.particles_length = pf_inst->local_particles_length; */
  /* own_belief_m.h_opt = opt_unit_gaussian_bandwidth(pf_inst->local_particles_length, DIM); */
  /* own_belief_m.variance = max_empirical_variance(own_belief_m.particles, own_belief_m.particles_length); */
  /* own_belief_m.type = DENSITY_ESTIMATION; */

  /* add_belief(pf_inst, own_belief_m); */

  do {
    if(n >= n_max || lambda < 1.0) {
      lambda = 1.0;
      n = n_max;
    }

    debug("lambda = %f", lambda);

    /* redistribute_particles(pf_inst, pf_inst->mstack->item, 0.1); // redistribute */

    regularized_reject_correct(pf_inst, lambda);

    lambda = calculate_progressive_factor(pf_inst, sigma_max);

    n++;
  } while (n <= n_max);


  clear_message_stack(&pf_inst->mstack);
}

void progressive_pre_regularisation_bp(struct particle_filter_instance *pf_inst, double sigma_max, int n_max) {
  size_t M = pf_inst->local_particles_length;
  int n;
  double lambda, lambda_acc;

  n = 0;
  lambda = calculate_progressive_factor(pf_inst, sigma_max);
  lambda_acc = 0.0;

  // redistribute according to first message received
  /* redistribute_particles(pf_inst, pf_inst->mstack->item, 0.1); // redistribute */

  do {
    if(n >= n_max-1 || lambda < 1.0) {
      lambda = 1 - lambda_acc;
      n = n_max;
    }

    // redistribute some particles
    /* redistribute_particles(pf_inst, pf_inst->mstack->item, 0.3); // redistribute */

    // First correct
    correct(pf_inst, lambda);

    // than perform resampling through density estimation
    double ess = effective_sample_size_estimator(pf_inst);
    /* if(ess < (double) M / 2) { */
      resample_local_particles(pf_inst, M);
    /* } */

    lambda_acc += 1.0/lambda;

    // calculate next lambda
    lambda = calculate_progressive_factor(pf_inst, sigma_max);
    debug("lambda = %f", lambda);

    //TODO break on lamdba < 1.0

    n++;
  } while(n <= n_max);


  clear_message_stack(&pf_inst->mstack);
}

void multiply_messages(struct particle_filter_instance *pf_inst) {
  struct particle *particles = pf_inst->local_particles;
  size_t amount = pf_inst->local_particles_length;

  for (size_t p = 0; p < amount; p++) {
    particles[p].weight *= evaluate_message_stack_at(pf_inst->mstack, particles[p]);
  }

  normalize_weights(particles, amount);
}

void upsample_message(struct particle_filter_instance *pf, struct message *m, size_t target_amount) {
  struct particle *new_particles = malloc(sizeof(struct particle) * target_amount);

  low_variance_resampling(pf, m->particles,  new_particles, m->particles_length, target_amount);
  // multinomial_resampling(pf, pf->local_particles, new_particles, pf->local_particles_length, target_amount);

  free(m->particles);
  m->particles_length = target_amount;
  m->particles = new_particles;
}

void upsample_message_stack(struct particle_filter_instance *pf, struct message_stack *ms, size_t target_amount) {
  struct message_stack *current_message = ms;

  while(current_message != NULL) {
    size_t current_length = current_message->item.particles_length;
    if(current_length < target_amount) {
      upsample_message(pf, &current_message->item, target_amount);
    }
    current_message = current_message->next;
  }
}

// convert belief received from other node into message by incorporating measurement
void belief_to_message(struct particle_filter_instance *pf, struct message *m, double std_dev) {
  // modify message in place
  size_t components = m->particles_length;
  struct particle *particles = m->particles;

  for(size_t c = 0; c < components; c++) {
    struct particle tmp_p = particles[c];

    double angle = gsl_rng_uniform(pf->r) * 2 * M_PI;

    tmp_p.x_pos += m->measured_distance * cos(angle);
    tmp_p.y_pos += m->measured_distance * sin(angle);
    sample_particles_from_gaussian(tmp_p, std_dev, particles + c, 1);
  }

  m->h_opt = opt_unit_gaussian_bandwidth(components, DIM);
  m->variance = max_empirical_variance(particles, components);
  m->type = DENSITY_ESTIMATION;
}


void generate_messages_from_beliefs(struct particle_filter_instance *pf_inst, struct message_stack *mstack) {
  // despite the nomenclature I used before, until now we only have beliefs stored in the message sdtack
  // this procedure converts the beliefs into proper messages
  struct message_stack *current_message = mstack;

  while(current_message != NULL) {
    belief_to_message(pf_inst, &current_message->item, pf_inst->uwb_error_likelihood->std_dev);

    current_message = current_message->next;
  }
}

void non_parametric_bp(struct particle_filter_instance *pf_inst) {
  /* redistribute_particles(pf_inst, pf_inst->mstack->item, 0.2); // redistribute */

  // step 1 upsample messages from message stack if necessary
  // this is mostly needed to get anchors to work which only send one particle
  upsample_message_stack(pf_inst, pf_inst->mstack, pf_inst->local_particles_length);

  // step 2 use ONE message as proposal distribution
  generate_messages_from_beliefs(pf_inst, pf_inst->mstack);

  // generate proposal samples
  size_t proposal_amount = pf_inst->local_particles_length;
  struct particle *proposal_particles = malloc(sizeof(struct particle) * proposal_amount);
  struct message proposal_distribution = pf_inst->mstack->item;   // for now use the first message we received as proposal distribution

  if (message_stack_len(pf_inst->mstack) <= 0) {
    log_err("empty message stack");
  }

  sample_kde(pf_inst, proposal_distribution, proposal_particles, proposal_amount);

  // add current belief as message (we perform prediction and correction asynchronously)
  struct message own_belief_m;
  own_belief_m.particles = pf_inst->local_particles;
  own_belief_m.particles_length = pf_inst->local_particles_length;
  own_belief_m.h_opt = opt_unit_gaussian_bandwidth(pf_inst->local_particles_length, DIM);
  own_belief_m.variance = max_empirical_variance(own_belief_m.particles, own_belief_m.particles_length);
  /* own_belief_m.variance = 0.1; */
  own_belief_m.type = DENSITY_ESTIMATION;

  // TODO TODO TODO TODO
  // Use A = min (std dev, interquartile range)  as it better handles bimodal distributions

  add_belief(pf_inst, own_belief_m);

  pf_inst->local_particles = proposal_particles;
  pf_inst->local_particles_length = proposal_amount;

  // multiply messages
  multiply_messages(pf_inst);

  // resample
  resample_local_particles(pf_inst, proposal_amount);

  // clear
  clear_message_stack(&pf_inst->mstack);
}

void resample_move_bp(struct particle_filter_instance *pf_inst) {
  // First correct
  correct(pf_inst, 1.0);

  // resample using classical low variance sampling
  resample_local_particles_with_replacement(pf_inst, pf_inst->local_particles_length);

  // move each particle (metropolis hastings)
  move_metropolis(pf_inst, 100);

  clear_message_stack(&pf_inst->mstack);
}

void sir_bp(struct particle_filter_instance *pf_inst, bool with_roughening) {
  correct(pf_inst, 1.0);

  double ess = effective_sample_size_estimator(pf_inst);

  if(ess < (double) pf_inst->local_particles_length / 2) {
    resample_local_particles_with_replacement(pf_inst, pf_inst->local_particles_length);
  }

  if(with_roughening) {
    // TODO apply noise to particles
    for (size_t p = 0; p < pf_inst->local_particles_length; p++) {
      struct particle *current_particle  = &pf_inst->local_particles[p];
      struct particle sample;
      __sample_from_unit_gaussian(pf_inst, &sample, 1);

      double h_opt = opt_unit_gaussian_bandwidth(pf_inst->local_particles_length, DIM);
      double variance = max_empirical_variance(pf_inst->local_particles, pf_inst->local_particles_length);

      /* sample_particles_from_gaussian(*current_particle, h_opt * sqrt(variance), current_particle, 1); */
      current_particle->x_pos += sample.x_pos * h_opt * sqrt(variance);
      current_particle->y_pos += sample.y_pos * h_opt * sqrt(variance);
    }
  }

  clear_message_stack(&pf_inst->mstack);
}


void iterate(struct particle_filter_instance *pf_inst) {
  /* pre_regularisation_bp(pf_inst); */
  if(!message_stack_len(pf_inst->mstack)) {
    log_info("empty message stack. Doing nothing");
  } else {

    if(pf_inst->local_particles_length == 0) {
      log_err("You have to call either set_particle_array or set_particle_amount before using this function");
    }

    if(pf_inst->local_particles == NULL) {
      // no prior has been set
      log_info("No prior information available. use first message to derive initial particle set");
      pf_inst->local_particles = malloc(sizeof(struct particle) * pf_inst->local_particles_length);

      redistribute_particles(pf_inst, pf_inst->mstack->item, 1.0);
      pf_inst->has_prior = true;
    }


    log_info("processing %zu messages", message_stack_len(pf_inst->mstack));

    switch (pf_inst->type) {
    case POST_REGULARIZATION: {
      post_regularisation_bp(pf_inst);
      break;
    }
    case PRE_REGULARIZATION: {
      pre_regularisation_bp(pf_inst);
      break;
    }
    case PROGRESSIVE_POST_REGULARIZATION: {
      progressive_post_regularisation_bp(pf_inst, 5.0, 5);
      break;
    }
    case PROGRESSIVE_PRE_REGULARIZATION: {
      progressive_pre_regularisation_bp(pf_inst, 5, 10);
      break;
    }
    case NON_PARAMETRIC_BP: {
      non_parametric_bp(pf_inst);
      break;
    }
    case RESAMPLE_MOVE: {
      resample_move_bp(pf_inst);
      break;
    }
    case SIR: {
      sir_bp(pf_inst, false);
      break;
    }
    case SIR_ROUGHENING: {
      sir_bp(pf_inst, true);
      break;
    }
    default:
      post_regularisation_bp(pf_inst);
      break;
    }
  }
}

void predict_dist_2D(struct particle_filter_instance *pf_inst, double moved_x, double moved_y) {
  if(pf_inst->local_particles == NULL) {
    log_info("No prior has been set yet. doing nothing");
  } else {
    for (size_t p = 0; p < pf_inst->local_particles_length; p++) {
      struct particle *current_particle  = &pf_inst->local_particles[p];

      // TODO add noise

      current_particle->x_pos = current_particle->x_pos + moved_x;
      current_particle->y_pos = current_particle->y_pos + moved_y;
    }
  }
}

void predict_dist(struct particle_filter_instance *pf_inst, double moved_distance) {
  for (size_t p = 0; p < pf_inst->local_particles_length; p++) {
    double dir = gsl_rng_uniform(pf_inst->r)* 2* M_PI;

    struct particle *current_particle  = &pf_inst->local_particles[p];

    // TODO add noise
    struct particle new_pos = pf_inst->local_particles[p];
    new_pos.x_pos = current_particle->x_pos + moved_distance * sin(dir);
    new_pos.y_pos = current_particle->y_pos + moved_distance * cos(dir);

    sample_particles_from_gaussian(new_pos, 1, current_particle, 1);
  }
}
