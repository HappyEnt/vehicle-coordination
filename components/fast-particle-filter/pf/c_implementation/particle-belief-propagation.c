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

/* #include <omp.h> */

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
  struct message curr;
  while(pop_message(ms, &curr) != 0) {
    free(curr.particles);
  };
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
      /* log_info("%f,%f,%f,%f", sample.x_pos, sample.y_pos, particles_other[j].x_pos, particles_other[j].y_pos); */
    }

    assert(likelihood == likelihood);

    weight_factor += likelihood;

    //add each threads partial sum to the total sum
  }
  return weight_factor / m.particles_length;
}

double message_stack_prob(struct particle_filter_instance *pf, struct particle sample, double lambda) {
  struct message_stack *current_message = pf->mstack;
  double weight_product = 1.0;


  while(current_message != NULL) {
    weight_product *= message_prob(pf, sample, current_message->item);

    current_message = current_message->next;
  }

  return pow(weight_product, 1.0/lambda);
}

void resample_local_particles(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles, size_t input_length, size_t target_length) {
  /* pf_inst->local_particles, pf_inst->local_particles_length, */
  struct particle *resampled_particles = malloc(sizeof(struct particle) * target_length);

  low_variance_resampling(weighted_particles, resampled_particles, input_length, target_length);

  pf->local_particles = resampled_particles;
  pf->local_particles_length = target_length;
}

// calculates new particle set for belief
void correct(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles, double lambda) {
  double total_weight = 0.0; // implicitly shared by being definde outside the following parallel block

  struct particle *particles = pf->local_particles;
  size_t samples = pf->local_particles_length;

  for (size_t i = 0; i < samples; ++i) {
    weighted_particles[i].particle = pf->local_particles[i];
    weighted_particles[i].weight = 1.0/samples;
  }

  for (size_t i = 0; i < samples; ++i) {
    weighted_particles[i].weight *= message_stack_prob(pf, weighted_particles[i].particle, lambda);
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
  /* return pow(components, - 1.0 / (dimensions + 4)) * 0.5; */
  /* return pow (components, -1.0/3.0) * 0.2; */
}

// for now we generate exactly the same amoutn of new samples as we had in our previous belief estimate
void regularized_reject_correct(struct particle_filter_instance *pf, double lambda) {
  struct particle *old_particles = pf->local_particles;
  size_t components = pf->local_particles_length;

  struct particle *new_particles = malloc(components * sizeof(struct particle));

  double supremum = value_from_normal_distribution(pf->uwb_error_likelihood, 0) * message_stack_len(pf->mstack);

  double variance = max_empirical_variance(old_particles, components);
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);
/* #pragma omp parallel shared(new_particles) */
/*   { */
/* #pragma omp for */
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

          double accept = message_stack_prob(pf, next_sample, lambda);

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

    for (double agg = -alpha; agg < 0 && index < components-1; agg += wp[index].weight) {
      index++;
    }

    assert(index < components);

    sample[index] += 1;
  }
}

// takes a weighted set of particles and resamples by performing kernel density estimation
void resample_local_particles_kde(struct particle_filter_instance *pf, struct weighted_particle *wp, size_t target_samples) {
  struct particle *old_particles = pf->local_particles;
  size_t components = pf->local_particles_length;

  struct particle *new_particles = malloc(target_samples * sizeof(struct particle));

  // todo has to be calculated from message
  /* double variance = max_empirical_variance(old_particles, components); */
  double variance = max_empirical_weighted_variance(wp, components);

  // chapter 12 douce et al. optimal bandwidth assuming unit gaussian distribution of underlying distribution
  double h_opt = opt_unit_gaussian_bandwidth(components, DIM);

  size_t component_freq[components];
  sample_from_multinomial_direct(wp, component_freq, components, target_samples);

  size_t sample_cnt = 0;
  for(size_t c = 0; c < components; c++) {
    for (size_t s = 0; s < component_freq[c]; s++) {
      struct particle sample;

      sample_particles_from_unit_gaussian(&sample, 1);

      new_particles[sample_cnt].x_pos = wp[c].particle.x_pos + sample.x_pos*h_opt*sqrt(variance);
      new_particles[sample_cnt].y_pos = wp[c].particle.y_pos + sample.y_pos*h_opt*sqrt(variance);

      assert(new_particles[sample_cnt].y_pos == new_particles[sample_cnt].y_pos);
      assert(new_particles[sample_cnt].x_pos == new_particles[sample_cnt].x_pos);

      sample_cnt++;
    }
  }

  assert(sample_cnt == target_samples);

  pf->local_particles = new_particles;
  pf->local_particles_length = target_samples;

  free(old_particles);
}

void sample_kde(struct message m, struct particle *target_particles, size_t target_samples) {
  if(m.type != DENSITY_ESTIMATION) {
    log_err("can only sample from message with kernel density estimation");
  }

  for(size_t s = 0; s < target_samples; s++) {
    size_t c = (double) rand() / RAND_MAX * m.particles_length;

    struct particle sample;
    sample_particles_from_unit_gaussian(&sample, 1);

    target_particles[s].x_pos = m.particles[c].x_pos + sample.x_pos * m.h_opt * sqrt(m.variance);
    target_particles[s].y_pos = m.particles[c].y_pos + sample.y_pos * m.h_opt * sqrt(m.variance);
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
    acc += value_from_independent_2D_distribution_(pos, particles[c], (sqrt(m.variance) * m.h_opt));
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

    double prob_cur = message_stack_prob(pf, current_sample, 1.0);
    double prob_next = message_stack_prob(pf, next_sample, 1.0);
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

double calculate_progressive_factor(struct particle_filter_instance *pf, double sigma_max) {
  struct message_stack *current_message = pf->mstack;
  struct particle *current_particles = pf->local_particles;
  size_t current_particles_length = pf->local_particles_length;

  double weight_product = 1.0;
  double max_prob = 0.0;

  for (size_t i = 0; i < current_particles_length; ++i) {
    double prob = -1.0 *  log(message_stack_prob(pf, current_particles[i], 1.0));
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

// Notes
// regularisation (approximation of belief) using KDE -> doucet et al.

// pre regularization belief propagation.
// see chapter 12 douce et al.
// This is also not yet the version that autonomously finds the best message order.
void pre_regularisation_bp(struct particle_filter_instance *pf_inst) {
  size_t M = pf_inst->local_particles_length;
  struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * M);

  // First correct
  correct(pf_inst, wp, 1.0);

  // than perform resampling through density estimation
  resample_local_particles_kde(pf_inst, wp, M);

  free(wp);

  clear_message_stack(&pf_inst->mstack);
}

void post_regularisation_bp(struct particle_filter_instance *pf_inst) {
  size_t M = pf_inst->local_particles_length;

  regularized_reject_correct(pf_inst, 1.0);

  clear_message_stack(&pf_inst->mstack);
}

void progressive_post_regularisation_bp(struct particle_filter_instance *pf_inst, double sigma_max, int n_max) {
  int n;
  double lambda;


  // initial conditions
  n = 0;
  lambda = 1000;
  // start with high lambda to start of

  do {
    if(n >= n_max || lambda < 1.0) {
      lambda = 1.0;
      n = n_max;
    }

    debug("lambda = %f", lambda);

    regularized_reject_correct(pf_inst, lambda);

    lambda = calculate_progressive_factor(pf_inst, sigma_max);

    n++;
  } while (n <= n_max);


  clear_message_stack(&pf_inst->mstack);
}

void progressive_pre_regularisation_bp(struct particle_filter_instance *pf_inst, double sigma_max, int n_max) {
  size_t M = pf_inst->local_particles_length;
  int n;
  double lambda;

  n = 0;
  lambda = 100;

  do {
    if(n >= n_max-1 || lambda < 1.0) {
      lambda = 1.0;
      n = n_max;
    }

    struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * M);

    // First correct
    correct(pf_inst, wp, lambda);

    // than perform resampling through density estimation
    resample_local_particles_kde(pf_inst, wp, M);

    free(wp);

    // calculate next lambda
    lambda = calculate_progressive_factor(pf_inst, sigma_max);
    debug("lambda = %f", lambda);

    //TODO break on lamdba < 1.0

    n++;
  } while(n <= n_max);


  clear_message_stack(&pf_inst->mstack);
}

void multiply_messages(struct particle_filter_instance *pf_inst, struct weighted_particle *wp,  struct particle *proposal_particles, size_t proposal_amount) {
  for (size_t i = 0; i < proposal_amount; ++i) {
    wp[i].particle = proposal_particles[i];
    wp[i].weight = 1.0/proposal_amount; // this is probably not needed as we are normalizing at the end anyway
  }

  for (size_t p = 0; p < proposal_amount; p++) {
    wp[p].weight *= evaluate_message_stack_at(pf_inst->mstack, wp[p].particle);
  }

  // normalize weights
  double total_weight = 0;
  for (size_t i = 0; i < proposal_amount; ++i) {
    total_weight += wp[i].weight;
  }

  for (size_t i = 0; i < proposal_amount; ++i) {
    wp[i].weight /= total_weight;
  }
}

void upsample_message_stack(struct message_stack *ms, size_t target_amount) {
  struct message_stack *current_message = ms;

  while(current_message != NULL) {
    size_t current_length = current_message->item.particles_length;
    if(current_length < target_amount) {
      struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * current_length);
      struct particle *new_particles = malloc(sizeof(struct particle) * target_amount);

      for(size_t p = 0; p < current_length; p++) {
        wp[p].particle = current_message->item.particles[p];
        wp[p].weight = 1.0/current_length;
      }

      low_variance_resampling(wp, new_particles, current_length, target_amount);

      free(current_message->item.particles);
      current_message->item.particles_length = target_amount;
      current_message->item.particles = new_particles;
    }
    current_message = current_message->next;
  }
}

// convert belief received from other node into message by incorporating measurement
void belief_to_message(struct message *m, double std_dev) {
  // modify message in place
  size_t components = m->particles_length;
  struct particle *particles = m->particles;

  for(size_t c = 0; c < components; c++) {
    struct particle tmp_p = particles[c];

    double angle = ((double) rand() / RAND_MAX) * 2 * M_PI;

    tmp_p.x_pos += m->measured_distance * cos(angle);
    tmp_p.y_pos += m->measured_distance * sin(angle);
    sample_particles_from_gaussian(tmp_p, std_dev, particles + c, 1);
  }

  m->h_opt = opt_unit_gaussian_bandwidth(components, DIM);
  m->variance = max_empirical_variance(particles, components);
  m->type = DENSITY_ESTIMATION;

  log_info("h_opt %f", m->h_opt);
}


void generate_messages_from_beliefs(struct particle_filter_instance *pf_inst) {
  // despite the nomenclature I used before, until now we only have beliefs stored in the message sdtack
  // this procedure converts the beliefs into proper messages
  struct message_stack *current_message = pf_inst->mstack;

  while(current_message != NULL) {
    belief_to_message(&current_message->item, pf_inst->uwb_error_likelihood->std_dev);

    current_message = current_message->next;
  }
}

void non_parametric_bp(struct particle_filter_instance *pf_inst) {
  // step 1 upsample messages from message stack if necessary
  // this is mostly needed to get anchors to work which only send one particle
  upsample_message_stack(pf_inst->mstack, pf_inst->local_particles_length);

  // step 2 use ONE message as proposal distribution
  generate_messages_from_beliefs(pf_inst);

  // generate proposal samples
  size_t proposal_amount = pf_inst->local_particles_length;
  struct particle *proposal_particles = malloc(sizeof(struct particle) * proposal_amount);
  struct message proposal_distribution = pf_inst->mstack->item;   // for now use the first message we received as proposal distribution

  if (message_stack_len(pf_inst->mstack) <= 0) {
    log_err("empty message stack");
  }

  sample_kde(proposal_distribution, proposal_particles, proposal_amount);

  // add current belief as message (we perform prediction and correction asynchronously)
  struct message own_belief_m;
  own_belief_m.particles = pf_inst->local_particles;
  own_belief_m.particles_length = pf_inst->local_particles_length;
  own_belief_m.h_opt = opt_unit_gaussian_bandwidth(pf_inst->local_particles_length, DIM);
  own_belief_m.variance = max_empirical_variance(own_belief_m.particles, own_belief_m.particles_length);
  own_belief_m.type = DENSITY_ESTIMATION;

  add_belief(pf_inst, own_belief_m);

  // multiply messages
  struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * proposal_amount);

  multiply_messages(pf_inst, wp, proposal_particles, proposal_amount);

  // resample
  resample_local_particles_kde(pf_inst, wp, proposal_amount);
  /* resample_local_particles(pf_inst, wp, proposal_amount, proposal_amount); */

  // clear
  clear_message_stack(&pf_inst->mstack);
  free(wp);
}


void iterate(struct particle_filter_instance *pf_inst) {
  /* pre_regularisation_bp(pf_inst); */
  if(!message_stack_len(pf_inst->mstack)) {
    log_info("empty message stack. Doing nothing");
  } else {
    /* post_regularisation_bp(pf_inst); */
    /* progressive_post_regularisation_bp(pf_inst, 10.0, 10); */
    /* progressive_pre_regularisation_bp(pf_inst, 10.0, 25); */
    non_parametric_bp(pf_inst);
  }
}

void predict(struct particle_filter_instance *pf_inst, double moved_distance) {
  for (size_t p = 0; p < pf_inst->local_particles_length; p++) {
    double dir = (((double) rand())/(RAND_MAX)) * 2* M_PI;

    struct particle *current_particle  = &pf_inst->local_particles[p];

    // TODO add noise

    current_particle->x_pos = current_particle->x_pos + moved_distance * sin(dir);
    current_particle->y_pos = current_particle->y_pos + moved_distance * cos(dir);
  }
}
