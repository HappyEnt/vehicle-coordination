#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <bool.h>
#include <assert.h>


enum {
  LOW_PRECISION = 8,
  MEDIUM_PRECISION = 16,
  HIGH_PRECISION = 20,
} decimal_places;

enum decimal_places fixed_point_prefix = MEDIUM_PRECISION;

struct particle {
  uint32_t x_pos; // [x_pos] = m
  uint32_t y_pos; // [y_pos] = m
};

struct weighted_particle {
  struct particle *particle; 
  uint32_t weight;
};

struct normal_distribution {
  uint32_t mean;
  uint32_t std_dev;

  size_t buckets;
  uint32_t bucket_size;

  uint32_t *cached_distribution;
};

enum actions {
  CONTROL_ACTION,
  UWB_MEASUREMENT,
};

struct action {
  enum actions type;
  union {
    uint32_t distance;
    int32_t delta_pos[2];
  } value;
};

typedef uint32_t measurement_t;
typedef int32_t action_t[2];

uint32_t to_fixed_point(uint32_t input) {
  uint32_t fixed_point_number = input >> fixed_point_prefix;
  
  assert(fixed_point_number > input);
  
  return fixed_point_number;
}

// proto
uint32_t distance1(uint32_t x, uint32_t y);
uint32_t distance2(struct particle p1, struct particle p2);

struct normal_distribution *generate_normal_distribution(
                                  uint32_t mean,
                                  uint32_t std_dev,
                                  bool cache_histogram) { // the standard deviation has also to be a integer
  static unsigned int buckets = 1000; // TODO calculate
  
  struct normal_distribution *result_distribution =
    malloc(sizeof(struct normal_distribution));
  
  if(cache_histogram) {
    result_distribution->cached_distribution = malloc(sizeof(uint32_t) * buckets);    
  } else {
    result_distribution->cached_distribution = NULL; 
  }
  
  result_distribution->bucket_size = bucket_size;
  result_distribution->std_dev = std_dev;
  result_distribution->mean = mean;

  if(cache_histogram) {
    uint32_t *cached_distribution = result_distribution->cached_distribution;
  
    for (size_t i = 0; i < buckets; ++i) {
      uint32_t x = i * bucket_size;
      cached_distribution[i] = from_normal(x);
    }
  }  

  return result_distribution;
}


uint32_t from_normal(uint32_t x) {
  static uint32_t sqpi = sqrt(2 * M_PI);
  return (to_fixed_point(1)) / (uint32_t) (sqpi) * exp(-0.5 * x);
}


void destroy_normal_distribution(struct normal_distribution *distribution) {
  free(distribution->cached_distribution);
  free(distribution);
}

uint32_t int_value_from_normal_distribution(struct normal_distribution *distribution,
                                      uint32_t x) {

  if(distribution->cached_distribution == NULL) {
    return from_normal(distance1(x/distribution->mean)/distribution->std_dev);
  }
  
  if (x > 3 * distribution->std_dev) {
    return 0.0;
  }

  uint32_t tx = distance1(distribution->mean, x) / distribution->bucket_size;
  return distribution->cached_distribution[tx]; // TODO maybe interpolate
}

double value_from_normal_distribution(struct normal_distribution *distribution, uint32_t x) {
  return (double) (int_value_from_normal_distribution(distribution, x) << fixed_point_prefix) // TODO think about this. Not right yet
}

void initialize_particles(struct particle *particles, size_t amount) {
  srand(time(NULL));

  for (size_t i = 0; i < amount; ++i) {
    particles[i].x_pos = (uint32_t)rand();
    particles[i].y_pos = (uint32_t)rand();
  }
}

uint32_t distance1(uint32_t x, uint32_t y) {
  if (x > y) {
    return x - y;
  }
  return y - x;
}

uint32_t distance2(struct particle p1, struct particle p2) {
  uint32_t dx = 0;
  uint32_t dy = 0;

  dx = distance1(p1.x_pos, p2.x_pos);
  dy = distance1(p1.y_pos, p2.y_pos);

  return sqrt(dx * dx + dy * dy);
}

struct particle *resample_particles(struct weighted_particle *particles, size_t amount) {
  struct particle *resampled_particles = malloc(sizeof(struct particle) * amount);
  for (size_t i = 0; i < amount; ++i) {
    resampled_particles[i] = *(particles[i].particle);
  }

  return resampled_particles;
}

struct weighted_particle *calculate_likelihood(uint32_t measurement, struct particle *particles,
                                               struct particle *particles_other, size_t amount,
                                               size_t amount_other) {
  struct weighted_particle *wp = malloc(sizeof(struct weighted_particle) * amount);
  
  struct normal_distribution *norm;
  norm = generate_normal_distribution(0, 1000);

  for (size_t i = 0; i < amount; ++i) {
    wp[i].particle = &particles[i];
    for (size_t j = 0; j < amount_other; ++j) {
      uint32_t likelihood = value_from_normal_distribution(
                                                         norm,
                                                         distance1(
                                                                   measurement,
                                                                   distance2(
                                                                             particles[i],
                                                                             particles[j]))); // if more precision is needed raise type to
      // uint64_t and use precision prefactor
      wp[i].weight += likelihood;
    }
  }

  destroy_normal_distribution(norm);

  /* return (double **)result_likelihoods; */
  return (struct weighted_particle *) wp;
}

struct particle *update_particle_set(struct action action, struct particle *particles,
                                     struct particle *particles_other, size_t amount,
                                     size_t amount_other) {
  struct particle *ret_p;
  
  switch (action.type) {
  case CONTROL_ACTION: // Particle Filter prediction step
    break;
  case UWB_MEASUREMENT: // Particle Filter correction step
    ;
    struct weighted_particle *wp = calculate_likelihood(action.value.distance, particles, particles_other, amount, amount_other);
    ret_p = resample_particles(wp, amount);
    break;
  default:
    break;
  }

  return ret_p;
}
