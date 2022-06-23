#include "particle-filter.h"

int main(int argc, char *argv[]) {
  struct particle node_1_particles[PARTICLES];
  struct particle node_2_particles[PARTICLES];

  size_t node1_amount = sizeof(node_1_particles) / sizeof(struct particle);
  size_t node2_amount = sizeof(node_1_particles) / sizeof(struct particle);

  initialize_particles(node_1_particles, node1_amount);
  initialize_particles(node_2_particles, node2_amount);

  double **result = calculate_likelihood(
      200, node_1_particles, node_2_particles, node1_amount, node2_amount);

  free(result);
  return 0;
}
