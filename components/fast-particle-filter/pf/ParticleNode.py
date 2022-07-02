from Node import Node

class ParticleNode(Node):
    def __init__(self, pos) -> None:
        super().__init__(pos)
        self.particles = [] # we assume that the weights are equal for all particles

        # we initialize the particles with uniform random samples of the whole area
        for i in range(NUM_PARTICLES):
            p = tuple(np.random.uniform(low=0.0, high=SIDE_LENGTH, size=2))
            self.particles.append(p)

    def get_particles(self) -> List[Tuple[float, float]]:
        return self.particles

    def get_particles_for_exchange(self) -> List[Tuple[float, float]]:
        return random.choices(
            population = self.particles,
            k = NUM_PARTICLES_FOR_EXCHANGE
        )

    def _add_particle_noise(self):
        # We "update" the particles by adding some noise to each of them
        # Usually, we would use our update function based on the velocity and add noise with that but as we are stationary, it is just noise for now
        for (i, p) in enumerate(self.particles):
            # TODO: I am currently not sure if this POSITION_STD - is good or bad ;)
            # normally you would add this noise while resampling
            noisy_p = (np.random.normal(p[0], POSITION_STD), np.random.normal(p[1], POSITION_STD))
            self.particles[i] = noisy_p

    # Resample using the given weights (do not need to be normalized yet)
    def _resample_particles(self, weights):
        sum_weights = sum(weights)
        normalized_weights = [x/sum_weights for x in weights]

        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        self.particles = random.choices(
            population=self.particles,
            weights=normalized_weights,
            k=len(self.particles)
        )

    # we update our particles and resample them directly
    def handle_measurement(self, d, recv_particles) -> None:

        self._add_particle_noise()

        # estimate the probability P( p1, p2 | d)
        # P( p1, p2 | d) = P( d | p1, p2) * (P(p1, p2) / P(d))
        # we just assume that P(p1, p2) and P(d) are uniform and therefore all particles share this as the same factor
        # as we normalize the weights, we can ignore this factor and can just use P( d | p1, p2) which we can easily compute
        def estimate_prob(p1, p2):
            # the expected distance for the positions
            expected_d = distance.euclidean(p1, p2)
            actual_measured_d = d
            # normalize the value using the mean ("expected") and the standard deviation
            norm_val = (actual_measured_d-expected_d) / MEASUREMENT_STD
            prob = norm.pdf(norm_val) / MEASUREMENT_STD
            return prob

        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in the particles list
        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

        for (i, p) in enumerate(self.particles):
            weight_factor = 0.0
            for (k, rp) in enumerate(recv_particles):
                weight_factor += estimate_prob(p, rp)
            weights[i] *= weight_factor

        self._resample_particles(weights)

    # we update our particles and resample them directly
    def handle_passive_measurement(self, r, recv_particles_a, recv_particles_b) -> None:

        self._add_particle_noise()

        # estimate the probability P( p1, p2, p3 | z)
        # P( p1, p2, p3 | z) = P( z | p1, p2, p3) * (P(p1, p2, p3) / P(z))
        # we just assume that P(p1, p2, p3) and P(z) are uniform and therefore all particles share this as the same factor
        # as we normalize the weights, we can ignore this factor and can just use P( d | p1, p2) which we can easily compute
        def estimate_prob(p1, p2, p3):
            # the expected distance for the positions
            expected_d = distance.euclidean(p1, p2) - distance.euclidean(p1, p3)
            actual_measured_r = r
            # normalize the value using the mean ("expected") and the standard deviation
            norm_val = (actual_measured_r - expected_d) / MEASUREMENT_STD
            prob = norm.pdf(
                norm_val)  # I am not sure if we would need to divide by the MEASUREMENT_STD again - but as this linear as well we drop it :)
            return prob

        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in the particles list
        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

        for (i, p1) in enumerate(self.particles):
            weight_factor = 0.0
            for p2 in recv_particles_a:
                for p3 in recv_particles_b:
                    weight_factor += estimate_prob(p1, p2, p3)
            weights[i] *= weight_factor

        print(weights)
        self._resample_particles(weights)
