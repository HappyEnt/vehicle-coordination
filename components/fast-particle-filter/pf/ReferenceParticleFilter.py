from AbstractParticleFilter import AbstractParticleFilter

import random
from scipy.stats import norm
from scipy.spatial import distance

class ReferenceParticleFilter(AbstractParticleFilter):
    particles = []
    MEASUREMENT_STD = 0.1  # Parameter of the model
    messages = []

    def get_particles(self):
        return self.particles

    def set_particles(self, particles):
        self.particles = particles

    def predict(self, action):
        raise NotImplementedError

    def add_message(self, message):
        self.messages.append(message)

    def iterate(self):
        num_particles = len(self.particles)        
        weights = [1.0 / num_particles] * num_particles        
        for m in self.messages:

            def estimate_prob(p1, p2):
                expected_d = distance.euclidean(p1, p2)
                
                norm_val = (m.get_measured_distance()-expected_d) / self.MEASUREMENT_STD
                prob = norm.pdf(norm_val) / self.MEASUREMENT_STD # use correct way to calculate prob in order to aid comparison between implementations

                return prob            

            for (i, p) in enumerate(self.particles):
                weight_factor = 0.0
                for (k, rp) in enumerate(m.get_sender_particles()):
                    weight_factor += estimate_prob(p, rp)
                weights[i] *= weight_factor / len(m.get_sender_particles())

        self.resample(weights)
        self.messages.clear()

    def resample(self, weights):
        sum_weights = sum(weights)
        normalized_weights = [x/sum_weights for x in weights]

        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        self.particles = random.choices(
            population=self.particles,
            weights=normalized_weights,
            k=len(self.particles)
        )
            
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    def value_from_normal_distribution(self, mean, std_dev, x):
        raise NotImplementedError
