from AbstractMeasurement import AbstractMeasurement

class TWR_Measurement(AbstractMeasurement):
    def __init__(self, particles, measured_distance):
        self.particles = particles
        self.measured_distance = measured_distance
        
        super().__init__(type = "TWR")
        
    # Returns the (possibly reduced) set of particles representing the distribution of the
    # random variable describing posterior of the Node we performed the TWR measurement with
    def get_measurement_particles(self):
        return self.particles

    def get_measured_distance(self):
        return self.measured_distance
