import sys, os
from ._pf_cffi import ffi, lib

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from AbstractParticleFilter import AbstractParticleFilter
from AbstractParticleFilter import AbstractParticleFilter

class CParticleFilter(AbstractParticleFilter):
    
    cache_normal_distribution = False
    
    def __init__(self, cache_distribution):
        "create and initialize c implementation particle filter instance"
        
        self.cache_normal_distribution = cache_distribution
        pf_inst_ptr = ffi.new("struct particle_filter_instance**")        
        lib.create_particle_filter_instance(pf_inst_ptr)
        self.pf_inst = pf_inst_ptr[0]
        

    def __del__ (self):
        "destroy particle filter"
        lib.destroy_particle_filter_instance(self.pf_inst)

    def get_particles(self):
        particles = ffi.new("struct particle*")
        particles_ptr = ffi.new("struct particle**", particles)
        length = lib.get_particle_array(self.pf_inst, particles_ptr)
        
        particles_list = ffi.unpack(particles, length)
        
        return [(p.x_pos, p.y_pos) for p in particles_list]
        
    def set_particles(self, particles):
        particle_array = ffi.new("struct particle[]", len(particles))
        for i in range(len(particles)):
            particle_array[i] = ffi.new("struct particle*", particles[i])[0]
        lib.set_particle_array(self.pf_inst, particle_array, len(particles))

    def predict(self, action):
        raise NotImplementedError

    def correct(self, measurement):
        if measurement.get_type() == "TWR":
            fp = ffi.new("struct particle*", measurement.get_measurement_particles())
            
            m = ffi.new("struct measurement*")
            m.measured_distance = measurement.get_measured_distance()
            m.foreign_particles = fp
            m.foreign_particles_length = len(measurement.get_measurement_particles())
            lib.correct(self.pf_inst, m)
        else:
            raise NotImplementedError

    def resample(self, weighted_particles):
        raise NotImplementedError
    
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    def value_from_normal_distribution(self, mean, std_dev, x):
        return lib.value_from_normal_distribution(lib.generate_normal_distribution(mean, std_dev, self.cache_normal_distribution), x)

