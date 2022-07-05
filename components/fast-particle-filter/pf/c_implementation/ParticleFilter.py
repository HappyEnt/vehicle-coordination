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
        particles_ptr = ffi.new("struct particle**")
        length = lib.get_particle_array(self.pf_inst, particles_ptr)
        
        particles_list = ffi.unpack(particles_ptr[0], length)

        return [(p.x_pos, p.y_pos) for p in particles_list]
        
    def set_particles(self, particles):
        particle_arr = ffi.new("struct particle[]", len(particles))
        for i in range(len(particles)):
            particle_arr[i] = ffi.new("struct particle*", particles[i])[0]
        lib.set_particle_array(self.pf_inst, particle_arr, len(particles))

    def predict(self, action):
        raise NotImplementedError

    def correct(self, measurement):
        if measurement.get_type() == "TWR":
            self.foreign_particle_list = measurement.get_sender_particles()
            self.foreign_particle_arr = ffi.new("struct particle[]", len(self.foreign_particle_list))
            for i in range(len(self.foreign_particle_list)):
                self.foreign_particle_arr[i] = ffi.new("struct particle*", self.foreign_particle_list[i])[0]
            
            self.m = ffi.new("struct measurement*")
            self.m.measured_distance = measurement.get_measured_distance()
            self.m.foreign_particles = self.foreign_particle_arr
            self.m.foreign_particles_length = len(self.foreign_particle_arr)
            
            # TODO because we have ownership of the data we create through ffi we need to store all data created in the object
            # otherwise the data is freed
            lib.correct(self.pf_inst, self.m)
        else:
            raise NotImplementedError

    def resample(self, weighted_particles):
        raise NotImplementedError
    
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    def value_from_normal_distribution(self, mean, std_dev, x):
        return lib.value_from_normal_distribution(lib.generate_normal_distribution(mean, std_dev, self.cache_normal_distribution), x)

