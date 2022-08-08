import sys, os
from ._pf_cffi import ffi, lib

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from AbstractParticleFilter import AbstractParticleFilter
from AbstractParticleFilter import AbstractParticleFilter

class CParticleFilter(AbstractParticleFilter):
    cache_normal_distribution = False
    message_list = []
    
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


    def add_message(self, message):
        if message.get_type() == "TWR":
            foreign_particle_list = message.get_sender_particles()
            foreign_particle_arr = ffi.new("struct particle[]", len(foreign_particle_list))
            for i in range(len(foreign_particle_list)):
                foreign_particle_arr[i] = ffi.new("struct particle*", foreign_particle_list[i])[0]
                
            m = ffi.new("struct message*")
            m.measured_distance = message.get_measured_distance()
            m.particles = foreign_particle_arr
            m.particles_length = len(foreign_particle_arr)

            lib.add_message(self.pf_inst, m[0])

            # we have ownership, prevent memory from being freed by storing in instance variable
            self.message_list.append((m, foreign_particle_arr))
        else:
            raise NotImplementedError
        
    def predict(self, action):
        raise NotImplementedError

    def correct(self):
            lib.correct(self.pf_inst)
            self.message_list.clear()


    def resample(self, weighted_particles):
        raise NotImplementedError
    
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    def value_from_normal_distribution(self, mean, std_dev, x):
        return lib.value_from_normal_distribution(lib.generate_normal_distribution(mean, std_dev, self.cache_normal_distribution), x)

