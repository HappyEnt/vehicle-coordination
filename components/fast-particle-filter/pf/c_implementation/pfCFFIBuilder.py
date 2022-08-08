from cffi import FFI
ffibuilder = FFI()

# cdef() expects a single string declaring the C types, functions and
# globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""
struct particle {
  double x_pos;
  double y_pos;
};

struct weighted_particle {
  struct particle particle; 
  double weight;
};

typedef double measurement_t;
typedef double action_t[2];

struct particle_filter_instance {
  // the minimum we have to store is the particle set 
  struct particle *local_particles;
  size_t local_particles_length;
  // Any other data that is usefull
  struct normal_distribution *uwb_error_likelihood;
};

struct normal_distribution {
  double mean;
  double std_dev;

  size_t buckets;
  double bucket_size;

  double *cached_distribution;
};

struct message {
  double measured_distance;
  struct particle *particles;
  size_t particles_length;
};


void create_particle_filter_instance(struct particle_filter_instance **pf_inst);
void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst);

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length);
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles);

// TODO find out what kind of actions will exist 
void predict(struct particle_filter_instance *pf_inst, int action);
void correct(struct particle_filter_instance *pf_inst);

void add_message(struct particle_filter_instance *pf_inst, struct message m);

// Test Interface
struct normal_distribution *generate_normal_distribution(
                                  double mean,
                                  double std_dev,
                                  bool cache_histogram);

void destroy_normal_distribution(struct normal_distribution *distribution);

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      double x);


void calculate_belief(struct particle_filter_instance *pf, struct weighted_particle *weighted_particles);

void resample(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length);
""")

# set_source() gives the name of the python extension module to
# produce, and some C source code as a string.  This C code needs
# to make the declarated functions, types and globals available,
# so it is often just the "#include".
ffibuilder.set_source("_pf_cffi",
                      """
     #include "importance-sampling-particle-filter.h"   // the C header of the library
""",
                      include_dirs=['./'],
                      library_dirs=['./build'],                      
                      libraries=['ParticleFilter'],
                      extra_link_args=['-Wl,-rpath,/Users/christian/Projects/vehicle-coordination/components/fast-particle-filter/pf/c_implementation/build']
                      )   # library name, for the linker

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
