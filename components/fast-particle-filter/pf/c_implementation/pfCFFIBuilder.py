from cffi import FFI
ffibuilder = FFI()

# cdef() expects a single string declaring the C types, functions and
# globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""

struct particle {
  double x_pos; // [x_pos] = m
  double y_pos; // [y_pos] = m
};

struct weighted_particle {
  struct particle particle;
  double weight;
};

enum actions {
  CONTROL_ACTION, // for mobility model p(x_t | x_t-1)
  TDOA_MEASUREMENT, // Time difference of Arrival -> 3 Sets of particles
  TWR_MEASUREMENT,  // Two Way Ranging -> 2 Sets of particles
  SELF_MEASUREMENT, // z_self for example pedometer measurement
};

struct message {
  double measured_distance;
  struct particle *particles;
  size_t particles_length;
};

struct message_stack {
  struct message item;
  struct message_stack *next;
};

struct particle_filter_instance {
  // the minimum we have to store is the particle set
  struct particle *local_particles;
  size_t local_particles_length;
  // Any other data that is usefull
  struct normal_distribution *uwb_error_likelihood;

  struct message_stack *mstack;
};


// _____Public Interface_____

// Use create and destroy to create instance, do not free memory yourself.
void create_particle_filter_instance(struct particle_filter_instance **pf_inst);
void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst);

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length);
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles);

void add_message(struct particle_filter_instance *pf_inst, struct message m);

// TODO find out what kind of actions will exist
void predict(struct particle_filter_instance *pf_inst, double moved_distance);
void iterate(struct particle_filter_instance *pf_inst);

struct normal_distribution {
  double mean;
  double std_dev;

  size_t buckets;
  double bucket_size;

  double *cached_distribution;
};

double distance1(double x, double y);
double distance2(struct particle p1, struct particle p2);

void sample_particles_from_gaussian(struct particle mean , double variance, struct particle *ps, size_t amount);

struct normal_distribution *generate_normal_distribution(
                                                         double mean,
                                                         double std_dev,
                                                         bool cache_histogram);

void destroy_normal_distribution(struct normal_distribution *distribution);

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      double x);


""")

# set_source() gives the name of the python extension module to
# produce, and some C source code as a string.  This C code needs
# to make the declarated functions, types and globals available,
# so it is often just the "#include".
ffibuilder.set_source("_pf_cffi",
                      """
                      #include "particle-belief-propagation.h"   // the C header of the library
                      #include "util.h"   // the C header of the library
""",
                      include_dirs=['./'],
                      library_dirs=['./build'],
                      libraries=['BeliefPropagation'],
                      extra_link_args=['-Wl,-rpath,/Users/christian/Projects/vehicle-coordination/components/fast-particle-filter/pf/c_implementation/build']
                      )   # library name, for the linker

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
