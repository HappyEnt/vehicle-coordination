from cffi import FFI
import platform

ffibuilder = FFI()

# cdef() expects a single string declaring the C types, functions and
# globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""

struct particle {
  double x_pos; // [x_pos] = m
  double y_pos; // [y_pos] = m
  double weight;
};

struct message {
  double measured_distance;

  struct particle *particles;
  size_t particles_length;

  enum {
    DENSITY_ESTIMATION, // determines how we sample from the message
    DUMB_PARTICLES,
  } type;

  double h_opt; // only used in case of non parametric belief propagation
  double variance;
};

struct particle_filter_instance;

// _____Public Interface_____

// Use create and destroy to create instance, do not free memory yourself.
void create_particle_filter_instance(struct particle_filter_instance **pf_inst);
void destroy_particle_filter_instance(struct particle_filter_instance *pf_inst);

void set_particle_array(struct particle_filter_instance *pf_inst, struct particle *particles, size_t length);
void set_particle_amount(struct particle_filter_instance *pf_inst, size_t amount);
int get_particle_array(struct particle_filter_instance *pf_inst, struct particle **particles);

void add_belief(struct particle_filter_instance *pf_inst, struct message m);

// TODO find out what kind of actions will exist
void predict_dist(struct particle_filter_instance *pf_inst, double moved_distance);
void predict_dist_2D(struct particle_filter_instance *pf_inst, double moved_x, double moved_y);
void iterate(struct particle_filter_instance *pf_inst);
""")

# set_source() gives the name of the python extension module to
# produce, and some C source code as a string.  This C code needs
# to make the declarated functions, types and globals available,
# so it is often just the "#include".


os = platform.system()
print("OS in my system : ",os)

loader_path_prefix = "@loader_path" if "Darwin" in os else "$ORIGIN"

ffibuilder.set_source("_pf_cffi",
                      """
                      #include "particle-belief-propagation.h"   // the C header of the library
                      #include "util.h"   // the C header of the library
""",
                      include_dirs=['./'],
                      library_dirs=['./build'],
                      libraries=['BeliefPropagation'],
                      extra_link_args=['-Wl,-rpath,' + loader_path_prefix + '/build']
                      # ORIGIN does not seem to work here. Maybe not supported on MacOS?
                      )   # library name, for the linker

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
