from cffi import FFI
ffibuilder = FFI()

# cdef() expects a single string declaring the C types, functions and
# globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""
struct particle {
  uint32_t x_pos;
  uint32_t y_pos;
};

struct normal_distribution {
  uint32_t mean;
  uint32_t std_dev;

  size_t buckets;
  uint32_t bucket_size;

  double *cached_distribution;
};

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      uint32_t x);
struct normal_distribution *generate_normal_distribution(
                                  size_t buckets, uint32_t mean,
                                  double std_dev);
""")

# set_source() gives the name of the python extension module to
# produce, and some C source code as a string.  This C code needs
# to make the declarated functions, types and globals available,
# so it is often just the "#include".
ffibuilder.set_source("_pf_cffi",
                      """
     #include "particle-filter.h"   // the C header of the library
""",
                      include_dirs=['./'],
                      library_dirs=['./build'],                      
                      libraries=['ParticleFilter'],
                      extra_link_args=['-Wl,-rpath,/Users/christian/Projects/vehicle-coordination/components/fast-particle-filter/pf/c_implementation/build']
                      )   # library name, for the linker

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
