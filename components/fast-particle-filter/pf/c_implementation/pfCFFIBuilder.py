from cffi import FFI
ffibuilder = FFI()

# cdef() expects a single string declaring the C types, functions and
# globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""
struct particle {
  double x_pos;
  double y_pos;
};

struct normal_distribution {
  double mean;
  double std_dev;

  size_t buckets;
  double bucket_size;

  double *cached_distribution;
};

struct normal_distribution *generate_normal_distribution(
                                  double mean,
                                  double std_dev,
                                  bool cache_histogram);

double value_from_normal_distribution(struct normal_distribution *distribution,
                                      double x);
""")

# set_source() gives the name of the python extension module to
# produce, and some C source code as a string.  This C code needs
# to make the declarated functions, types and globals available,
# so it is often just the "#include".
ffibuilder.set_source("_pf_cffi",
                      """
     #include "floating-point-particle-filter.h"   // the C header of the library
""",
                      include_dirs=['./'],
                      library_dirs=['./build'],                      
                      libraries=['ParticleFilter'],
                      extra_link_args=['-Wl,-rpath,/Users/christian/Projects/vehicle-coordination/components/fast-particle-filter/pf/c_implementation/build']
                      )   # library name, for the linker

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
