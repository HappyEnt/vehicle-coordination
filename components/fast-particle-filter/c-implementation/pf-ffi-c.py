from _pf_cffi import ffi, lib

print(lib.value_from_normal_distribution(lib.generate_normal_distribution(50, 0, 1000), 10));
print(dir(lib))

# prints: 23
# print("{}".format())
