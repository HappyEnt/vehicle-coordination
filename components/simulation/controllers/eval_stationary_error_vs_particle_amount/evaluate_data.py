# import numpy,matplotlib etc.
import numpy as np
import matplotlib.pyplot as plt


# load file error-robot(3).csv
data = np.loadtxt('error-robot(3).csv', delimiter=',', skiprows=1)

# extract first column to particles, second column to filter_type and last column to error
particles = data[:,0]
filter_type = data[:,1]
error = data[:,2]

# extract for each pair of particles and filter type the corresponding error values and store in dictionary
error_dict = {}
for i in range(len(particles)):
    if (particles[i], filter_type[i]) in error_dict:
        error_dict[(particles[i], filter_type[i])].append(error[i])
    else:
        error_dict[(particles[i], filter_type[i])] = [error[i]]

# get only values from dictionary and store in list
error_list = list(error_dict.values())
config_names = list(error_dict.keys())

print(error_list)

# print(particles[filter_type == 0])
# print(error[filter_type == 0])

# plot boxplot for each tuple of particles and filter_type against the error
plt.boxplot(error_list)

# add config_names to each boxplot
plt.xticks(range(1, len(config_names)+1), config_names)


# plot for filter_type = 0, the error against the particles
# plt.plot(particles[filter_type == 0], error[filter_type == 0], 'r', label='EKF')
plt.show()
