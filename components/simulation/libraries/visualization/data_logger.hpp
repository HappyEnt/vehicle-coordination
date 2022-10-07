#ifndef VIS_H
#define VIS_H

#include <boost/interprocess/ipc/message_queue.hpp>
#include <cstdlib> //std::system
#include <memory>
#include <sstream>

#include <fstream>

extern "C" {
#include <particle-belief-propagation.h>
#include "util.h"
}


class DataLogger {
private:
  // int message_period;
  std::shared_ptr<boost::interprocess::message_queue> message_queue;
  std::string NodeName;


  void create_particle_mq(size_t max_particles_per_message);

public:
  DataLogger(std::string NodeName);
  DataLogger(std::string NodeName, size_t max_particles_per_message);

  void write_particles(struct particle *particles, size_t amount);
  void register_node_with_vis();

  virtual ~DataLogger();
};


#endif /* VIS_H */



