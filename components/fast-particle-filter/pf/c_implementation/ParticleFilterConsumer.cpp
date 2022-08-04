#include "ParticleFilterConsumer.hpp"
#include <iostream>

// extern "C" void correct(struct particle_filter_instance *pf_inst, struct measurement *m);
// extern "C" void create_particle_filter_instance(struct particle_filter_instance **pf_inst);

extern "C" {
  #include "floating-point-particle-filter.h"
}

ParticleFilterConsumer::ParticleFilterConsumer() {
  this->message_queue = std::shared_ptr<boost::interprocess::message_queue>(
      new boost::interprocess::message_queue(boost::interprocess::open_only,
                                             "particle_queue"));

  create_particle_filter_instance(&this->pf_inst);
}

ParticleFilterConsumer::~ParticleFilterConsumer() {
  destroy_particle_filter_instance(this->pf_inst);
}

int ParticleFilterConsumer::startConsumerLoop() {
    boost::interprocess::message_queue::size_type recvd_size;
    unsigned int priority;
    static int initial_message = 1;
  do {
    struct particle particles[1000];

    if (initial_message) {
      set_particle_array(this->pf_inst, particles, 1000);
      initial_message = 0;
    }
    
    this->message_queue->receive(particles, sizeof(particles), recvd_size, priority);

    if(recvd_size != sizeof(particles)) {
      std::cout << "received wrong size " << recvd_size << " vs " << sizeof(particles) << std::endl;
      return -1; // TODO proper error handling
    }

    struct measurement m;
    m.measured_distance = 2;
    m.foreign_particles = particles;
    m.foreign_particles_length = 1000;
    
    
    // free(calculate_likelihood(1000, particles, particles, 1000, 1000));
    correct(this->pf_inst, &m);

    // std::cout << "consumed new particle cloud" << std::endl;
  } while(true);
}

int main(int argc, char *argv[])
{
  ParticleFilterConsumer consumer;

  consumer.startConsumerLoop();
  return 0;
}
