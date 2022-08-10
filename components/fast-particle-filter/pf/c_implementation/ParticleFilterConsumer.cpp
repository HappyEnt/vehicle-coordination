#include "ParticleFilterConsumer.hpp"
#include "util.h"
#include <iostream>

extern "C" {
  #include "particle-belief-propagation.h"
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

    struct message m;
    m.measured_distance = 2;
    m.particles = particles;
    m.particles_length = 1000;
    
    
    // free(calculate_likelihood(1000, particles, particles, 1000, 1000));

    add_message(this->pf_inst, m);
    iterate(this->pf_inst);

    // std::cout << "consumed new particle cloud" << std::endl;
  } while(true);
}

int main(int argc, char *argv[])
{
  ParticleFilterConsumer consumer;

  consumer.startConsumerLoop();
  return 0;
}
