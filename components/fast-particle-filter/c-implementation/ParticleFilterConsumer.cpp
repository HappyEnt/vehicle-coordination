#include "ParticleFilterConsumer.hpp"
#include <iostream>

extern "C" double **calculate_likelihood(uint32_t measurement, struct particle *particles,
                                        struct particle *particles_other, size_t amount,
                                        size_t amount_other);

ParticleFilterConsumer::ParticleFilterConsumer() {
  this->message_queue = std::shared_ptr<boost::interprocess::message_queue>(
      new boost::interprocess::message_queue(boost::interprocess::open_only,
                                             "particle_queue"));
}

ParticleFilterConsumer::~ParticleFilterConsumer() {
  
}

int ParticleFilterConsumer::startConsumerLoop() {
    boost::interprocess::message_queue::size_type recvd_size;
    unsigned int priority;  
  do {
    struct particle particles[1000];
    
    this->message_queue->receive(particles, sizeof(particles), recvd_size, priority);

    if(recvd_size != sizeof(particles)) {
      std::cout << "received wrong size " << std::endl;
      return -1; // TODO proper error handling
    }
    
    free(calculate_likelihood(1000, particles, particles, 1000, 1000));

    // std::cout << "consumed new particle cloud" << std::endl;
  } while(true);
}

int main(int argc, char *argv[])
{
  ParticleFilterConsumer consumer;

  consumer.startConsumerLoop();
  return 0;
}
