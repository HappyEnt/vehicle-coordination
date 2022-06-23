#include "MeasurementGenerator.hpp"

#include <chrono>
#include <thread>
#include <iostream>


MeasurementGenerator::MeasurementGenerator() {
  boost::interprocess::message_queue::remove("particle_queue");

  // this->message_period = long(1000000/initial_frequency);
  
  this->message_queue = std::shared_ptr<boost::interprocess::message_queue>(
      new boost::interprocess::message_queue(boost::interprocess::create_only,
                                             "particle_queue", 1000,
                                             sizeof(particle) * 1000));
}

MeasurementGenerator::~MeasurementGenerator() {
}

void MeasurementGenerator::send_particle_cloud() {
  struct particle particles[1000];

  this->message_queue->send(particles, sizeof(particles), 1);
}

void MeasurementGenerator::send_single() {
  send_particle_cloud();
}

void MeasurementGenerator::start_send_loop() {
  int averaging_msg_amount = 500;
  
  while (true) {
    auto start = std::chrono::steady_clock::now();
    for (auto i = 0; i < averaging_msg_amount; ++i) {
      send_single();
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << "Message Frequency averaged over "  << averaging_msg_amount << " messages: "
         << (long) averaging_msg_amount * 1e9 / std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
         << " Hz" << std::endl;

  }
}

