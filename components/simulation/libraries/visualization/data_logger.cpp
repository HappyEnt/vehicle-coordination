#include "data_logger.hpp"

#define MAX_QUEUE_MESSAGES 100

// initialize DataLogger by setting NodeName
DataLogger::DataLogger(std::string NodeName, size_t max_particles_per_message) : NodeName(NodeName) {
  create_particle_mq(max_particles_per_message);

  // create basic_ofstream for error_stats_file
  error_stats_file = std::unique_ptr<std::ofstream>(new std::ofstream());
  error_stats_file->open(std::string("error-").append(NodeName).append(".csv"), std::ios::out);
}

DataLogger::DataLogger(std::string NodeName) : NodeName(NodeName) {
  create_particle_mq(3000);
}


DataLogger::~DataLogger() {
  error_stats_file->close();
}

void DataLogger::create_particle_mq(size_t max_particles_per_message) {
  boost::interprocess::message_queue::remove(std::string("particle_queue-").append(NodeName).c_str());

  this->message_queue = std::shared_ptr<boost::interprocess::message_queue>(
      new boost::interprocess::message_queue(boost::interprocess::create_only,
                                             std::string("particle_queue-").append(NodeName).c_str(), MAX_QUEUE_MESSAGES,
                                             sizeof(struct particle) * max_particles_per_message));
}

void DataLogger::write_particles(struct particle *particles, size_t amount) {
  this->message_queue->send(particles, sizeof(struct particle) * amount, 1);
}

void DataLogger::write_error_to_csv(struct particle estimated_location, struct particle ground_truth) {
  double error = sqrt(pow(estimated_location.x_pos - ground_truth.x_pos, 2) + pow(estimated_location.y_pos - ground_truth.y_pos, 2));
  *error_stats_file << error << std::endl;
}

void DataLogger::register_node_with_vis() {
  // first open message queue
  boost::interprocess::message_queue register_node_interface(boost::interprocess::open_only,
                                             "register_node_interface");

  register_node_interface.send(this->NodeName.data(), this->NodeName.size(), 0);
}
