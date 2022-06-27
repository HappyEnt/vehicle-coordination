#include "MeasurementGenerator.hpp"
#include <chrono>
#include <thread>

int main(int argc, char *argv[]) {

  MeasurementGenerator msg_generator;

  msg_generator.start_send_loop();
  
  return 0;
}
