#include <boost/interprocess/ipc/message_queue.hpp>
#include <cstdlib> //std::system
#include <memory>
#include <sstream>

struct particle {
public:
  uint32_t x_pos;
  uint32_t y_pos;
};


class MeasurementGenerator {
private:
  // int message_period;
  std::shared_ptr<boost::interprocess::message_queue> message_queue;
  void send_particle_cloud();

public:
  MeasurementGenerator();

  void send_single();
  void start_send_loop();  

  virtual ~MeasurementGenerator();
};
