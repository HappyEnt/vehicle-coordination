#include <boost/interprocess/ipc/message_queue.hpp>
#include <cstdlib> //std::system
#include <memory>
#include <sstream>

// struct particle {
// public:
//   uint32_t x_pos;
//   uint32_t y_pos;
// };

class ParticleFilterConsumer
{
private:
  std::shared_ptr<boost::interprocess::message_queue> message_queue;
public:
  int startConsumerLoop();
  ParticleFilterConsumer();
  virtual ~ParticleFilterConsumer();

  struct particle_filter_instance *pf_inst;
};
