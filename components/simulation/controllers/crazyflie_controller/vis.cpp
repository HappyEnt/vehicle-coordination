#include "vis.hpp"

#include <boost/log/trivial.hpp>

#include <cstdlib>
#include <memory>
#include <sstream>


extern "C" {
  #include <particle-belief-propagation.h>
  #include <util.h>
}

#define MQ_MAX_STRING_SIZE 300
#define MAX_PARTICLES 5000
#define PARTICLES 500


void ParticleScatterPlot::register_queues() {
  boost::interprocess::message_queue::remove("register_node_interface");

  this->register_node_interface = std::shared_ptr<boost::interprocess::message_queue>(
      new boost::interprocess::message_queue(boost::interprocess::create_only,
                                             "register_node_interface", 100,
                                             MQ_MAX_STRING_SIZE
                                             ));

  BOOST_LOG_TRIVIAL(info) << "register node interface";
}

void ParticleScatterPlot::register_new_node(std::string NodeName) {
  this->node_particle_queues.push_back(std::make_pair(
      NodeName,
      std::shared_ptr<boost::interprocess::message_queue>(
          new boost::interprocess::message_queue(
              boost::interprocess::open_only,
              std::string("particle_queue-").append(NodeName).c_str()))));

  BOOST_LOG_TRIVIAL(info) << "add plot for node " << NodeName;

  // initialize new table
  // vtkNew<vtkTable> table; //
  auto table = vtkSmartPointer<vtkTable>::New();

  auto arrX = vtkSmartPointer<vtkFloatArray>::New();
  arrX->SetName("X Coordinates");
  table->AddColumn(arrX.GetPointer());

  auto arrY = vtkSmartPointer<vtkFloatArray>::New();
  arrY->SetName("Y Coordinates");
  table->AddColumn(arrY.GetPointer());

  table->SetNumberOfRows(PARTICLES); // TODO SEND WITH REGISTER NEW NODE

  // initialize table to zeros
  for (int i = 0; i < PARTICLES; i++) {
    table->SetValue(i, 0, 0.0);
    table->SetValue(i, 1, 0.0);
  }

  vtkPlot* points = this->chart->AddPlot(vtkChart::POINTS);
  points->SetInputData(table, 0, 1);
  points->SetColor(0, 0, 0, 255);
  points->SetWidth(1.0);
  dynamic_cast<vtkPlotPoints*>(points)->SetMarkerStyle(vtkPlotPoints::CROSS);

  // create pair of NodeName and table
  this->particle_tables.push_back(std::make_pair(NodeName, table));

  BOOST_LOG_TRIVIAL(info) << "registered new node " << NodeName;
}

ParticleScatterPlot::ParticleScatterPlot() {
  this->register_queues();
  this->init();
}

ParticleScatterPlot::~ParticleScatterPlot() {

}

void ParticleScatterPlot::init() {
  vtkNew<vtkNamedColors> colors;

  // Set up a 2D scene, add an XY chart to it
  view->GetRenderWindow()->SetSize(400, 300);
  view->GetRenderWindow()->SetWindowName("Particles");
  view->GetRenderer()->SetBackground(colors->GetColor3d("White").GetData());

  chart = vtkSmartPointer<vtkChartXY>::New();
  view->GetScene()->AddItem(chart);
  chart->SetShowLegend(true);
}

void ParticleScatterPlot::Execute(vtkObject *caller, unsigned long eventId, void* vtkNotUsed(callData)) {
    if (vtkCommand::TimerEvent == eventId)
    {
      update_plot_data();
    }
}

void ParticleScatterPlot::start_render() {

  view->GetRenderWindow()->SetMultiSamples(0);
  view->GetRenderWindow()->Render();
  view->GetInteractor()->Initialize();

  // add observer to renderWindowInteractor
  view->GetInteractor()->AddObserver(vtkCommand::TimerEvent, this);

  // create timer
  timerId = view->GetInteractor()->CreateRepeatingTimer(100);

  view->GetInteractor()->Start();
}

void ParticleScatterPlot::update_plot_data() {
  // remove all plots
  size_t recvd_size;
  unsigned int priority;

  // Check for new Nodes
  std::string NodeName;

  NodeName.resize(MQ_MAX_STRING_SIZE);

  if(this->register_node_interface->try_receive(&NodeName[0], MQ_MAX_STRING_SIZE, recvd_size, priority)) {
    NodeName.resize(recvd_size);

    register_new_node(NodeName);
  }

  for (auto it : this->node_particle_queues) {
    struct particle particle_data[sizeof(struct particle) * MAX_PARTICLES]; // TODO MAX_PARTICLES PASS THROUGH REGISTER INTERFACE TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO TODO

    if(! it.second->try_receive(particle_data, sizeof(struct particle) * MAX_PARTICLES, recvd_size, priority)) {
      continue;
    }

    size_t amount = recvd_size / sizeof(struct particle);

    // create data for vtk
    for(auto ta : this->particle_tables) {
      if( !ta.first.compare(it.first) ) {
        BOOST_LOG_TRIVIAL(info) << "add " << amount << " particles to table for node " << it.first;
        for(size_t p = 0; p < amount; p++) {
          ta.second->SetValue(p, 0, particle_data[p].x_pos);
          ta.second->SetValue(p, 1, particle_data[p].y_pos);
        }
        // set table modified
        ta.second->Modified();
      }

    }
  }

  this->view->GetRenderWindow()->Render();
}

int main(int argc, char *argv[])
{

  ParticleScatterPlot vis;
  vis.start_render();

  return EXIT_SUCCESS;
}
