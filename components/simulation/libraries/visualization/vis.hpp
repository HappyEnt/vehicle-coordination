#ifndef VIS_HPP
#define VIS_HPP

#include <boost/interprocess/ipc/message_queue.hpp>
#include <string>
#include <vector>

#include <vtkChartXY.h>
#include <vtkContextScene.h>
#include <vtkContextView.h>
#include <vtkFloatArray.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPlotPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTable.h>

#include <vtkCallbackCommand.h>


class ParticleScatterPlot : public vtkCallbackCommand
{
public:
  ParticleScatterPlot();
  virtual ~ParticleScatterPlot();
  void start_render();


private:
  std::vector<std::pair<std::string, std::shared_ptr<boost::interprocess::message_queue>>> node_particle_queues;
  std::shared_ptr<boost::interprocess::message_queue> register_node_interface;
  // make vector of pairs of Strings and vtkTable
  std::vector<std::pair<std::string, vtkSmartPointer<vtkTable>>> particle_tables;

  void register_queues();
  void register_new_node(std::string NodeName);
  void update_plot_data();

  void init();

  virtual void Execute(vtkObject *caller, unsigned long, void*);

  // vtk
  vtkNew<vtkContextView> view;
  vtkSmartPointer<vtkChartXY> chart;

  // vtk interactor
  int timerId;
};

#endif /* VIS_HPP */
