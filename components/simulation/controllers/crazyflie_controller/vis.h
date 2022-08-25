#ifndef VIS_H
#define VIS_H

#include <particle-belief-propagation.h>
#include <stdio.h>
#include <stdlib.h>

struct vis_instance {
  char name[80];
  FILE *data;
  FILE * gnuplotPipe;
};

void visualize(struct vis_instance *vis, struct particle_filter_instance *pf_inst);
void create_vis(struct vis_instance *vis, const char *name);

#endif /* VIS_H */



