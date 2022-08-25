#include <stdio.h>
#include <stdlib.h>

#include <util.h>
#include <time.h>
#include <unistd.h>

#include <particle-belief-propagation.h>

#define GNUPLOT "gnuplot -persist"
/* #define GNUPLOT "cat"	// to test output */

FILE * temp;
FILE * gnuplotPipe;

/* void vis(struct particle_filter_instance *pf_inst) */
void vis(struct particle_filter_instance *pf_inst)
{
  struct particle *particles = pf_inst->local_particles;
  size_t amount = pf_inst->local_particles_length;

  if (ftruncate(fileno(temp), 0) == -1) {
    printf("Could not truncate\n");
  }
  fflush(temp);

  for (unsigned int i = 0; i < amount; ++i) {
    double x = particles[i].x_pos;
    double y = particles[i].y_pos;
    fprintf(temp, "%lf %lf \n", x, y); //Write the data to a temporary file
  }

  fflush(temp);

  /* char * commandsForGnuplot[] = {"replot", "set xrange [-20:20]", "set yrange [-20:20]"}; */
  char * commandsForGnuplot[] = {"replot"};
  /* char * commandsForGnuplot[] = {}; */
  int commands = sizeof(commandsForGnuplot) / sizeof(char*);
  /* int commands = 0; */

  for (size_t i=0; i < commands; i++)
    {
      fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
    }

  fflush(gnuplotPipe);
}

void create_vis() {
  char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"", "set xrange [-5:5]", "set yrange [-5:5]", "set term qt noraise", "plot 'data.temp'"};
  /* char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"",  "set xrange [-20:20]", "set yrange [-20:20]", "plot 'data.temp'", "pause 1", "reread"}; */
  int commands = sizeof(commandsForGnuplot) / sizeof(char*);
  /* char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"", "plot 'data.temp'"}; */

  gnuplotPipe = popen ("gnuplot -persistent", "w");
  temp = fopen("data.temp","w");

  fprintf(temp, "%lf %lf \n", 0.0, 0.0); //Write the data to a temporary file
  fflush(temp);

  printf("sending %d commands \n", commands);


  for (size_t i=0; i < commands; i++)
    {
      fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
    }

  fflush(gnuplotPipe);
}
