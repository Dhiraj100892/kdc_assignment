/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"

/*****************************************************************************/
/*****************************************************************************/

extern SIM sim;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* Call this once to do one time operations like memory allocation */

struct data {
  // 2000 frames because the trajectory was simulated for 20s from start
  //COM of Alien
  float COM_x[2000];
  float COM_y[2000];
  float COM_z[2000];

  //quaternion of alien
  float q_scalar[2000];
  float q_x[2000];
  float q_y[2000];
  float q_z[2000];
};

struct data read_data_from_file(char* file){
  struct data my_data;
  FILE *fp;
  char* tmp=NULL;
  size_t len = 0;
  fp = fopen(file, "r");
  int i=0;
  ssize_t nread;
  
  if (fp!=NULL){
    while((nread = getline(&tmp,&len,fp))!=-1){
      // printf("i:%d, %s\n",i,tmp);
      // sscanf(tmp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &my_data.COM_x[i], &my_data.COM_y[i], &my_data.COM_z[i], &my_data.q_scalar[i], &my_data.q_x[i], &my_data.q_y[i], &my_data.q_z[i], &my_data.w_x[i], &my_data.w_y[i], &my_data.w_z[i]);
      // printf("%f,%f\n", my_data.COM_x[i], my_data.COM_y[i]);
      sscanf(tmp, "%f,%f,%f,%f,%f,%f,%f", &my_data.COM_x[i], &my_data.COM_y[i], &my_data.COM_z[i], &my_data.q_scalar[i], &my_data.q_x[i], &my_data.q_y[i], &my_data.q_z[i]);
      i += 1;
    }
    printf("read %d lines\n",i);
  }
  else{
    printf("can't open file");
  }
  fclose(fp);
  // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &my_data.COM_x[i], &my_data.COM_y[i], &my_data.COM_z[i], &my_data.q_scalar[i], &my_data.q_x[i], &my_data.q_y[i], &my_data.q_z[i], &my_data.w_x[i], &my_data.w_y[i], &my_data.w_z[i]);
  return my_data;
}

int init_sim( SIM *s )
{

  init_dynamics( s );
  init_controller( s );
  init_data( s );

  reinit_sim( s );
}

/*****************************************************************************/
/* call this many times to restart a simulation */

int reinit_sim( SIM *s )
{

  srand( s->rand_seed );

  reinit_dynamics( s );
  reinit_controller( s );
}

/*****************************************************************************/

main( int argc, char **argv )
{
  // reading the alien trajectory data
  char *filename = "problem_3.dat";
  printf("filename:%s\n",filename);

  // struct data alien_sim_data;
  
  // read_data_from_file(alien_sim_data, filename);
  struct data alien_sim_data = read_data_from_file(filename);

  PARAMETER *params;
  int n_parameters;
  int i;

  init_default_parameters( &sim );
  sim.rand_scale = 0;
  sim.controller_print = 1;

  /* Parameter file argument? */
  if ( argc > 1 )
    {
      params = read_parameter_file( argv[1] );
      n_parameters = process_parameters( params, &sim, 1 );
      if ( n_parameters > MAX_N_PARAMETERS )
	{
	  fprintf( stderr, "Too many parameters %d > %d\n",
		   n_parameters, MAX_N_PARAMETERS );
	  exit( -1 );
	}
    }

  init_sim( &sim );

  for( i = 0; sim.time < sim.duration; i++ )
    {
      if ( (i % 1000) == 0 )
        {
          
        }
        // printf("%f,%f\n", alien_sim_data.COM_x[i], alien_sim_data.COM_y[i]);
        sim.lander_x_d[0] = alien_sim_data.COM_x[i]; 
        sim.lander_x_d[1] = alien_sim_data.COM_y[i];
        sim.lander_x_d[2] = alien_sim_data.COM_z[i];
        sim.lander_q_d[Q0] = alien_sim_data.q_scalar[i];
        sim.lander_q_d[Q1] = alien_sim_data.q_x[i];
        sim.lander_q_d[Q2] = alien_sim_data.q_y[i];
        sim.lander_q_d[Q3] = alien_sim_data.q_z[i];
      controller( &sim );
      save_data( &sim );
      if ( sim.status == CRASHED )
	break;
      integrate_one_time_step( &sim );
    }

  write_the_mrdplot_file( &sim );
}

/*****************************************************************************/
