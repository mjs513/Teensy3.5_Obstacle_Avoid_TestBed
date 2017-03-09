#include <histogram_grid.h>
#include <polar_histogram.h>
#include <vfh.h>

/*
** no-velocity.c
**
** Author: Carlos Agarie Junior
**
** This is an example file used to show how to setup basic usage of the Virtual
** Field Histogram.
*/

#include <stdio.h>
#include <stdlib.h>
#include "Streaming.h"
#include <Wire.h>

#define MEASUREMENTS 2000
#define OBJECTIVE_DIRECTION 90 /* [degrees] */ 

void setup() {
  
          Serial.begin(115200);
          Wire.begin();
          
	int i, position_x, position_y;
        randomSeed(analogRead(1));
	Serial << " setting up grid" << endl;

	/* Declaration of the needed data structures. */
	grid_t * certainty_grid;
        range_measure_t measure[MEASUREMENTS];
	hist_t * polar_histogram;
	control_signal_t control_signal;

	/** Initialization of the grid and the histogram. */
	certainty_grid = grid_init(50, 10);
	polar_histogram = hist_init(2, 20, 10, 5);

	/* Are the initializations ok? */
	if (certainty_grid == NULL) Serial << "certainty_grid = -1" << endl;
	if (certainty_grid->cells ==  NULL) Serial << "certainty_grid->cells = -1" << endl;
	if (polar_histogram == NULL) Serial << "polar_histogram = -1" << endl;
	if (polar_histogram->densities == NULL) Serial << "polar_histogram->densities = -1" << endl;
  
  	Serial << " Finished setting up grid" << endl;
  
	Serial << "Measures\n";
		
	for (i = 0; i < MEASUREMENTS; ++i) {
		measure[i].direction = (int) (random(100)*360./100); /* [degrees] */
		measure[i].distance = (int) (random(100)*130./100); /* [cm] */
		
		if (i < 50)
			Serial << i << ",  " << (int) measure[i].distance << ",  " <<
				measure[i].direction << endl;
	}
	
	if (i > 50) Serial << endl;

	/* Let's assume the 'robot' is in the middle of the grid. */
	position_x = position_y = (certainty_grid->dimension + 1) / 2;
	//position_x = 10;
        //position_y =  30;

	Serial << "\nPosition of the robot: (%d, %d)\n" << position_x << ",  " << position_y <<endl;

	/* Add the information from the measures to the grid. */
	Serial << "\nUpdating the certainty grid...\n" << endl;
	for (i = 0; i < MEASUREMENTS; ++i) {
		grid_update(certainty_grid, position_x, position_y, measure[i]);
	}

	mprint(certainty_grid);

	/*
	** Calculating the control signals.
	*/
	
	/* Generate the histogram for the current grid. */
	Serial << "\nUpdating the polar histogram...\n" << endl;
	hist_update(polar_histogram, certainty_grid);

	/* What's the next direction? */
	control_signal.direction = calculate_direction(polar_histogram, OBJECTIVE_DIRECTION);
	
	Serial << "\nNext direction: %d [degrees]\n" << control_signal.direction << endl;

	free(certainty_grid);
	free(polar_histogram);
	
	certainty_grid = NULL;
	polar_histogram = NULL;

	Serial.println("0"); 


}

void loop() {
  
  
}

void mprint(grid_t * grid) {
  int dim = grid->dimension;

  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      if (grid->cells[i * dim + j] >= 8) {
        Serial.print("o");
      } else if (grid->cells[i * dim + j] >= 4) {
        Serial.print("x");
      } else {
        Serial.print("-");
      }
    }
    Serial.println("ddddd");
  }
}

