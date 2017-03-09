/*
** no-velocity.c
**
** Author: Carlos Agarie Junior
**
** This is an example file used to show how to setup basic usage of the Virtual
** Field Histogram.
*/

#include <Arduino.h>
#include <vfh.h>

#include <Servo.h>            // include the standard servo library
#include <NewPing.h>

// DC hobby servo
Servo servo1;

/* The servo minimum and maximum angle rotation */
static const int minAngle = 0;
static const int maxAngle = 179;
int servoAngle;
int servoPos;
int servoPin = 9;
#define MAX_DISTANCE 500

/* Define pins for HC-SR04 ultrasonic sensor */
#define echoPin 11 // Echo Pin = Analog Pin 0
#define trigPin 12 // Trigger Pin = Analog Pin 1
#define LEDPin 13 // Onboard LED
long duration; // Duration used to calculate distance
long HR_dist=0; // Calculated Distance
int HR_angle=0; // The angle in which the servo/sensor is pointing
int HR_dir=1; // Used to change the direction of the servo/sensor
int minimumRange=0; //Minimum Sonar range
int maximumRange=500; //Maximum Sonar Range

NewPing sonar(trigPin, echoPin, maximumRange);


#define MEASUREMENTS 180
#define OBJECTIVE_DIRECTION 90 /* [degrees] */
#define RAND_MAX 100

int i, position_x, position_y;

/* Declaration of the needed data structures. */
grid_t * certainty_grid;
range_measure_t measure[MEASUREMENTS];
hist_t * polar_histogram;
control_signal_t control_signal;

void setup() {
        Serial.begin(9600);
        
       // Tell the arduino that the servo is attached to Digital pin 10.
       servo1.attach(servoPin);        
        
	/*
	** Initialization of the grid and the histogram.
	*/
	certainty_grid = grid_init(50, 10);
	polar_histogram = hist_init(2, 20, 10, 5);
	
	/* Are the initializations ok? */
	if (certainty_grid == NULL) Serial.println("-1");
	if (certainty_grid->cells == NULL) Serial.println("-1");
	if (polar_histogram == NULL) Serial.println("-1");
	if (polar_histogram->densities == NULL) Serial.println("-1");
        delay(2000);
        
        servo1.write(0);
        delay(50);
        
  for(int pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servo1.write(pos);              // tell servo to go to position in variable 'pos' 
                          // waits 15ms for the servo to reach the position 
    measure[pos].direction = pos; /* [degrees] */
    measure[pos].distance = sonar.ping_cm();
    Serial.print(measure[pos].direction); Serial.print(" -------  "); Serial.println(measure[pos].distance);
    delay(35);

  } 
//  for(int pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
//  {                                
//    servo1.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//    measure[pos].direction = pos; /* [degrees] */
//    measure[pos].distance = sonar.ping_cm();
//    delay(50);
//  }
 	/* Let's assume the 'robot' is in the middle of the grid. */
	position_x = position_y = (certainty_grid->dimension + 1) / 2;

	printf("\nPosition of the robot: (%d, %d)\n", position_x, position_y);

	/* Add the information from the measures to the grid. */
	printf("\nUpdating the certainty grid...\n");
	for (i = 0; i < MEASUREMENTS; ++i) {
		grid_update(certainty_grid, position_x, position_y, measure[i]);
	}
		
	/*
	** Calculating the control signals.
	*/
	
	/* Generate the histogram for the current grid. */
	printf("\nUpdating the polar histogram...\n");
	hist_update(polar_histogram, certainty_grid);
	
	/* What's the next direction? */
	control_signal.direction = calculate_direction(polar_histogram, OBJECTIVE_DIRECTION);
	
	printf("\nNext direction: %d [degrees]\n", control_signal.direction);

	free(certainty_grid);
	free(polar_histogram);
	
	certainty_grid = NULL;
	polar_histogram = NULL;
 
}        
        
void loop() {

}
