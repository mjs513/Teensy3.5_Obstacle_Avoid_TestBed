#include <Wire.h>
#include <LSM303.h>
#include "Yamartino.h"

Yamartino yamartino(40);
LSM303 compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-3061,  -2357,  -2424};
  compass.m_max = (LSM303::vector<int16_t>){+2839,  +3278,  +3135};
}

void loop() {
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  */
  compass.read();
  float heading = compass.heading();
  yamartino.add(heading);
  
  Serial.print("Current Wind Direction: ");
  Serial.println(heading);
  Serial.print("Average Wind Direction: "); 
  Serial.println(yamartino.averageHeading());
  Serial.print("Standard Deviation: ");
  Serial.println(yamartino.standardDeviation());
  Serial.println("");
  
  delay(100);
}
