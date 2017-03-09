/*
luckylarry.co.uk
Radar Screen Visualisation for Sharp GP2Y0A02 IR range finder
Sends sensor readings for every degree moved by the servo
values sent to serial port to be picked up by Processing
*/
#include <Servo.h>            // include the standard servo library
#include <NewPing.h>

Servo leftRightServo;         // set a variable to map the servo
int leftRightPos = 0;         // set a variable to store the servo position
const int numReadings = 8;   // set a variable for the number of readings to take
int index = 0;                // the index of the current reading
float total = 0;              // the total of all readings must be a float to allow totaling of float values
int average = 0;              // the average
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 500

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

/* setup the pins, servo and serial port */
void setup() { 
  leftRightServo.attach(9);
  // initialize the serial port:
  Serial.begin(57600);
} 
 
/* begin rotating the servo and getting sensor values */
void loop() { 
  for(leftRightPos = 0; leftRightPos < 180; leftRightPos++) {  // going left to right.                                
    leftRightServo.write(leftRightPos);             
      for (index = 0; index<=numReadings;index++) {            // take x number of readings from the sensor and average them
        int uS = sonar.ping_cm();
        total = total + uS;                              // update total
        delay(20);
      }
    average = (int) total/numReadings;                         // create average reading CAST TO INT!! remove the decimal places
 
    if (index >= numReadings)  {                               // reset the counts when at the last item of the array    
      index = 0;           
      total = 0;     
    }
    Serial.print("X");                                         // print leading X to mark the following value as degrees
    Serial.print(leftRightPos);                                // current servo position
    Serial.print("V");                                         // preceeding character to separate values
    Serial.println(average);                                   // average of sensor readings
  }
  /* 
  start going right to left after we got to 180 degrees 
  same code as above
  */
  for(leftRightPos = 180; leftRightPos > 0; leftRightPos--) {  // going right to left                                
    leftRightServo.write(leftRightPos);             
    for (index = 0; index<=numReadings;index++) {
        int uS = sonar.ping_cm();
        total = total + uS; 
        delay(20);
    }
    average = (int) total/numReadings;  
    if (index >= numReadings)  {           
      index = 0;           
      total = 0;     
    }
    Serial.print("X");
    Serial.print(leftRightPos);
    Serial.print("V");
    Serial.println(average);
   }  
}

