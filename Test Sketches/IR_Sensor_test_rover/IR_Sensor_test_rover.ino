#include <Wire.h>

//IR Sensor Pins
const int leftIRsensor = A0;
const int rightIRsensor = A2;

int frtIRdistance, rearIRdistance;
int leftCounter, rightCounter;

void setup() {
  // put your setup code here, to run once:
    Serial3.begin(9600);
    Wire.begin();

    //delay(5000);

}

void loop() {
  // put your main code here, to run repeatedly:

    
    frtIRdistance = frtIRaverage(50);
    rearIRdistance = rearIRaverage(50);
  
    Serial3.print(frtIRdistance); Serial3.print(",  ");
    Serial3.print(rearIRdistance); Serial3.println();

    //delay(500);
}

  
int frtIRaverage(int average_count) {
  int sum = 0;
  for (int i=0; i<average_count; i++) {
    int sensor_value = analogRead(leftIRsensor);  //read the sensor value
    int distance_cm = pow(3027.4/sensor_value, 1.2134); //convert readings to distance(cm)
    sum = sum + sensor_value;
    delay(5);
    }
  return(sum/average_count);  
}
  
int rearIRaverage(int average_count) {
  int sum = 0;
  for (int i=0; i<average_count; i++) {
    int sensor_value = analogRead(rightIRsensor);  //read the sensor value
    int distance_cm = pow(3027.4/sensor_value, 1.2134); //convert readings to distance(cm)
    sum = sum + sensor_value;
    delay(5);
  }
  return(sum/average_count);  
}

