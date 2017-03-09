

#include <Servo.h>

Servo panServo;

// SERVO VARS
int angle = 0;
const int pulse0Degrees = 2100;   // pulse width for 0 degrees, was 1545 now 1360 for 120
const int pulse360Degrees = 990;  // pulse width for 360 degrees
                                   //was2100
const int pcenter = 2100; // using 1000 used to be 990 for center, or 0 degrees, you may want to use something different

int servoPanPin = 9;     // Control pin for servo motor

void setup() {
  Serial.begin(9600);         // connect to the serial port
  panServo.attach(servoPanPin);  

 // Default center, or close to it
 panServo.writeMicroseconds(pcenter);      // close to center GWS125-IT/2BB/F ?
}

void loop() {
    readSerialString();  // read, wait for command
}

void readSerialString () {
 //if ( Serial.available())
  //{
    //char ch = Serial.read();
    //if(ch >= '0' && ch <= '9')              // is ch a number?  
    //  angle = angle * 10 + ch - '0';           // yes, accumulate the value
    //else if(ch == 'P'| ch == 'p')  
    //{
    for(angle = 0; angle <= 240; angle += 5){
      int pulse = map(angle,0,360,pulse0Degrees, pulse360Degrees);
      Serial.println(pulse);
      panServo.writeMicroseconds(pulse);
      //angle = 0;
      delay(450);
    }
    angle = 0;
    delay(250);
    panServo.writeMicroseconds(pcenter);
    delay(500);
    
  //}
}



