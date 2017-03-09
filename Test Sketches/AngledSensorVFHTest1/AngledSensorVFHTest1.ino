#include <StandardCplusplus.h>
#include <vector>
#include <utility>

#include <NewPing.h>
#include <Servo.h>         //servo library
#include <Wire.h>
#include <LSM303.h>
#include <Yamartino.h>
#include <Streaming.h>

#include "Constants.h"
#include "IOpins.h"

Yamartino yamartino(compass_avg_cnt);
LSM303 compass;
float roll, pitch;
float fXg = 0;
float fYg = 0;
float fZg = 0;

Servo headservo;

// the interval in mS 
unsigned long currentTime;
unsigned long currentTime_avg;

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t obs_array[5];
//const int obsDist = 27;
const int obsDist = 39;  // was 47
const int sidedistancelimit = 45;
unsigned int cm_head[5];

int frtIRdistance, rearIRdistance;
int leftCounter, rightCounter;

//int fowardheadThreshold = 52;    //30.48,41,51,52      headservo - obs[3]
//int lcThreshold = 50;           //40.64,38,45,50      sonarlc obs[1]
//int lcIRthreshold = 40;  //was 45
//int sideSensorThreshold = 45;         //50.8,38,45,41,45,36 sonarll (points to right) obs[0]
//

int fowardheadThreshold = 39; //was 49
int lcThreshold = 37;         // was 47
int lcIRthreshold = 37;  //was 45, last 47
int sideSensorThreshold = 32; //was 42

int backupSensorThreshold = 17;   //17.78 - not implemented yet

int minDistance, nextTurn;
String nextMove, lastMove, turnDirection;
int minIndex;
float mina;

int roam = 0;

//int Speed;
int turn_time_mult;
boolean clockwise;

// read RPM
 volatile byte half_revolutions_l;
 unsigned int rpm_l;
 unsigned long timeold_l;
 unsigned int rpm_l_avg;
 unsigned int rpm_l_index;
 
 volatile byte half_revolutions_r;
 unsigned int rpm_r;
 unsigned int rpm_r_avg;
 unsigned int rpm_r_index;
 unsigned long timeold_r;

 
void setup() {
    telem.begin(9600);
    Wire.begin();

    //==================================================
    // Change I2C bus speed from 100KHz to 400KHz
    //==================================================
    
    #if defined(__AVR_ATmega128) || defined( __AVR_ATmega2560__ ) // only valid on AVR, not on 32bit platforms (eg: Arduino 2, Teensy 3.0)
      if(fastmode) { // switch to 400KHz I2C - eheheh
        TWBR = ((F_CPU / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
      }
    #elif defined(__arm__)
      if(fastmode) {
        #if defined(CORE_TEENSY) && F_BUS == 48000000
          I2C0_F = 0x1A;  // Teensy 3.0 at 48 or 96 MHz
          I2C0_FLT = 2;
        #elif defined(CORE_TEENSY) && F_BUS == 24000000
          I2C0_F = 0x45;  // Teensy 3.0 at 24 MHz
          I2C0_FLT = 1;
        #endif
      }
    #elif defined(__SAM3X8E__) //Arduino Due
      if(fastmode) { // switch to 400KHz I2C - eheheh
        TWBR = ((F_CPU / 400000L) - 16) / 2; 
      }
    #endif
    
    //==================================================
    // Initial AltIMU10 v3
    //==================================================
    
    AltIMU_Init();
    
    delay(100);
    
    telem.println("Ready to receive telem Commands![f, b, r, l, s, t]"); // Tell us I"m ready
    //telem.println("My Commands are: ");
    //telem.println("f:forward");
    //telem.println("b:backward");
    //telem.println("r:right");
    //telem.println("l:left");
    //telem.println("s:stop");
    //telem.println("t:toggleRoam");    
    
    //signal output port
    //set all of the outputs for the motor driver
    pinMode(IN1, OUTPUT);       // Motor Driver 
    pinMode(IN2, OUTPUT);       // Motor Driver
    pinMode(IN3, OUTPUT);       // Motor Driver
    pinMode(IN4, OUTPUT);       // Motor Driver
 
    //Enable pull ups on encoders
    digitalWrite(l_encoder, HIGH);
    digitalWrite(r_encoder, HIGH);


    analogReference(DEFAULT);  //INTERNAL1V1 INTERNAL2V56

    // headservo interface
    headservo.attach(HeadServopin);
    headservo.write(head_fwd);
    delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  read_sensors(); 
  Select_Direction();
while(Serial.available() == 0) { }  // There really is nothing between the {} braces
char x = Serial.read(); 

}
