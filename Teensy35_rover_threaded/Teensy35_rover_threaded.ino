//============================================================================
//    Sketch to test various technicques in robotic car design such as
//    obstacle detection and avoidance, compass as turn guide,
//    motor control, etc.
//    Copyright (C) 2015  Michael J Smorto
//    https://github.com/mjs513/Sainsmart_4WD_Obstacle_Avoid_TestBed.git
//    FreeIMU@gmail.com
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// ---------------------------------------------------------------------------
// Code based on example code from following:
// Dagu Rover 5 Motor Control:
//    Created by: William Garrido - FriedCircuits.us
//    Exmaple of Dagu 4 Channel Motor Controller with 4 motors on the Rover 5
//    Encoder support is provided by Teensy Encoder Library.
//    telem processing code is borrowed from
//    https://github.com/hbrobotics/ros_arduino_bridge
// Sainsmart Obstacle Avoidance Robot:
//    http://www.mkme.org/index.php/arduino-sainsmart-4wd-robot/
//    https://github.com/gmossy/Sainsmart-4WD-Robot
// Wheel Encoders - the DAGU Simple Encoder Kit
//    http://www.bajdi.com/adding-encoders-to-those-cheap-yellow-motors/
//    http://letsmakerobots.com/node/38636
//    http://playground.arduino.cc/Main/ReadingRPM
//    http://elimelecsarduinoprojects.blogspot.com/2013/06/measure-rpms-arduino.html 
//  Compass Averaging
//    Yamartino Library, Christopher Baker  https://github.com/SAIC-ATS/Algorithms.git
//  Obstacle avoidance approaches
//    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/index.html
//    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/software.html
//  IR Sensing
//    http://letsmakerobots.com/node/40502
//    http://adhocnode.com/arduino-irrecv-module/
//    http://arduino-info.wikispaces.com/IR-RemoteControl
//
//  More to follow as I add PID controls and Bubble rebound algorithm for obstacle
//  avoidance
//
//  Advance obstacle avoidance algoritm:
//    Susnea, I., Viorel Minzu, Grigore Vasiliu. Simple, real-time obstacle
//    avoidance algorithm for mobile robots. in 8th WSEAS International
//    Conference on Computational intelligence, man-machine systems and
//    cybernetics (CIMMACS'09) 2009
//    http://www.wseas.us/e-library/conferences/2009/tenerife/CIMMACS/CIMMACS-03.pdf
//
//    Ulrich, I., and Borenstein, J., "VFH+: Reliable Obstacle
//    Avoidance for Fast Mobile Robots", IEEE Int. Conf. on Robotics and Automation, May 1998,
//    pp. 1572-1577. http://www.cs.cmu.edu/~iwan/papers/vfh+.pdf
//
//    VFH Gap routine extracted from Orca: Components for Robotics, vfh algorithm
//    http://orca-robotics.sourceforge.net/
//
//  Use of angle sensors and draft avoidance logic references
//    Obstacle Avoidance in the Real World: 
//      http://www.pirobot.org/blog/0003/
//      https://youtu.be/4yXysCkcdwQ
//       Where Am I? Place Recognition Using Omni-directional Images and Color Histograms
//       http://forums.trossenrobotics.com/tutorials/introduction-129/where-am-i-place-recognition-using-omni-directional-images-and-color-histograms-3253/
//    Megamouse An Autonomous Maze Solving Robot
//       https://djgeorgevas.com/static/megamouse.pdf
//
//==================== DAGU ROVER 5 =========================================================
// GPS: UBlox NEO-M8N with HMC5883L magnetometer - Library from SensorPlot
//      TinyGPSplus works now also for Neo-M8N sensors,
//      https://github.com/SensorsIot/TinyGPSplus-for-Neo-M8N.git
//
//
//    : NeoGPS Library, https://github.com/SlashDevin/NeoGPS
//      This fully-configurable Arduino library uses minimal RAM, PROGMEM and CPU time, 
//      requiring as few as 10 bytes of RAM, 866 bytes of PROGMEM, and less than 1mS of 
//      CPU time per sentence.
//    : See Arduino forum threads
//        NeoGPS - configurable, ultra-small RAM footprint, http://forum.arduino.cc/index.php?topic=289342.0
//        Polling issue with a Neo-M8N using a Mega2560, http://forum.arduino.cc/index.php?topic=429369.0
//============================================================================================
// rcarduino.blogspot.com
//
// A simple approach for reading two RC Channels from a hobby quality receiver
// and outputting to the common motor driver IC the L293D to drive a tracked vehicle
//
// We will use the Arduino to mix the channels to give car like steering using a standard two stick
// or pistol grip transmitter. The Aux channel will be used to switch and optional momentum mode on and off
//
// See related posts -
//
// Reading an RC Receiver - What does this signal look like and how do we read it - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
//
// The Arduino library only supports two interrupts, the Arduino pinChangeInt Library supports more than 20 - 
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
//
// The Arduino Servo Library supports upto 12 Servos on a single Arduino, read all about it here - 
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// The wrong and then the right way to connect servos to Arduino
// http://rcarduino.blogspot.com/2012/04/servo-problems-with-arduino-part-1.html
// http://rcarduino.blogspot.com/2012/04/servo-problems-part-2-demonstration.html
//
// Using pinChangeInt library and Servo library to read three RC Channels and drive 3 RC outputs (mix of Servos and ESCs)
// http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
//
// RC Channels, L293D Motor Driver - Part 2 Calibration And Code 
// http://rcarduino.blogspot.com/2012/05/interfacing-rc-channels-to-l293d-motor.html
//
// rcarduino.blogspot.com
//
// ==================================================================
// Utilities:
//    Teensy 3.2 Interrupt, https://forum.pjrc.com/threads/35733-Teensy-3-2-Interrupt
// 
// ==================================================================
// Odometry:
//    IMU Odometry, by David Anderson
//    http://geology.heroy.smu.edu/~dpa-www/robo/Encoder/imu_odo/
//
//    Position Estimation
//    http://www.robotnav.com/position-estimation/
// -------------------------------------------------------------------

#include <vector>

#include <EEPROM.h>
#include <NewPing.h>
#include <NMEAGPS.h>
#include <Servo.h>         //servo library
#include <Wire.h>
#include <Streaming.h>
#include <Encoder.h>
#include <StopWatch.h>
#include <elapsedMillis.h>
#include "TeensyThreads.h"
#include <atomic>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

NMEAGPS  gps; // This parses the GPS characters
#define gpsPort Serial6

//Setup PS4 Controller
#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
  #include <spi4teensy3.h>
  #include <SPI.h>
#endif

USB usb;
//USBHub Hub1(&usb); // Some dongles have a hub inside
BTD Btd(&usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in paring mode
//PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

Servo headservo;

#include "Constants.h"
#include "IOpins.h"

// Set elapsed time constants
elapsedMillis motorFwd;
elapsedMillis motorFwdRunTime;
elapsedMillis motorTurnTime;
elapsedMillis motorRevTime;
elapsedMillis turn_timer;
elapsedMillis telem_timer;
elapsedMillis gps_waypoint_timer;
elapsedMillis odo_timer;
elapsedMillis sensorTimer;
elapsedMillis sensorTimer1;

StopWatch etm_millis;

// the interval in mS 
unsigned long currentTime;
unsigned long currentTime_avg;
unsigned long sensorRunTime;


#define SONAR_NUM     4 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 45 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
 
volatile unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
volatile unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.

NewPing sonar[SONAR_NUM] = {
  NewPing(27, 26, MAX_DISTANCE), //lower left sensor
  NewPing(36, 35, MAX_DISTANCE), //front middle
  NewPing(30, 29, MAX_DISTANCE), //lower rigth sensro
  NewPing(34, 33, MAX_DISTANCE)  // Each sensor's trigger pin, echo pin, and max distance to ping.
};

NewPing sonarhd(34, 33, MAX_DISTANCE);  // Each sensor's trigger pin, echo pin, and max distance to ping.

uint8_t obs_array[5];
std::atomic<int> cm_head[5];

std::atomic<int> frtIRdistance, rearIRdistance;
std::atomic<int> leftCounter, rightCounter;

int lls, lcs, lrs, hds, irF, irR;

//encoder variables
long ticksRR,
     ticksLR,
     left_encoder_count,
     right_encoder_count;

uint8_t stasis_err = 0;
uint8_t stasis_flag = 0;
long randNumber;

volatile uint8_t rState = 0;
volatile uint8_t lState = 0;
uint8_t motor_on = 0;

//GPS Values
std::atomic<float> lat, longit, hdop, pdop, alt;
std::atomic<float> gpsSpeed, laterr, longerr;
std::atomic<float> gpsHeading;
std::atomic<int> satno;

//*** Odometry variables
int odo_mode_toggle = 0;
float new_heading,
      init_heading,
      ENCODER_SCALE_FACTOR,
      pos_x, 
      pos_y,
      odo_start_distance;

int odo_start = 0;

// global for heading from compass
std::atomic<float> yar_heading;
std::atomic<float> roll, pitch;
std::atomic<float> gyroz, accelx, accely;
float fXg = 0;
float fYg = 0;
float fZg = 0;

// Compass navigation
float targetHeading,              // where we want to go to reach current waypoint
      currentHeading,             // where we are actually facing now
      headingError;               // signed (+/-) difference between targetHeading and currentHeading

/* Set the delay between fresh samples for BNO055*/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int minDistance, nextTurn;
String nextMove, lastMove, turnDirection;
int minIndex;
float mina;

int roam = 0;
int stop_flag = 0;
int rc_mode_toggle = 0;
int rc_sw_on = 0;

//rcarduino shared variables
// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unRCInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulRCStart;

uint8_t throttleLeft;
uint8_t throttleRight;
int tDeadZoneRange, sDeadZoneRange;


//int Speed;
boolean clockwise;

int Sonar, IR, IMU, GPS;

void setup() {
    telem.begin(57600);
    while ( !telem && millis() < 2000 ) {};

    Wire.begin();
    Wire.setClock(400000L);
    gpsPort.begin(57600);

    delay(2000);
    
	  if (usb.Init() == -1) {
		  telem.print(F("\r\nOSC did not start"));
		  while (1); // Halt
	  }
	  telem.println(F("\r\nPS4 Bluetooth Library Started"));
    telem << endl;


    //==================================================
    // Initial Adafruit BNO055
    //==================================================
    
    BNO055_Init();
    
    delay(100);
    //Set Motor Speed
    //speed = 50;
    //turnSpeed = 150;
      
    //signal output port
    //set all of the outputs for the motor driver
    pinMode(IN1, OUTPUT);       // Motor Driver 
    pinMode(IN2, OUTPUT);       // Motor Driver
    pinMode(IN3, OUTPUT);       // Motor Driver
    pinMode(IN4, OUTPUT);       // Motor Driver

    //Bumper Switches
    pinMode(rSwitch, INPUT_PULLUP);
    pinMode(lSwitch, INPUT_PULLUP);
    attachInterrupt(rSwitch, rightBumper, RISING);
    attachInterrupt(lSwitch, leftBumper, RISING);

    //headservo interface
    headservo.attach(HeadServopin);
    headservo.write(head_fwd);
    delay(100);

    randomSeed(analogRead(0));

    pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
    for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
      pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

    Sonar = threads.addThread(readSonar);
    IR = threads.addThread(readIR);
    IMU = threads.addThread(readIMU);
    GPS = threads.addThread(readGPS);
    threads.setTimeSlice(0, 1);
    threads.setTimeSlice(Sonar, 20);
    threads.setTimeSlice(IR, 1);
    threads.setTimeSlice(IMU, 50);
    threads.setTimeSlice(GPS, 50);
    if (threads.getState(Sonar) == Threads::RUNNING) Serial.println("Sonar thread started");
    if (threads.getState(IR) == Threads::RUNNING) Serial.println("IR thread started");
    if (threads.getState(IMU) == Threads::RUNNING) Serial.println("BNO055 thread started");
    if (threads.getState(GPS) == Threads::RUNNING) Serial.println("GPS thread started");

    telem << "Ready to receive telem Commands![f, b, r, l, s, t]" << endl; // Tell us I"m ready
    //telem.println("My Commands are: ");
    //telem.println("f:forward");
    //telem.println("b:backward");
    //telem.println("r:right");
    //telem.println("l:left");
    //telem.println("s:stop");
    //telem.println("t:toggleRoam");    
  
}

void loop() {

    oneSensorCycle();
    
    //Continuously read servos
    ticksRR = encB.read();
    ticksLR = encA.read();
    
    if (telem.available() > 0)
    {
    int val = telem.read();	//read telem input commands

    turn_time_mult = telem.parseInt();
    if(turn_time_mult == 0)
                turn_time_mult = 4;          
    
    switch(val)
    {
  
    case 'f' : 
        motorFwdRunTime = 0;
        motorFwd = 0;
        odo_timer = 0;
        set_speed(speed);
        gDirection = DIRECTION_FORWARD;        
        telem.println("Rolling Forward!");
        mForward();
        etm_millis.start();        
       while(motorFwdRunTime < defaultFwdTime){
        //currentTime = millis();
        if((motorFwd % 600) == 0){
          //Cycle through obstacle avoidance sensors for emergency stop
          //read_sensors();
          oneSensorCycle();
          if(obs_array[0] == 1 || obs_array[1] == 1 || obs_array[2] == 1) {
            stop_flag = 1;
          } else {
            stop_flag = 0;
          }
          motorFwd = 0;
        }
        if(stop_flag == 1){
          mStop();
        } else {
          mForward();
        }
       }

      mStop();
      stop_flag = 0; 
      etm_millis.stop();
      motorFwdRunTime = 0;
      break;
      
    case 'l' :
      motorTurnTime = 0;
      //===================================================
      // Used for turn calibration curve
      //===================================================
      //compass_update();
      //telem << "Current heading: " << yar_heading << endl;
      //telem << "Turn Multiplier: " << turn_time_mult << endl;
      //telem.println("Turning Left!");
      set_speed(turnSpeed);
      //compass_update();
      telem << "Change heading: " << turn_time_mult*100 << ", " << (float) yar_heading << ", ";
       mLeft();
      //delay(400);  //was 2000
      delay(turn_time_mult * 100);
      mStop();
      while(motorTurnTime < defaultTurnTime) {
       }
      mStop();
      //compass_update();
      telem << (float) yar_heading << endl;

      motorTurnTime = 0;
      break;
      
    case 'r' :
      //===================================================
      // Used for turn calibration curve
      //===================================================
      //telem << "Turn Multiplier: " << turn_time_mult << endl;
      //telem.println("Turning Right!");
      //compass_update();
      set_speed(turnSpeed);
      //compass_update();
      telem << "Change heading: " << turn_time_mult*100 << ", " << (float) yar_heading << ", ";
      mRight();
      //delay(400);
      delay(turn_time_mult * 100);
      mStop();
      while(motorTurnTime < defaultTurnTime) {
        }
      mStop();
      //compass_update();
      telem << (float) yar_heading << endl;
      motorTurnTime = 0;
      break;
      
   case 'b' :    
      motorRevTime = 0;    
      telem.println("Moving Backward!");
      //moveBackward(motorSpeed);
      set_speed(backup_high);
      mBackward();
      while(motorRevTime < defaultRevTime) {
        }
      mStop();
      motorRevTime = 0;
      break;
      
   case 's' :      
      telem.println("Stop!");
      mStop();
      break;
      
    case 't' :      
      telem.println("toggle Roam Mode"); 
      toggleRoam();
      break;
      
    case 'o' :
      telem << "Odometry Activated" << endl;
      toggleOdo();
      break;
	  
    case 'p' :
      telem.println("toggle RC control mode");
      toggleRC();
      break;
    }      
    delay(1);  
    telem.println("I'm Ready to receive telem Commands![f, b, r, l, s, t]"); // Tell us I"m ready
  }
      
  if(roam == 0){ 
      noInterrupts();
      //just listen for telem commands and wait
    }
  else if(roam == 1){  //If roam active- drive autonomously
    interrupts();
    goRoam();
    }

  if(odo_mode_toggle == 0){ 
      //just listen for telem commands and wait
     }
  else if(odo_mode_toggle == 1) {  //If roam active- drive autonomously
    toggleOdo();
    }
	
  if(rc_mode_toggle == 0){ 
      //just listen for telem commands and wait
      }
  else if(rc_mode_toggle == 1) {  //If roam active- drive autonomously
    goRC();
    }
    
}

void toggleRoam(){
  // This method chooses to make the robot roam or else use the telem command input.
  if(roam == 0){
   roam = 1;
   etm_millis.start();
   telem.println("Activated Roam Mode");
  } else {
    etm_millis.stop();
    etm_millis.reset();
    roam = 0;
    mStop();
    telem.println("De-activated Roam Mode");
    telem.println("I'm Ready to receive telem Commands![g, f, b, r, l, s, t, p, o]"); // Tell us I"m ready

  }
}

void goRoam() {  
  
   //read_sensors();   
   //oneSensorCycle(); 
   decide_direction();
    
}

void toggleOdo(){
  if(odo_mode_toggle == 0) {
    odo_mode_toggle = 1;
    telem << "Odometry Nav Activated" << endl;
    telem.println("I'm Ready to receive telem Commands![f (inches), b (inches), r (degs), l (degs), s, o]");
    pos_x = pos_y = 0;
    odometry();
  } else {
    odo_mode_toggle = 0;
    mStop();
    telem << "Odometry Nav De-activated" << endl;
    telem.println("I'm Ready to receive telem Commands![g, f, b, r, l, s, t, c, w, o]"); // Tell us I"m ready
  }
}



void toggleRC(){
  // This method chooses to make the robot roam or else use the telem command input.
  if(rc_mode_toggle == 0){
   rc_mode_toggle = 1;
   telem.println("Activated RC Mode");
   etm_millis.start();
  } else {
    rc_mode_toggle = 0;
    throttleLeft = throttleRight = speed;
    mStop();
    etm_millis.stop();
    etm_millis.reset();
    noInterrupts();
    telem.println("De-activated RC Mode");
	  telem.println("I'm Ready to receive telem Commands![g, f, b, r, l, s, t, c, w, o]"); // Tell us I"m ready
  }
}

void goRC() {  
	rc_control();   
}


