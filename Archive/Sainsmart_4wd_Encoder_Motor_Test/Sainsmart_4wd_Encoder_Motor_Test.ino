/* 
4WDRobot_Motor_Test
Sketch to test out the operation of the Sainsmart 4WD Robot.
4WD Robot by Glenn Mossy
https://github.com/gmossy/Sainsmart-4WD-Robot
gmossy@gmail.com

DC Robotics Group, Arduino Motors Workshop, Dec 6, 2014
Dec 1, 2014, version 1.0      
      
       *THE SUBROUTINES WORK AS FOLLOWS*

motorA(mode, speed)  // % replace A with B to control motor B %

mode is a number 0 -> 3 that determines what the motor will do.
0 = coast/disable the H bridge
1 = turn motor clockwise
2 = turn motor counter clockwise
3 = brake motor

speed is a number 0 -> 100 that represents percentage of motor speed.
0 = off
50 = 50% of full motor speed
100 = 100% of full motor speed

EXAMPLE
Say you need to have motor A turn clockwise at 33% of its full speed.  The subroutine call would be the following...
motorA(1, 33);
*/

#include "IOpins.h"
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <elapsedMillis.h>
elapsedMillis telem_timer;
elapsedMillis RunTimeTimer;
#define defaultTelemTime 200
#define defaultRunTime 2000
Encoder myEnc(24, 25);

// the interval in mS 
#define interval 7500
unsigned long currentTime;

int roam = 0;             //just listen for serial commands and wait

int motorSpeed;          //define motor speed parameter which will be mapped as a percentage value

// read RPM
 volatile byte half_revolutions_l;
 unsigned int rpm_l;
 unsigned long timeold_l;
 
 volatile byte half_revolutions_r;
 unsigned int rpm_r;
 unsigned long timeold_r;
 
  long position  = -999;
  
void setup() { 
  Serial.begin(9600); // Enables Serial monitor for debugging purposes
  Serial.println("Ready to receive Serial Commands![f, b, r, l, s, t]"); // Tell us I"m ready
  Serial.println("My Commands are: ");
  Serial.println("f:forward");
  Serial.println("b:backward");
  Serial.println("r:right");
  Serial.println("l:left");
  Serial.println("s:stop");
  Serial.println("t:toggleRoam");

  telem_timer = 0;
  
 //signal output port
 //set all of the outputs for the motor driver
  pinMode(IN1, OUTPUT);       // Motor Driver
  pinMode(IN2, OUTPUT);       // Motor Driver
  pinMode(IN3, OUTPUT);       // Motor Driver
  pinMode(IN4, OUTPUT);       // Motor Driver
  
  motorSpeed = 40;        // 55 Set motorSpeed variable with an initial motor speed % (percentage)
}

void loop() 
  {

    position = myEnc.read();

    if (Serial.available() > 0)
    {
    int val = Serial.read();	//read serial input commands

    switch(val)
    {
    case 'f' : 
      Serial.println("Rolling Forward!");
      //currentTime = millis();
      //while(!IsTime(&currentTime, interval)){
      //   encoder_l();
      //   encoder_r();        
      //
      moveForward(motorSpeed);
      while(RunTimeTimer < interval){
         position = myEnc.read();
        if(telem_timer > defaultTelemTime) {
        Serial.println(position);
        telem_timer = 0;
        myEnc.write(0);
        }
      }
      RunTimeTimer = 0;
      brake();
      break;
      
    case 'l' : 
      Serial.println("Turning Left!");
      body_lturn(motorSpeed);
      delay(2000);
      brake();
      delay(5000);
      break;
      
    case 'r' :   
      Serial.println("Turning Right!");
      body_rturn(motorSpeed);
      delay(2000);
      brake();
      delay(5000);
      break;       
   case 'b' :    
      Serial.println("Moving Backward!");
      moveBackward(motorSpeed);
      while(RunTimeTimer < interval){
         position = myEnc.read();
        if(telem_timer > defaultTelemTime) {
        Serial.println(position);
        telem_timer = 0;
        myEnc.write(0);
        }
      }
      RunTimeTimer = 0;
      brake();
      break;
   case 's' :      
      Serial.println("Stop!");
      brake();
      delay(5000);
      break;
   case 't' :      
      Serial.println("toggle Roam Mode"); 
      toggleRoam();
      break;
    }      
    delay(1);  
    Serial.println("I'm Ready to receive Serial Commands![f, b, r, l, s, t]"); // Tell us I"m ready
  }
             
  if(roam == 0){ 
      //just listen for serial commands and wait
      }
  else if(roam == 1){  //If roam active- drive autonomously
    goRoam();
    }
  }
 
void moveForward(int motorSpeed)
{
   // int motorSpeed);  // change the 15 to the Speed variable, and put Speed int the function call command arguments.
    //also add delayTime for example like this:   moveForward(int delayTime, int motorSpeed)
    
    motorA(2, motorSpeed);  //have motor A turn clockwise at % speed, , call motor control method
    motorB(2, motorSpeed);  //have motor B turn clockwise at % speed
    Serial.println("Forward");
}

void moveBackward(int motorSpeed)
{
    motorA(1, motorSpeed);  //have motor A turn counterclockwise 
    motorB(1, motorSpeed);  //have motor B turn counterclockwise
    Serial.println("Backward");
}
void body_rturn(int motorSpeed)
{
   motorA(2, motorSpeed);  //have motor A turn counterclockwise 
   motorB(1, motorSpeed);  //have motor B turn clockwise 
   Serial.println("Right");
}
   void body_lturn(int motorSpeed)
{
   motorA(1, motorSpeed);  //have motor A turn clockwise
   motorB(2, motorSpeed);  //have motor B turn counterclockwise 
   Serial.println("Left");
}

void brake()
{
    motorA(3, 100);  //brake motor A with 100% braking power, call motor control method
    motorB(3, 100);  //brake motor B with 100% braking power, call motor control method
    Serial.println("Brake");
}


//******************   Motor A control   *******************
void motorA(int mode, int percent)
{
 // Method that will accept mode and speed in percentage.  
 // mode: The 3 modes are 0) coast  1) Clockwise motor  2) Counter-clockwise 3) brake
 // percent: The speed of the motor in percentage value for the PWM.
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENA, LOW);  //set enable low to disable A
      Serial.println("Disable/coast A");
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      Serial.println("turn motor A clockwise");
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      Serial.println("turn motor A counter-clockwise");
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      Serial.println("brake motor A");
      
      break;
  }
}
//**********************************************************


//******************   Motor B control   *******************
  void motorB(int mode, int percent)
{
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENB, LOW);  //set enable low to disable B
      Serial.println("Disable/coast B");
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty); 
           Serial.println("turn motor B clockwise"); 
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
       Serial.println("turn motor B counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      Serial.println("brake motor B");      
      break;
  }
}
//**********************************************************

void toggleRoam(){
  // This method chooses to make the robot roam or else use the serial command input.
if(roam == 0){
   roam = 1;
   Serial.println("Activated Roam Mode");
  }
  else{
    roam = 0;
    brake();
    Serial.println("De-activated Roam Mode");
  }
}

void goRoam(){
 // insert roaming function control here. 
   Serial.println("Im going roaming");
    moveForward(motorSpeed);    // temporary just go forward for a little while
   delay(20000);
} 
      

//**********************************************************
//IsTime() function - David Fowler, AKA uCHobby, http://www.uchobby.com 01/21/2012

#define TIMECTL_MAXTICKS  4294967295L
#define TIMECTL_INIT      0

int IsTime(unsigned long *timeMark, unsigned long timeInterval){
  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result=false;
  
  timeCurrent=millis();
  if(timeCurrent<*timeMark) {  //Rollover detected
    timeElapsed=(TIMECTL_MAXTICKS-*timeMark)+timeCurrent;  //elapsed=all the ticks to overflow + all the ticks since overflow
  }
  else {
    timeElapsed=timeCurrent-*timeMark;  
  }

  if(timeElapsed>=timeInterval) {
    *timeMark=timeCurrent;
    result=true;
  }
  return(result);  
}
