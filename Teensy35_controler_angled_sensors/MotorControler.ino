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
//============================================================================

//******************   moveForward *******************
//void moveForward(int Speed)
void mForward()
{
   // int motorSpeed);  // change the 15 to the Speed variable, and put Speed int the function call command arguments.
    //also add delayTime for example like this:   moveForward(int delayTime, int motorSpeed
    
    gDirection = DIRECTION_FORWARD;
    motor_on == 1;
    //motorA(2, Speed);  //have motor A turn clockwise at % speed, , call motor control method
    motorA(2, motorSpeed_right);  //have motor B turn clockwise at % speed (Right side)
    motorB(2, motorSpeed_left*1.1);  //have motor B turn clockwise at % speed,0.73 (Left Side)
    //telem.println("Forward");
}

//void moveBackward(int Speed)
void mBackward()
{
    gDirection = DIRECTION_BACKWARD;
    motor_on = 1;
    //motorA(1, Speed);  //have motor A turn counterclockwise 
    //motorB(1, Speed);  //have motor B turn counterclockwise
    motorA(1, motorSpeed_right * 0.75);  //have motor A turn counterclockwise 
    motorB(1, motorSpeed_left  * 0.75*1.1);  //have motor B turn counterclockwise
    //telem.println("Backward");
}

void mRight()
{
   gDirection = DIRECTION_RIGHT;
   motor_on = 1;
   motorA(2, motorSpeed_right);  //have motor A turn counterclockwise 
   motorB(1, motorSpeed_left*1.1);  //have motor B turn clockwise 
   //telem.println("Right");
}

void mLeft()
{
   gDirection = DIRECTION_LEFT;
   motor_on = 1;
   motorA(1, motorSpeed_right);  //have motor A turn clockwise
   motorB(2, motorSpeed_left*1.1);  //have motor B turn counterclockwise 
   //telem.println("Left");
}

void mStop()
{
    gDirection = DIRECTION_STOP;
    motor_on = 0;
    motorA(3, 100);  //brake motor A with 100% braking power, call motor control method
    motorB(3, 100);  //brake motor B with 100% braking power, call motor control method
    //telem.println("Brake");
}

void coastBrake() {
    motorA(3, 50);  //coast motor A with 100% braking power, call motor control method
    motorB(3, 50);  //brake motor B with 100% braking power, call motor control method
    //telem.println(" Coast Brake");
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
      //telem.println("Disable/coast A");
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      //telem.println("turn motor A clockwise");
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      //telem.println("turn motor A counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      //telem.println("brake motor A");
      
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
      //telem.println("Disable/coast B");
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty); 
      //telem.println("turn motor B clockwise"); 
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
       //telem.println("turn motor B counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      //telem.println("brake motor B");      
      break;
  }
}

void set_speed(int motor_speed) {
  motorSpeed_right = motor_speed;
  motorSpeed_left = motor_speed;
}



