// ---------------------------------------------------------------------------
// This example code was used to successfully communicate with 15 ultrasonic sensors. You can adjust
// the number of sensors in your project by changing SONAR_NUM and the number of NewPing objects in the
// "sonar" array. You also need to change the pins for each sensor for the NewPing objects. Each sensor
// is pinged at 33ms intervals. So, one cycle of all sensors takes 495ms (33 * 15 = 495ms). The results
// are sent to the "oneSensorCycle" function which currently just displays the distance data. Your project
// would normally process the sensor results in this function (for example, decide if a robot needs to
// turn and call the turn function). Keep in mind this example is event-driven. Your complete sketch needs
// to be written so there's no "delay" commands and the loop() cycles at faster than a 33ms rate. If other
// processes take longer than 33ms, you'll need to increase PING_INTERVAL so it doesn't get behind.
// ---------------------------------------------------------------------------
#include <NewPing.h>
#include <Servo.h>         //servo library

#define telem Serial

Servo headservo;
const int HeadServopin = 4; // signal input of headservo
//const int head_tilt_pin = 3; // signal input for headservo tilt
//sets up servo angles
const int head_fwd = 90; // center is 80 until i fix
const int head_lft = 0;
const int head_rt = 180;
const int head_ldiag = 45;
const int head_rdiag = 135;

#define SONAR_NUM     4    // Number or sensors.
#define MAX_DISTANCE 500   // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50   // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t obs_array[5];
//const int obsDist = 27;
const int obsDist = 45;
const int sidedistancelimit = 35;
unsigned int cm_head[5];

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(27, 26, MAX_DISTANCE), //lower left sensor
  NewPing(32, 33, MAX_DISTANCE), //front middle sensor
  NewPing(28, 29, MAX_DISTANCE), //lower right sensor
  NewPing(49, 48, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
};

//Hardware set the pins
// L298D Motor Controller
#define ENA 5  //enable A on pin 5 (must be a pwm pin)   Speed Control
#define ENB 3  //enable B on pin 3 (must be a pwm pin)   Speed Control

#define IN1 7  //IN1 on pin controls Motor A  Motor A Right Motor (view from front)
#define IN2 6  //IN2 on pin controls Motor A
#define IN3 4  //IN3 on pin conrtols Motor B  Motor B Left Motor (view from front)
#define IN4 2  //IN4 on pin controls Motor B 

const int rightmotorpin1 = IN1;  //signal output 1 of Dc motor 
const int rightmotorpin2 = IN2;  //signal output 2 of Dc motor 
const int leftmotorpin1  = IN3;  //signal output 3 of Dc motor 
const int leftmotorpin2  = IN4;  //signal output 4 of Dc motor 

int motorSpeed = 55;      //define motor speed parameter which will be mapped as a percentage value
int turnSpeed = 100;      //define turning speed parameter

int Speed;

void setup() {
  telem.begin(9600);
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
    //signal output port
    //set all of the outputs for the motor driver
    pinMode(IN1, OUTPUT);       // Motor Driver 
    pinMode(IN2, OUTPUT);       // Motor Driver
    pinMode(IN3, OUTPUT);       // Motor Driver
    pinMode(IN4, OUTPUT);       // Motor Driver
  
    motorSpeed = 55;            // Set motorSpeed variable with an initial motor speed % (percentage)  low end is about 20

  // headservo interface
  headservo.attach(HeadServopin);
  headservo.write(head_fwd);
  delay(100);
  
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { 		// Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         		// Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  	// Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          	// Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          	// Sensor being accessed.
      cm[currentSensor] = 0;                      	// Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); 	// Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of your code would go here.
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    telem.print(i);
    telem.print("=");
    telem.print(cm[i]);
    telem.print("cm ");
  }
  telem.println();
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if(cm[i] < obsDist) {
      obs_array[i] = 1;
    } else {
      obs_array[i] = 0;} 
    telem.print(obs_array[i]);
    telem.print("  ");
  }
  telem.println();

  decide_direction();
}

void decide_direction() {
  if(obs_array[3] == 1 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
       telem.println("OBSTACLE HEIGHT - STOP or REVERSE");
       //head_distance(); 
       return;
     }
  if(obs_array[0] == 1 && obs_array[2] == 1) {
       telem.println("STOP or Reverse ");
       brake();
       //head_distance(); 
       return;
   }       
  if(obs_array[3] == 0 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
       telem.println("NO OBSTACLES");
       moveForward(motorSpeed);
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 0  && obs_array[2] == 0) {
       telem.println("33% Left Blocked");
       brake();
       body_rturn(turnSpeed); 
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 0  && obs_array[2] == 1) {
       telem.println("33% Right Blocked");
       brake();
       body_lturn(turnSpeed); 
       brake();
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 1  && obs_array[2] == 0) {
       telem.println("50% Left Blocked");
       brake(); 
       body_rturn(turnSpeed);  
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 1  && obs_array[2] == 1) {
       telem.println("50% Right Blocked");
       brake(); 
       body_lturn(turnSpeed); 
       brake();
       return;
   }  
}

void head_distance() {
    headservo.write(head_lft);
    cm_head[0]=sonar[3].ping_result / US_ROUNDTRIP_CM;
    delay(550);
    
    headservo.write(head_ldiag);
    cm_head[1]=sonar[3].ping_result / US_ROUNDTRIP_CM;
    delay(550);
    
    headservo.write(head_fwd);
    cm_head[2]=sonar[3].ping_result / US_ROUNDTRIP_CM;
    delay(550);
    
    headservo.write(head_rdiag);
    cm_head[3]=sonar[3].ping_result / US_ROUNDTRIP_CM;
    delay(550);
    
    headservo.write(head_rt);
    cm_head[4]=sonar[3].ping_result / US_ROUNDTRIP_CM;
    delay(550);

    headservo.write(head_fwd);

  for (uint8_t i = 0; i < 5; i++) {
    telem.print(i);
    telem.print("=");
    telem.print(cm_head[i]);
    telem.print("cm Head");
  }
  telem.println();
  
  decide_direction_head();
}

void decide_direction_head() {
  brake();
  
  //if(obs_array[0] == 1 && obs_array[1] == 1 && obs_array[2] == 1 && obs_array[3] == 1 && obs_array[4] == 1) {
  //     telem.println("All Directions Blocke - REVERSE");
  //     backup(motorSpeed); 
  //     return;
  // }
   
  if(cm_head[0] > cm_head[4] &&  cm_head[0] > cm_head[2] ) {
       telem.println("Head - turn left ");
       body_lturn(turnSpeed); 
       return;
   }       

  if(cm_head[4] > cm_head[0] &&  cm_head[4] > cm_head[0] ) {
       telem.println("Head - turn right");
       body_rturn(turnSpeed); 
       return;
   }

  if(cm_head[1] > cm_head[3] &&  cm_head[1] > cm_head[2] ) {
       telem.println("Head - turn left ");
       body_lturn(turnSpeed); 
       return;
   }       

  if(cm_head[3] > cm_head[1] &&  cm_head[3] > cm_head[0] ) {
       telem.println("Head - turn right");
       body_rturn(turnSpeed); 
       return;
   }
  
  if (cm_head[4] < sidedistancelimit && cm_head[0] < sidedistancelimit && cm_head[2] < obsDist) {
      moveBackward(motorSpeed);
      return;
  } 
}

  
//******************   moveForward *******************
void moveForward(int Speed)
{
   // int motorSpeed);  // change the 15 to the Speed variable, and put Speed int the function call command arguments.
    //also add delayTime for example like this:   moveForward(int delayTime, int motorSpeed)
    
    motorA(2, Speed);  //have motor A turn clockwise at % speed, , call motor control method
    motorB(2, Speed);  //have motor B turn clockwise at % speed
    telem.println("Forward");
}
void moveBackward(int Speed)
{
    motorA(1, Speed);  //have motor A turn counterclockwise 
    motorB(1, Speed);  //have motor B turn counterclockwise
    telem.println("Backward");
}
void body_rturn(int Speed)
{
   motorA(1, Speed);  //have motor A turn counterclockwise 
   motorB(2, Speed);  //have motor B turn clockwise 
   telem.println("Right");
}
   void body_lturn(int Speed)
{
   motorA(2, Speed);  //have motor A turn clockwise
   motorB(1, Speed);  //have motor B turn counterclockwise 
   telem.println("Left");
}

void brake()
{
    motorA(3, 100);  //brake motor A with 100% braking power, call motor control method
    motorB(3, 100);  //brake motor B with 100% braking power, call motor control method
    telem.println("Brake");
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
      telem.println("Disable/coast A");
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      telem.println("turn motor A clockwise");
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      telem.println("turn motor A counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      telem.println("brake motor A");
      
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
      telem.println("Disable/coast B");
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty); 
           telem.println("turn motor B clockwise"); 
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
       telem.println("turn motor B counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      telem.println("brake motor B");      
      break;
  }
}

