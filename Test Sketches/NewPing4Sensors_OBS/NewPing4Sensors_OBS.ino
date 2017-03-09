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

#define telem Serial3

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
#define PING_INTERVAL 33   // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t obs_array[5];
//const int obsDist = 27;
const int obsDist = 31;
const int sidedistancelimit = 25;
unsigned int cm_head[5];

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(27, 26, MAX_DISTANCE), //lower left sensor
  NewPing(32, 33, MAX_DISTANCE), //front middle sensor
  NewPing(28, 29, MAX_DISTANCE), //lower right sensor
  NewPing(49, 48, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
};

//Motor control
const int leftmotorpin1 = 8; //signal output of Dc motor driven plate
const int leftmotorpin2 = 9;
const int rightmotorpin1 = 6;
const int rightmotorpin2 = 7;
const int motor_speed = 50;

void setup() {
  telem.begin(9600);
  
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
  //signal output port
  for (int pinindex = 5; pinindex < 10; pinindex++) {
    pinMode(pinindex, OUTPUT); // set pins 5 to 10 as outputs
  }
  
  // headservo interface
  headservo.attach(HeadServopin);
  headservo.write(head_fwd);
  delay(100);
  
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
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
       totalhalt();
       //head_distance(); 
       return;
   }       
  if(obs_array[3] == 0 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
       telem.println("NO OBSTACLES");
       moveForward();
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 0  && obs_array[2] == 0) {
       telem.println("33% Left Blocked");
       totalhalt();
       body_rturn(120); 
       totalhalt();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 0  && obs_array[2] == 1) {
       telem.println("33% Right Blocked");
       totalhalt();
       body_lturn(120); 
       totalhalt();
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 1  && obs_array[2] == 0) {
       telem.println("50% Left Blocked");
       totalhalt(); 
       body_rturn(220);  
       totalhalt();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 1  && obs_array[2] == 1) {
       telem.println("50% Right Blocked");
       totalhalt(); 
       body_lturn(220); 
       totalhalt();
       return;
   }  
}

// forward
void moveForward() {
   analogWrite(leftmotorpin1, 0);//Changed these to analog write for slower, was 120
   analogWrite(leftmotorpin2, motor_speed);
   analogWrite(rightmotorpin1, 0);
   analogWrite(rightmotorpin2, motor_speed);
   telem.println("Moving Forward");
}

//backward
void backup() {  
  telem.println("Backing Up");
  analogWrite(leftmotorpin1, motor_speed);//Changed these to analog write for slower
  analogWrite(leftmotorpin2, 0);
  analogWrite(rightmotorpin1, motor_speed);
  analogWrite(rightmotorpin2, 0);

  totalhalt();
}
 
void totalhalt() {
  digitalWrite(leftmotorpin1, HIGH);
  digitalWrite(leftmotorpin2, HIGH);
  digitalWrite(rightmotorpin1, HIGH);
  digitalWrite(rightmotorpin2, HIGH);
  return;
}

//turn left
void body_lturn(int turnSpeed) {
  analogWrite(leftmotorpin1, 0);
  analogWrite(leftmotorpin2, turnSpeed);
  analogWrite(rightmotorpin1, turnSpeed);
  analogWrite(rightmotorpin2, 0);
  telem.println("Turning Left");
  delay(400);
}

//turn right
void body_rturn(int turnSpeed) {
  analogWrite(leftmotorpin1, turnSpeed);
  analogWrite(leftmotorpin2, 0);
  analogWrite(rightmotorpin1, 0);
  analogWrite(rightmotorpin2, turnSpeed);
  telem.println("Turning Right");
  delay(400);
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
  totalhalt();
  
  //if(obs_array[0] == 1 && obs_array[1] == 1 && obs_array[2] == 1 && obs_array[3] == 1 && obs_array[4] == 1) {
  //     telem.println("All Directions Blocke - REVERSE");
  //     backup(); 
  //     return;
  // }
   
  if(cm_head[0] > cm_head[4] &&  cm_head[0] > cm_head[2] ) {
       telem.println("Head - turn left ");
       body_lturn(220); 
       return;
   }       

  if(cm_head[4] > cm_head[0] &&  cm_head[4] > cm_head[0] ) {
       telem.println("Head - turn right");
       body_rturn(220); 
       return;
   }

  if(cm_head[1] > cm_head[3] &&  cm_head[1] > cm_head[2] ) {
       telem.println("Head - turn left ");
       body_lturn(220); 
       return;
   }       

  if(cm_head[3] > cm_head[1] &&  cm_head[3] > cm_head[0] ) {
       telem.println("Head - turn right");
       body_rturn(220); 
       return;
   }
  
  if (cm_head[4] < sidedistancelimit && cm_head[0] < sidedistancelimit && cm_head[2] < obsDist) {
      backup();
      return;
  } 
}
