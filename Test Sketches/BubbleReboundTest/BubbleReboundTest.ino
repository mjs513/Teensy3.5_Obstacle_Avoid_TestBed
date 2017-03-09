#include <Servo.h> 
#include <NewPing.h>
#include <Streaming.h>

Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
unsigned int pos1 = 0;    // variable to store the servo position 

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

  unsigned int sonar_dist[9];
  int angle = 20;  //degrees
  int delta_t;
  float V;
  float Ki = 0.2;
  int N = 10;
  float bubble_boundary = 65;
  int blocked;
  float numer, denom;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 

}

void loop() {
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  int start_time = millis();
  
  for(int i =0; i < N; i++)     // goes from 180 degrees to 0 degrees 
  {
    myservo.write(i*angle);               // tell servo to go to position in variable 'pos' 
    delay(550);                           // waits 15ms for the servo to reach the position 
   
    //unsigned int uS = sonar.ping_median(); // Send ping, get ping time in microseconds (uS).
    unsigned int uS = sonar.ping_median(11); 
    sonar_dist[i] = uS / US_ROUNDTRIP_CM;
    if(sonar_dist[i] == 0) sonar_dist[i] = 999;
    Serial << ( sonar_dist[i]) << ", ";
    delay(60); 
   
  } 
  Serial << endl;
  
  int current_time = millis();

  delta_t = (current_time - start_time)/1000;
  V = 200;
  bubble_boundary = Ki*V*delta_t;
  Serial << bubble_boundary << endl;
  
  for(int i=0;i < N;i++) {
    if(sonar_dist[i] <= bubble_boundary)
      blocked = 1;
     else blocked = 0;
  }
  
  Serial << blocked << endl;
  numer = 0;
  denom =0;
  if(blocked == 1) {
    for(int i=0; i < N; i++) {
      numer = numer + radians(angle*i)*sonar_dist[i];
      denom = denom + sonar_dist[i];
    }

    Serial << endl << "REBOUND ANGLE = " << degrees(numer/denom) << " degrees" << endl;  
  }
  
}


int getIndexOfMaximumValue(int* array, int size){
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i=1; i<size; i++){
    if (max<array[i]){
      max = array[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}


/*
unsigned int sonar_readings[N];
unsigned int bubble_boundary[N];
bubble_boundary[i]=Ki*V*delta_t;

int check_for_obstacles(void){
  for(i=0;i<N;i++)
  {
    if(sonar_readings[i]<=bubble_
      boundary[i] return(1);
    else return(0);
}
*/

