

#include <NewPing.h>
#include <Streaming.h>
#define pingCount = 5;

#define SONAR_NUM     4          // Number or sensors.
#define MAX_DISTANCE 250        // Maximum distance (in cm) to ping.
#define PING_INTERVAL 40        // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

#define telem Serial
//#define telem Serial3 // bluetooth

const int HeadServopin = 8; // signal input of headservo
//const int head_tilt_pin = 3; // signal input for headservo tilt

NewPing sonarll(27, 26, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlc(32, 33, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlr(28, 29, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarhd(49, 48, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
 
void setup() {
   telem.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
   read_sensors();
}

void read_sensors() {

    int currentTime = millis();
    cm[0] = 0;  
    //unsigned int uS = sonarll.ping();
    unsigned int uS = sonarll.ping_median(5);
    cm[0] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);  
  
    cm[1] = 0;  
    //uS = sonarlc.ping();
    uS = sonarlc.ping();
    cm[1] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL); 
  
    cm[2] = 0;  
    //uS = sonarlr.ping();
    uS = sonarlr.ping();
    cm[2] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    cm[3] = 0;  
    uS = sonarhd.ping();
    cm[3] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      if(cm[i] == 0) cm[i] = MAX_DISTANCE;
      telem << i << "=" << cm[i] << "cm, ";
     }
    telem << (millis() - currentTime) << endl << endl;
}


