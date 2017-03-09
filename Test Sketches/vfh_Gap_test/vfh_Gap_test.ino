

#define MAX_DISTANCE 200   			// Maximum distance (in cm) to ping.
#define PING_INTERVAL 50   			// Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

const int N = 12;  //was 10, 12 for 12 readings
const int angle = 15;  //was 20 degrees
 
#include <StandardCplusplus.h>
#include <vector>
#include <utility>

#include <NewPing.h>
#include <Servo.h>         //servo library
#include <Wire.h>
#include <Streaming.h>

const int HeadServopin = 8; // signal input of headservo
Servo headservo;

//sets up servo angles
const int head_fwd = 90; 


NewPing sonarhd(12, 11, 200); // NewPing setup of pins and maximum distance.
//const int obsDist = 27;
const int obsDist = 47;
const int sidedistancelimit = 45;

float yar_heading, new_heading, angle_delta;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();
    
    // headservo interface
    headservo.attach(HeadServopin);
    headservo.write(head_fwd);
    delay(100);

    //local variables
    int i, start, left;
    float new_angle, Delta_Angle;
    float numer, denom;
    unsigned int sonar_dist[N+1], Hist[N+1];
    
    std::vector<std::pair<int,int> > border;
    std::vector<float> Candidate_Angle;
    std::pair<int,int> new_border;
    std::vector<std::pair<float,float> > rebound; 
 
    // Rotate head by N angles defined in Constants.h
    // default is 12 sectors and 15 Degrees from 0-180 degrees
    //
    for(i =0; i <= N; i++)     // goes from 180 degrees to 0 degrees 
    {
      headservo.write(i*angle);               // tell servo to go to position in variable 'pos' 

      delay(550);                             // waits 15ms for the servo to reach the position 
      //Read head ultrasonic sensor  
      unsigned int uS = sonarhd.ping(); 
      sonar_dist[i] = uS / US_ROUNDTRIP_CM;
      if(sonar_dist[i] == 0) sonar_dist[i] = 999;

      if(sonar_dist[i] <= obsDist) 
        Hist[i] = 1;
       else
        Hist[i] = 0;
      //Serial <<  Hist[i] << ", " << sonar_dist[i] << endl;    
      delay(PING_INTERVAL); 
    } 
    Serial << endl;
  
/////////////////////////////////////////////////// 
// only look at the forward 180deg for first obstacle.
    for(i = 0;i <= N;i++) 
    {
      if (Hist[i] == 1) 
      {
          start = i;
          break;
      }
    }
    
    //
    // Find the left and right borders of each opening
    //
    border.clear();

    //Serial << "START: " << start << endl;
    start = 0;
    left = 1;
    for(i=start;i<=(start+N);i++) {
      if ((Hist[i % (N+1)] == 0) && (left)) {
        new_border.first = (i % (N+1)) * angle;
        left = 0;
        //Serial << "A-BORDER: " << new_border.first << endl;
      }

      if ((Hist[i % (N+1)] == 1) && (!left)) {
        new_border.second = ((i % (N+1)) - 1) * angle;
        border.push_back(new_border);
        left = 1;
        //Serial << "B-BORDER: " << new_border.second << endl;
      }
    }
    
    if(left == 0){
      new_border.second = (N) * angle;
      border.push_back(new_border);
      left = 1;
      //Serial << "B-BORDER: " << new_border.second << endl;
    }
   
    //
    // Consider each opening
    //
    int angle_cnt = 0;
    for(i=0;i < (int)border.size();i++) 
    {
      //Serial << "BORDER: " << border[i].first << " ,   " << border[i].second << endl;
      Delta_Angle = border[i].second - border[i].first + angle;
      //Serial << "DELTA ANGLE: " << Delta_Angle << endl << endl;
    
      if (fabs(Delta_Angle) < 60) 
      {
        // narrow opening: aim for the centre
        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;
        //Serial << "DELTA ANGLE: " << Delta_Angle << endl;
        Serial << "Narrow Opening (New Angle):  " << new_angle << endl;
        Candidate_Angle.push_back(new_angle);
      } else {
        new_angle =  + (border[i].second - border[i].first) / 2.0;
        Serial << "Wide Opening (Center):  " << new_angle << endl;
        Candidate_Angle.push_back(new_angle);
        
        //Bubble rebound for opening   
        numer = 0;  denom =0;
        //Serial << "a-BORDER: " << border[i].first << " ,   " << border[i].second << endl;
        int start_index = border[i].first/angle;
        int end_index = border[i].second/angle;
        
        //Serial << start_index << ", " << end_index << endl;
        for(int i=start_index; i <= end_index; i++) {
          numer = numer + radians(angle*i)*sonar_dist[i];
          denom = denom + sonar_dist[i];
        }
        
        rebound[angle_cnt].second = denom / (end_index-start_index+1);
        rebound[angle_cnt].first = degrees(numer/denom);
        Serial << "REBOUND ANGLE = " << rebound[angle_cnt].first << " degrees" << endl;
        Serial << "Average Distance: " << rebound[angle_cnt].second <<endl;  
        Serial << endl;
        angle_cnt = angle_cnt + 1;
      }
    }

    headservo.write(head_fwd);

   int maxIndex = 0;
   float maxa = rebound[maxIndex].second;
   for (int i=1; i < angle_cnt; i++){
     if (maxa < rebound[i].second){
       maxa = rebound[i].second;
       maxIndex = i;
     }
   }
   Serial << "Best Angle:  " << rebound[maxIndex].first << endl;

}    


void loop() {
  // put your main code here, to run repeatedly:

}
