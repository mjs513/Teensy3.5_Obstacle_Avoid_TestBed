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

#define telem Serial
//#define telem Serial4 // bluetooth

//Enable 400Khz I2C bus speed
const boolean fastmode = true;
//int fowardheadThreshold = 52;    //30.48,41,51,52      headservo - obs[3]
//int lcThreshold = 50;           //40.64,38,45,50      sonarlc obs[1]
//int lcIRthreshold = 40;  //was 45
//int sideSensorThreshold = 45;         //50.8,38,45,41,45,36 sonarll (points to right) obs[0]
//
//const int obsDist = 27;
int obsDist = 27; //was 47
int sidedistancelimit = 27;  // was 27, 2/6/17 increas to 37
int fowardheadThreshold = 29; //was 49, 39, 29; was 27, increase to 32 (2/6)
int lcThreshold = 29;         // was 47,27; was 26, increase to 29 (2/6)
int lcIRthreshold = 15;  //was 45, last 47; was 27; was 15
int sideSensorThreshold = 27; //was 42; was 27, 2/6/17 increas to 37
int max_IR_distance = 200;
int turnCounter = 0;

int backupSensorThreshold = 17;   //17.78 - not implemented yet
int backup_high = 65;
int backup_low = 50;

//int motorSpeed_right = 87;        //define motor speed parameter which will be mapped as a percentage value
//int motorSpeed_left = 75;         // these are reversed right is left when looking from behind
int motorSpeed_right = 50;          //define motor speed parameter which will be mapped as a percentage value
                                    // with canakit seems like i have to increase this value, increase to 50
int motorSpeed_left = 50;           // offset required for differences in motor speed,82, (84/86)
int turnSpeed = 75;                 //define turning speed parameter, was 75, was 87
                                    //change due to canakit driver
int speed = 40;                     //increase from 40 to 50 (2/6)
int turn_time_mult = 2;
int turn_time;

uint8_t motor_on = 0;

//sets up servo angles
const int head_fwd = 90; 
const int head_lft = 0;
const int head_rt = 180;
const int head_ldiag = 45;
const int head_rdiag = 135;

#define SONAR_NUM     4    			// Number or sensors. 4 with head
#define MAX_DISTANCE 200   			// Maximum distance (in cm) to ping.
#define PING_INTERVAL 40   			// Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
                                          // was set to 33

// the interval in mS 
//#define interval 7500    //was 7500
#define interval 100
#define interval1 2000
#define defaultTurnTime 1000
#define defaultFwdTime 10000 //was 7000
#define defaultRevTime 700
#define defaultTelemTime 500
#define defaultWayPointTime 1000
#define defaultOdoTime 50

//Bubble Rebound Parameters
const float V = 21;
const float Ki = 0.2;

//compass reads
//#define DEC_ANGLE -13.1603  // -13.1603 degrees for Flushing, NY
#define DEC_ANGLE 0

const int N = 31;  //was 10, 12 for 12 readings, was 12
const int angle = 6;  //was 20 degrees, was 15 for 12

//const int N = 25;  //was 10, 12 for 12 readings, was 12
//const int angle = 7.5;  //was 20 degrees, was 15 for 12

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD 2
#define DIRECTION_RIGHT 3
#define DIRECTION_LEFT 4
#define DIRECTION_PIVOT_RIGHT 5
#define DIRECTION_PIVOT_LEFT 6


// Waypoints  Constants
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading
float wp_heading;

// Speeds (range: 0 - 255)
int FAST_SPEED = 60;
#define NORMAL_SPEED 40
#define TURN_SPEED 30
int TURN_SPEED_DIFF = 75;
#define SLOW_SPEED 20
//#define NORMAL_SPEED speed


const int left_37 = 443;  //was 387 for 37 deg, 250 for 22deg (443
const int left_45 = 513; 
const int left_57 = 614;  //was 571 for 57, 461 for 45 (499)

const int right_37 = 482; //was 440 for 37, 295 for 22 deg 482
const int right_45 = 543;
const int right_57 = 628; //was 636 for 57, 519 for 45 (543)
   

//rcarduino constants
#define RC_NEUTRAL_STEERING 128  //1504 was 1506
#define RC_NEUTRAL_THROTTLE 128  //1500 was 1511

#define RC_MAX_STEERING 255
#define RC_MAX_THROTTLE 255

#define RC_MIN_STEERING 0  //1012 was 1016
#define RC_MIN_THROTTLE 0  //1012 was 1014

#define RC_DEADBAND 40

#define RC_MODE_TOGGLE 128

uint16_t unSteeringMin = RC_MIN_THROTTLE;
uint16_t unSteeringMax = RC_MAX_STEERING;
uint16_t unSteeringCenter = RC_NEUTRAL_STEERING;

uint16_t unThrottleMin = RC_MIN_THROTTLE;
uint16_t unThrottleMax = RC_MAX_THROTTLE;
uint16_t unThrottleCenter = RC_NEUTRAL_THROTTLE;

		  
uint16_t ThrottleDeadBandMax = RC_NEUTRAL_THROTTLE + RC_DEADBAND;
uint16_t ThrottleDeadBandMin = RC_NEUTRAL_THROTTLE - RC_DEADBAND;
uint16_t SteeringDeadBandMax = RC_NEUTRAL_STEERING + RC_DEADBAND;
uint16_t SteeringDeadBandMin = RC_NEUTRAL_STEERING - RC_DEADBAND;

#define PWM_MIN 0
#define PWM_MAX 100

#define GEAR_NONE 0
#define GEAR_IDLE 1
#define GEAR_FULL 2
#define GEAR_NEUTRAL 3

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define RC_FLAG 3

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;


uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 40
   
   
   

//***Odometry
#define TRACK 9.5
#define WHEEL_DIA 2.5
#define CLICKS_PER_REV 16



