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
//#define telem Serial3 // bluetooth

//Enable 400Khz I2C bus speed
const boolean fastmode = true;

//int motorSpeed_right = 87;        //define motor speed parameter which will be mapped as a percentage value
//int motorSpeed_left = 75;         // these are reversed right is left when looking from behind
const int motorSpeed_right = 82;    //define motor speed parameter which will be mapped as a percentage value
                                    // with canakit seems like i have to increase this value
const int motorSpeed_left = 84;     // offset required for differences in motor speed,82, (84/86)
const int turnSpeed = 80;           //define turning speed parameter, was 75, was 87
                                    //change due to canakit driver

//sets up servo angles
const int head_fwd = 90; 
const int head_lft = 0;
const int head_rt = 180;
const int head_ldiag = 45;
const int head_rdiag = 135;

#define SONAR_NUM     4          // Number or sensors.
#define MAX_DISTANCE 200        // Maximum distance (in cm) to ping.
#define PING_INTERVAL 40        // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
                                          // was set to 33
// the interval in mS 
#define interval 7500    //was 7500
#define interval1 2000

//compass reads
const int compass_avg_cnt = 20;
const float alpha = 0.5;

//Bubble Rebound Parameters
const float V = 21;
const float Ki = 0.2;

const int N = 20;  //was 10, 12 for 12 readings, was 12
const int angle = 9;  //was 20 degrees, was 15 for 12



