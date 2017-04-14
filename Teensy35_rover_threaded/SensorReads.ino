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

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  int start = millis();
  //telem << "Distance: ";
  //for (uint8_t i = 0; i < SONAR_NUM; i++) {
  //  telem <<  cm[i] << "cm, ";
  // }
  //telem << endl;
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if(cm[i] < obsDist) {
      obs_array[i] = 1;
    } else {
      obs_array[i] = 0;} 
    //telem << obs_array[i] << ", ";
  }
  //telem << endl;  

  //telem << "IR Distances: " << frtIRdistance << " -- " << rearIRdistance << endl;

}

void head_distance() {
    headservo.write(head_lft);
    delay(100);
    cm_head[0]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_ldiag);
    delay(100);
    cm_head[1]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_fwd);
    delay(100);
    cm_head[2]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_rdiag);
    delay(100);
    cm_head[3]= sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_rt);
    delay(100);
    cm_head[4]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    headservo.write(head_fwd);
    
  
  //for (uint8_t i = 0; i < 5; i++) {
  //  telem.print(i);
  //  telem.print("=");
  //  telem.print(cm_head[i]);
  //  telem.print("cm Head");
  //}
  //telem.println();
  
  return;
}


void getTicks_noreset()
{
  ticksRR = encA.read();  //Left Motor
  ticksLR = encB.read();  //Right motor
  
  telem << ticksLR << ",";  telem << ticksRR << ",";

}

void getTicks()
{
  ticksRR = encA.read();  //Left Motor
  ticksLR = encB.read();  //Right motor

}
void send_telemetry(){     
    //===  Telemetry section =========
    if(telem_timer > defaultTelemTime) {

      //DateTime time = rtc.now();
      //telem << time.timestamp(DateTime::TIMESTAMP_TIME);
      //telem << utc << ",";

      telem << etm_millis.elapsed()/1000. << ",";
    
      // IMU
      //compass_update();
      telem << (float) -roll << "," << (float) -pitch << "," << (float) yar_heading << ",";
      //telem << wp_heading << ",";

      //Direction
      telem << "," << gDirection << ",";

      //Wheel Encoders
      // zeros out encoder counts and reads encoders zero value
      // encA.write(0); encB.write(0); encC.write(0); encD.write(0);
      getTicks_noreset();

      stasis();
      telem << " ---- " << motor_on <<  "," << stasis_err << ", " << stasis_flag;
      telem << ", " << _FLOAT((float) gyroz,4) << ", " << _FLOAT((float) accelx, 4);
      telem << ", " << _FLOAT((float) accely, 4) << ", ";

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      telem <<  (int) cm[i] << "cm, "; }

      telem << (int) frtIRdistance << ", " << (int) rearIRdistance << ", ";

      telem << (int) satno << ", " << (float) hdop/1000. << ", " << (float) pdop/1000 << ", ";
      telem << _FLOAT((float) lat, 6) << ", " << _FLOAT((float) longit,6) << ", "  << (float) gpsHeading << ", ";
      telem << (float) gpsSpeed << ", " << (float) alt;

      telem << endl;
      
      telem_timer = 0;
      encA.write(0); encB.write(0);

    }
}





