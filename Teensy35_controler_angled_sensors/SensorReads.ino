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
}

void read_sensors() {
  int start = millis();
    cm[0] = 0;  
    unsigned int uS = sonarll.ping();
    //unsigned int uS = sonarll.ping_median();
    cm[0] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);  
  
    cm[1] = 0;  
    uS = sonarlc.ping();
    //uS = sonarlc.ping_median();
    cm[1] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL); 
  
    cm[2] = 0;  
    uS = sonarlr.ping();
    //uS = sonarlr.ping_median();
    cm[2] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    cm[3] = 0;  
    uS = sonarhd.ping();
    uS = sonarhd.ping_median();
    cm[3] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      if(cm[i] == 0) cm[i] = MAX_DISTANCE;
     }
    
    frtIRdistance = frtIRaverage(3);
    rearIRdistance = rearIRaverage(3);
    
    //telem << "IR Distances: " << frtIRdistance << " -- " << rearIRdistance << endl;

  if(cm[1] == MAX_DISTANCE) {
    if(frtIRdistance <= 0.9 * max_IR_distance ) {
      cm[1] = frtIRdistance;
    }
  }
    
    compass_update(); 
    //getInclination();
}

void compass_update() {
    sensors_event_t event;
    bno.getEvent(&event);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    roll = (float)event.orientation.y;
    pitch = (float)event.orientation.z;
    yar_heading = (float)event.orientation.x;

  // Adjust heading to account for declination
    wp_heading = yar_heading;
    wp_heading += DEC_ANGLE;
    
    //telem << "Compass Yar/dec Heading: " << yar_heading << " , " << heading << endl;
    
    // Correct for when signs are reversed.
    if(wp_heading < 0)
      wp_heading += 360.;
    
    // Check for wrap due to addition of declination.
    if(wp_heading > 360.)
      wp_heading -= 360.;
    
    //telem << roll << "\t" << pitch << "\t" << yar_heading << endl;
    //telem << "Changed heading: " << yar_heading << endl;
   
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

  
int frtIRaverage(int average_count) {
  int sum = 0;
  for (int i=0; i<average_count; i++) {
    int sensor_value = analogRead(leftIRsensor);  //read the sensor value
    if(sensor_value < 100){
      sensor_value = 100; 
    }
    sum = sum + sensor_value;
    delay(5);
    }
  int distance_cm = 27495 * pow(sum/average_count,-1.36); //convert readings to distance(cm)
  return(distance_cm);   
}
  
int rearIRaverage(int average_count) {
  int sum = 0;
  for (int i=0; i<average_count; i++) {
    int sensor_value = analogRead(rightIRsensor);  //read the sensor value
    if(sensor_value < 100){
      sensor_value = 100; 
    }
    sum = sum + sensor_value;
    delay(5);
  }
  int distance_cm = 25445 * pow(sum/average_count,-1.362); //convert readings to distance(cm)
  return(distance_cm);   
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
      telem << -roll << "," << -pitch << "," << yar_heading << ",";
      //telem << wp_heading << ",";

      //Direction
      telem << "," << gDirection << ",";

      //Wheel Encoders
      // zeros out encoder counts and reads encoders zero value
      //encA.write(0); encB.write(0); encC.write(0); encD.write(0);
      getTicks_noreset();
      
      telem << endl;
      
      telem_timer = 0;
      encA.write(0); encB.write(0);


      if(motor_on == 1 && (ticksLR == 0 || ticksRR == 0)){
        telem.println("I am possibly stuck - heheheh !");
      }
    }
}





