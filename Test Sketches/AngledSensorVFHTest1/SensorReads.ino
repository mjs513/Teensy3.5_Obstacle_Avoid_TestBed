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
  telem << "Distance: ";
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    telem <<  cm[i] << "cm, ";
   }
  telem << endl;
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if(cm[i] < obsDist) {
      obs_array[i] = 1;
    } else {
      obs_array[i] = 0;} 
    telem << obs_array[i] << ", ";
  }
  telem << endl;
  
}

void read_sensors() {
    cm[0] = 0;  
    //unsigned int uS = sonarll.ping();
    unsigned int uS = sonarll.ping_median();
    cm[0] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);  
  
    cm[1] = 0;  
    //uS = sonarlc.ping();
    uS = sonarlc.ping_median();
    cm[1] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL); 
  
    cm[2] = 0;  
    //uS = sonarlr.ping();
    uS = sonarlr.ping_median();
    cm[2] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    cm[3] = 0;  
    uS = sonarhd.ping_median();
    cm[3] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      if(cm[i] == 0) cm[i] = MAX_DISTANCE;
     }
    
    //frtIRdistance = frtIRaverage(3);
    //rearIRdistance = rearIRaverage(3);
    
    //telem << "IR Distances: " << frtIRdistance << " -- " << rearIRdistance << endl;
    //telem << "IR Distances: " << leftIRdistance << " -- " << endl;
    
    //compass_update(); 
    getInclination();

}

void compass_update() {
    for(int i = 0; i < compass_avg_cnt; i++) {
      compass.read();
      //float aheading = compass.heading();
      yamartino.add(compass.heading());
      delay(20);
      //telem.print("Average Wind Direction: ");
      //telem.println(yamartino.averageHeading());
      //telem.println(aheading);
      //telem.println("");
    }
    yar_heading = yamartino.averageHeading();
    telem << "Changed heading: " << yar_heading << endl;
   
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
    
  /*
  for (uint8_t i = 0; i < 5; i++) {
    telem.print(i);
    telem.print("=");
    telem.print(cm_head[i]);
    telem.print("cm Head");
  }
  telem.println();
  */
  
  return;
}

  
int frtIRaverage(int average_count) {
  int sum = 0;
  for (int i=0; i<average_count; i++) {
    int sensor_value = analogRead(leftIRsensor);  //read the sensor value
    int distance_cm = pow(2649.3/sensor_value, 1.2531); //convert readings to distance(cm)
    sum = sum + distance_cm;
  }
  return(sum/average_count);  
}
  
int rearIRaverage(int average_count) {
  int sum = 0;
  for (int i=0; i<average_count; i++) {
    int sensor_value = analogRead(rightIRsensor);  //read the sensor value
    int distance_cm = pow(2471.5/sensor_value, 1.3123); //convert readings to distance(cm)
    sum = sum + distance_cm;
  }
  return(sum/average_count);  
}

void getInclination() {
      compass.read();
      
      float pitch, roll, Xg, Yg, Zg;

      Zg = compass.a.y;
      Yg = compass.a.z;
      Xg = compass.a.x;
  
      //Low Pass Filter
      fXg = Xg * alpha + (fXg * (1.0 - alpha));
      fYg = Yg * alpha + (fYg * (1.0 - alpha));
      fZg = Zg * alpha + (fZg * (1.0 - alpha)); 
      
      //Roll & Pitch Equations
      roll  = 90+(atan2(fYg, -fZg)*180.0)/M_PI;  //reverse signs from An3461
      pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI; 

      telem << endl << pitch << ",  " << roll << endl;

}
