
void sonar_thread(){
  while(1){
    cm[0] = 0;  
    unsigned int uS = sonarll.ping();
    //unsigned int uS = sonarll.ping_median();
    cm[0] = uS / US_ROUNDTRIP_CM;
    threads.delay(PING_INTERVAL);  
  
    cm[1] = 0;  
    uS = sonarlc.ping();
    //uS = sonarlc.ping_median();
    cm[1] = uS / US_ROUNDTRIP_CM;
    threads.delay(PING_INTERVAL); 
  
    cm[2] = 0;  
    uS = sonarlr.ping();
    //uS = sonarlr.ping_median();
    cm[2] = uS / US_ROUNDTRIP_CM;
    threads.delay(PING_INTERVAL);

    cm[3] = 0;  
    uS = sonarhd.ping();
    //uS = sonarhd.ping_median();
    cm[3] = uS / US_ROUNDTRIP_CM;
    threads.delay(PING_INTERVAL);

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      if(cm[i] == 0) cm[i] = MAX_DISTANCE;
     }
  }
}


void sharp_dist_thread(){
  while(1){  
  int sum = 0;
  for (int i=0; i<3; i++) {
    int sensor_value = analogRead(leftIRsensor);  //read the sensor value
    if(sensor_value < 100){
      sensor_value = 100; 
    }
    sum = sum + sensor_value;
    threads.delay(5);
    }
    frtIRdistance = 27495 * pow(sum/3,-1.36); //convert readings to distance(cm)

  sum = 0;
  for (int i=0; i<3; i++) {
    int sensor_value = analogRead(rightIRsensor);  //read the sensor value
    if(sensor_value < 100){
      sensor_value = 100; 
    }
    sum = sum + sensor_value;
    threads.delay(5);
  }
  rearIRdistance = 25445 * pow(sum/3,-1.362); //convert readings to distance(cm)
  }
}


void bno055_thread(){
  while(1){
    sensors_event_t event;
    bno.getEvent(&event);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel_linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rot_rate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    roll = (float)event.orientation.y;
    pitch = (float)event.orientation.z;
    yar_heading = (float)event.orientation.x;

    gyroz = (float) rot_rate.z() * rad2deg;
    accelx = (float) accel_linear.x() * m2ft;
    accely = (float) accel_linear.y() * m2ft;

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
}

