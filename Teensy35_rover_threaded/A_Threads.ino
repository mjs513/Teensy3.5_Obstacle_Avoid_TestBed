
void readSonar(){
  while(1){
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

    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      if(cm[i] == 0) cm[i] = MAX_DISTANCE;
     }
  }
}


void readIR(){
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


void readIMU(){
  while(1){
    sensors_event_t sensor;
    bno.getEvent(&sensor);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel_linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rot_rate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    roll = (float)sensor.orientation.y;
    pitch = (float)sensor.orientation.z;
    yar_heading = (float)sensor.orientation.x;

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
    threads.delay(BNO055_SAMPLERATE_DELAY_MS);

  }
}

void readGPS(){
 while(1){
  while (gps.available( gpsPort )) {
    gps_fix fix = gps.read();

    satno = fix.satellites;
    hdop = fix.hdop;
    pdop = fix.pdop;
    lat = fix.latitude();
    longit = fix.longitude();
    gpsHeading = fix.heading();
    gpsSpeed = fix.speed_kph();
    laterr = fix.lat_err();
    longerr = fix.lon_err();
    alt = fix.altitude();
    //cout << fix.satellites << ", " << fix.hdop << ", " << fix.pdop << ", ";
    //cout << _FLOAT(fix.latitude(),8)                << ", " << _FLOAT(fix.longitude(),8) << ", "  << fix.heading() << ", ";
    //cout << fix.speed_kph() << ", " << fix.altitude() << ", " << fix.lat_err() << ", ";
    //cout << fix.lon_err() << endl;
  } 
 }
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

