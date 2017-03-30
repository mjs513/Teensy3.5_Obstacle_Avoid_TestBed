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

void Select_Direction() {

    threads.suspend(id1);  //suspend sonar thread
    
    ////telem << endl << endl << "************************************" << endl;
    ////telem << "Entering Select Direction Function" << endl;

    ///////////////////////////
    //
    // set local variables
    //
    ///////////////////////////
    int i, start, left, delay_time;
    float fit_time, new_angle, Delta_Angle, angle_delta;
    float numer, denom;
    unsigned int sonar_dist[N+1], Hist[N+1];
    unsigned int nowidegap = 1;

    /////////////////////////////////////////////////////
    //
    // Establish vector pairs to store opening borders
    // and calculated rebound angles.
    // Requires the StandardCplusplus library in repository
    // Original version will not compile correctly in latest
    // Arduino IDEs
    //
    /////////////////////////////////////////////////////

    std::vector<std::pair<int,int> > border;
    std::pair<int,int> new_border;
    std::vector<std::pair<float,float> > rebound; 
    rebound.reserve(10);
 

    /////////////////////////////////////////////////////////
    //
    // Rotate head by N angles defined in Constants.h
    // default is 12 sectors and 15 Degrees from 0-180 degrees
    //
    /////////////////////////////////////////////////////////

    for(i = 0; i < N; i++)
    {
      headservo.write(i*angle);               
      delay(250);    //was 550                         

      /////////////////////////////////////////////////////////
      //
      // Read head ultrasonic sensor  
      // Head sensor already attached in rover3_controller.ino
      // This code assumes you are using the NewPing library.
      // If you can not use NewPing you will have to provide your
      // sensor read function, i.e., your own timings and distance
      // conversion.
      //
      /////////////////////////////////////////////////////////

      unsigned int uS = sonarhd.ping_median(); 
      sonar_dist[i] = uS / US_ROUNDTRIP_CM;
      if(sonar_dist[i] == 0) sonar_dist[i] = MAX_DISTANCE;

      /////////////////////////////////////////////////////////
      //
      // Essentially creating a binary histogram of obstacles
      // 
      /////////////////////////////////////////////////////////
      ////telem << "GAP TEST:  ";
      if(sonar_dist[i] <= obsDist) 
        Hist[i] = 1;
       else
        Hist[i] = 0;
        
      //telem << i*angle << ", " << Hist[i] << ", " << sonar_dist[i] << endl;   
      delay(PING_INTERVAL); 
    }
    //headservo.write(90);
    //telem << endl;
  
    /////////////////////////////////////////////////////////
    //
    // Start GAP analysis based of vFH
    // Code based on select_direction from VFH_algorithm.cpp
    // of Orca Robot Components
    //
    /////////////////////////////////////////////////////////

    for(i = 0;i < N;i++) 
    {
      if (Hist[i] == 1) 
      {
          start = i;
          break;
      }
    }
    
    ///////////////////////////////////////////////////
    //
    // Find the left and right borders of each opening
    //
    ///////////////////////////////////////////////////

    border.clear();

    //telem << "START: " << start << ", " << N << endl;
    start = 0;
    left = 1;
    for(i=start;i<=(start+N);i++) {
      if ((Hist[i % (N+1)] == 0) && (left)) {
        new_border.first = (i % (N+1)) * angle;
        left = 0;
        //telem << "A-BORDER: " << new_border.first << endl;
      }

      if ((Hist[i % (N+1)] == 1) && (!left)) {
        new_border.second = ((i % (N+1)) - 1) * angle;
        border.push_back(new_border);
        left = 1;
        //telem << "B-BORDER: " << new_border.second << endl;
      }
    }
    
    if(left == 0){
      new_border.second = (N) * angle;
      border.push_back(new_border);
      left = 1;
      //telem << "B-BORDER: " << new_border.second << endl;
    }
   
    //////////////////////////////////////////////////
    //
    // Evaluate each opening - narrow or wide
    // currently looking for wide openings only while
    // still identifying narrow openings.
    //
    //////////////////////////////////////////////////

    int angle_cnt = 0;
    
    for(i=0;i < (int)border.size();i++) 
    {
      //telem << "BORDER: " << border[i].first << " ,   " << border[i].second << endl;
      Delta_Angle = border[i].second - border[i].first + angle;
      //telem << "DELTA ANGLE: " << Delta_Angle << endl << endl;
    
      if (fabs(Delta_Angle) < 45)   // was 60
      {
        ////////////////////////////////////////////////////
        //
        // narrow opening: aim for the centre if so desired
        // 
        ////////////////////////////////////////////////////

        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;
        //telem << "DELTA ANGLE: " << Delta_Angle << endl;
        //telem << "Narrow Opening (New Angle):  " << new_angle << endl;

      } else {
        new_angle =+ (border[i].second - border[i].first) / 2.0;
        //telem << "Wide Opening (Center):  " << new_angle << endl;
        nowidegap = 0;
        
        ////////////////////////////////////////////////////
        //
        // Calculate Bubble rebound angles for wide opening  
        //
        ////////////////////////////////////////////////////
 
        numer = 0;  denom =0;
        //telem << "a-BORDER: " << border[i].first << " ,   " << border[i].second << endl;
        int start_index = border[i].first/angle;
        int end_index = border[i].second/angle;
        
        //telem << start_index << ", " << end_index << endl;
        for(int i=start_index; i <= end_index; i++) {
          numer = numer + radians(angle*i)*sonar_dist[i];
          denom = denom + sonar_dist[i];
          //telem << i << ", " << numer << ", " << denom << endl;
        }

        //////////////////////////////////////////////////
        //
        // Load Rebound vector with rebound angle and 
        // average of read distances of the opening
        // this will be used as a descriminator for the
        //  prefered opening  
        //
        //////////////////////////////////////////////////
        
        //telem << denom / (end_index-start_index+1) << endl;
        rebound[angle_cnt].second = denom / (end_index-start_index+1);
        rebound[angle_cnt].first = degrees(numer/denom);

        //telem << "REBOUND ANGLE = " << rebound[angle_cnt].first << " degrees" << endl;
        //telem << "Average Distance: " << rebound[angle_cnt].second <<endl;  
        //telem << endl;
        angle_cnt = angle_cnt + 1;
      }
    }

    headservo.write(head_fwd);

    //////////////////////////////////////////////////////
    //
    // Best angle based on highest average value of sensor
    // distances for opening
    //
    //////////////////////////////////////////////////////

    if(nowidegap == 0) {
      int maxIndex = 0;
      float maxa = rebound[maxIndex].second;
      for (int i=1; i < angle_cnt; i++){
        if (maxa < rebound[i].second){
          maxa = rebound[i].second;
          maxIndex = i;
        }
      }
   
      float rebound_angle = rebound[maxIndex].first - 90;
      //telem << "Best Angle:  " << rebound_angle << endl;

      //////////////////////////////////////////////////
      //
      // read compass, calculate expected new compass
      // heading
      //
      //////////////////////////////////////////////////

      //compass_update(); 
      //telem << "Current heading: " << yar_heading << endl; 
  
      float new_heading = yar_heading + (rebound_angle);
      //telem << "Actual calc for new heading: " << new_heading << endl;
  
      if(new_heading > 360.0f)
         new_heading -= 360.0f;
      else if(new_heading < 0.0f)
        new_heading = 360.0f + new_heading;


      //////////////////////////////////////////////////////
      //
      // Determine whether turn direction will be clockwise
      // or counterclockwise
      //
      //////////////////////////////////////////////////////
  
      //boolean clockwise;
      clockwise = false;
  
      angle_delta = abs(new_heading - yar_heading);
      clockwise = (rebound_angle >= 0.0f);
  
      //telem << "New Heading: " << new_heading << "  Clockwise: " << clockwise << endl;
      //telem << "Angle delta: " << angle_delta << endl;
      //telem << endl;

      ////////////////////////////////////////////////////////
      //
      // Turn time is based on a predefined calibration curve
      // angle vs time.  CW and CCW calibrations are different
      // so if you use this option make sure you verify the
      // calibration in both directions.
      //
      ////////////////////////////////////////////////////////

      if(clockwise) {
        fit_time = 0.00001*pow(rebound_angle,3)+0.0444*pow(rebound_angle,2)+10.844*rebound_angle+135.94;
        delay_time = ceil(fit_time);
        //telem << "Curve Fit (CW): " << fit_time << endl;
		
        if(delay_time < 150) delay_time = 150;
		nextMove = "Right";
		gDirection = DIRECTION_RIGHT;
        mRight();
		
      } else {
        fit_time = 29.474*pow(rebound_angle,0.7506);
        delay_time = ceil(fit_time/2.0);
        //telem << "Curve Fit (CCW): " << fit_time << endl; 
		
        if(delay_time < 150) delay_time = 150;  // was 50
		nextMove = "Left";
		gDirection = DIRECTION_LEFT;
        mLeft();
      }
  
      //telem << "Turn Time: " << delay_time << endl;
      delay(delay_time);
      mStop();
    
      //compass_update();
      //telem << "New Heading: " << yar_heading << endl;
    } else {
      //telem << "No viable (wide) gap !" << endl;
      
      // Do a clockwise 180 and return
      float rebound_angle = 180;
      fit_time = 0.00001*pow(rebound_angle,3)+0.0444*pow(rebound_angle,2)+10.844*rebound_angle+135.94;
      delay_time = ceil(fit_time);
      //telem << "Curve Fit (CW): " << fit_time << endl;
	  
	  nextMove = "Right";
	  gDirection = DIRECTION_RIGHT;
      mRight();
      delay(delay_time);
    }

    threads.restart(id1);
}


  




