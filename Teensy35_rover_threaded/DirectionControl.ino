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

void decide_direction() {
  //interrupts();

  if(sensorTimer1 > sensorUpdate1){
     telem << "Entering Decide Direction Test" << endl;
     update_sensors();
     sensorTimer1 = 0;
  }
  
  // No forward obstacles
  if(hds > fowardheadThreshold && lls > sideSensorThreshold && 
        lcs > lcThreshold && lrs > sideSensorThreshold &&
        irF > lcIRthreshold) {
    nextMove = "Straight";
    //telem << "(DC) Next Move Straight" << endl;
  }
  
  // If everything is blocked in the forward direction lets backupSensorThreshold
  // and run the Bubble rebound/VFH routing
  else if(lls < sideSensorThreshold && lrs < sideSensorThreshold && 
      (hds < fowardheadThreshold || lcs < lcThreshold 
      || irF < lcIRthreshold)) {
    
    moveBackward();
   
    nextMove = "RunBubble";
    //telem << "(DC) Everything blocked Next Move Backup" << endl;
  }
  
  // Special Cases
  
  else if(lls < sideSensorThreshold && lrs == MAX_DISTANCE && 
      hds < fowardheadThreshold && lcs < lcThreshold 
      && irF < lcIRthreshold) {

    //moveBackward();
    //nextMove = "RunBubble";
	
    //nextMove = "Left";
    //turn_time = left_45;
    nextMove = "Right";
    turn_time = right_45;

    //telem << "(DC) Everything blocked Next Move Backup" << endl;
  }

  else if(lrs < sideSensorThreshold && lls == MAX_DISTANCE && 
      hds < fowardheadThreshold && lcs < lcThreshold 
      && irF < lcIRthreshold) {

//    moveBackward();  
//    nextMove = "RunBubble";

    //nextMove = "Right";
    //turn_time = right_45;
    nextMove = "Left";
    turn_time = left_45;
	
    //telem << "(DC) Everything blocked Next Move Backup" << endl;
  } 
  
  // Do any of the front facing range sensors detect an obstacle closer than their
  // threshold?  If so, then prepare to turn left or right. hds < fowardheadThreshold ||
  //else if( lcs < lcThreshold || irF < lcIRthreshold)
    
  // If head sensor is blocked (less than threshold) run bubble and be done with iter_swap
  // else if(hds < fowardheadThreshold) {
  //  nextMove = "RunBubble";
  //}
  
  else if(irF < lcIRthreshold || lcs < fowardheadThreshold
          || hds < fowardheadThreshold) 
		{
			//moveBackward();  
			//nextMove = "RunBubble";

			if(lls < lrs) {       //If left  is greater than right distance move left
				//nextMove = "Left";
				//turn_time = left_45;    //was 37
				nextMove = "Right";
				turn_time = right_45;				
				//telem << "(DC) Next Move is left (Center Blocked)" << endl;
				//mLeft();
				//delay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
				//mStop();
				//nextMove = "Straight";
			} else if(lls > lrs){    //If right is greater than left distance move right
				//nextMove = "Right";
				//turn_time = right_45; //was 37
				nextMove = "Left";
				turn_time = left_45; //was 37
				//telem << "(DC) Next Move is right (Center Blocked)" << endl;
				//mRight();
				//delay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
				//mStop();
				//nextMove = "Straight";
			} else {            //if all else fails run bubble band algorithm
				nextMove = "RunBubble";
				//telem << "(DC) Next Move Determined by bubble [fwd sensors blocked - backup first]" << endl;

			}
  }

  // What about the angled looking  detectors?
  // If right and center distances is less than threshold move left alot
  else if(lls < sideSensorThreshold && lcs < fowardheadThreshold){
    //nextMove = "Left";
    //turn_time = left_57;
	  nextMove = "Right";
	  turn_time = right_57;
    //telem << "(DC) Next Move alot Left" << endl;
  }
  
  // If left and center distances is less than threshold move right alot
  else if(lrs < sideSensorThreshold && lcs < fowardheadThreshold){
    //nextMove = "Right";
    //turn_time = right_57;
    nextMove = "Left";
    turn_time = left_57;
    //telem << "(DC) Next Move alot Right" << endl;
  }
  
  // If right distances is less than threshold and center not blocked 
  // move left a little
  else if (lls < sideSensorThreshold && lcs > fowardheadThreshold)
  {
    //nextMove = "Left";
    //turn_time = left_37;
    nextMove = "Right";
    turn_time = right_37;
    //telem << "(DC) Next Move Left" << endl;
  }
  //
  // If left distance is less than threshold and center not blocked 
  // move right a little
  //
  else if(lrs < sideSensorThreshold && lcs > fowardheadThreshold)
  {
    //nextMove = "Right";
    //turn_time = right_37;
    nextMove = "Left";
    turn_time = left_37;
    //telem << "(DC) Next Move Right" << endl;
  }
  
  // If we can't go forward, stop and take the appropriate evasive action.
  if (nextMove != "Straight")
  {
    // lets stop and figure out what's next
    mStop();
    set_speed(turnSpeed);

    //telem << lastMove << ", " << nextMove << endl << endl;
    
    if(lastMove == "Right" && nextMove == "Left") {
      nextMove == "Right";
      turn_time = right_57;
      turnCounter = turnCounter + 1;
      //telem << "(OSC) Next Move Right" << endl;
	  
     } else if(lastMove == "Left" && nextMove == "Right") {
      nextMove == "Left";
      turn_time = left_57;
      turnCounter = turnCounter + 1;
      //telem << "(OSC) Next Move Left" << endl;
      }
      
    if(turnCounter != 0 && nextMove != "RunBubble"){
      nextMove = lastMove;
    }
    
    //telem << "Actual Move: " << nextMove << endl << endl;
    
    turn_timer = 0;
	
    if (nextMove == "Right") {
		  mRight();
      gDirection = DIRECTION_RIGHT;
		  smartDelay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
		  mStop();
		  send_telemetry();
	  } else if (nextMove == "Left") { 
        gDirection = DIRECTION_LEFT;
        mLeft();
        smartDelay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
        mStop();
        send_telemetry();
       } else if (nextMove == "RunBubble") {
			    turnCounter = 0;
			    mBackward();
			    smartDelay(200);
			    mStop();
			    Select_Direction();
       }

    lastMove = nextMove;    
    
    } else {
		// If no obstacles are detected close by, keep going straight ahead.
		lastMove = "Straight";
    
		set_speed(speed);
    
		//reset counters after clearing obstacle
		turnCounter = 0;
    
		mForward();
		//keep moving forward until next obstacle is found
		while(hds > fowardheadThreshold && lls > sideSensorThreshold && 
          lcs > lcThreshold && lrs > sideSensorThreshold &&
          irF > lcIRthreshold && stasis_flag == 0) {

			if(telem.available() > 0) {
				int val = telem.read();  //read telem input commands  
				if(val == 't') {
					//telem.println("toggle Roam Mode");
          //noInterrupts();
					toggleRoam();
					return;
				}
			}
			  //telem << "(DC) End of Tests Updating Sensors" << endl << endl;  

			  //Send telemetry data to radio
			  gDirection = DIRECTION_FORWARD;
        getTicks();
			  send_telemetry();

        if(sensorTimer > sensorUpdate){
          telem << "While Loop for Going Straight" << endl;
          update_sensors();
          sensorTimer = 0;
        }
 
			  //read_sensors();   
			  //oneSensorCycle();
		  }   
		  mStop();
      if(stasis_flag == 1) stasis_correction();
    }
}   

void moveBackward() {
  //If I have to move backward 1st test to see to see if there is a obstacle
  //if no obstacle move a lot, if obstacle move a little and then run the bubble
  //rebound algorithm
    
  //telem << "Moving Backward to avoid obstacle" << endl;
  int irR = rearIRdistance;
  if(irR > backupSensorThreshold) {
    set_speed(backup_high);
    mBackward();
    smartDelay(450);
    mStop();
    lastMove == "Backup";
  } else {
    set_speed(backup_low);
    mBackward();
    smartDelay(250);
    mStop();
    lastMove == "Backup";
  }
    gDirection = DIRECTION_BACKWARD;
    send_telemetry();
}

// This custom version of delay() ensures that the encoder object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    getTicks();
    send_telemetry();
  } while (millis() - start < ms && stasis_flag == 0);
  
  if(stasis_flag == 1) {
    mStop();
    stasis_correction();
  }
  
}

void update_sensors(){
  lls = cm[0];
  lcs = cm[1];
  lrs = cm[2];
  hds = cm[3];
  irF = frtIRdistance;
  irR = rearIRdistance;

  telem << lls << ", " << lcs << ", " << lrs << ", " << hds << endl;
  telem << irF << ", " << irR << endl;
}

