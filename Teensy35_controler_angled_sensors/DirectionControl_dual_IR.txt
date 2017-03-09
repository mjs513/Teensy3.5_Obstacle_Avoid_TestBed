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

void decide_direction_dualIR() {
   
   // No forward obstacles
   if(cm[3] > fowardheadThreshold && cm[0] > sideSensorThreshold && 
                 cm[1] > lcThreshold && cm[2] > sideSensorThreshold &&
		 leftIRdistance > lcThreshold && rightIRdistance > lcThreshold) {
			nextMove = "Straight";
		        telem << "(DC) Next Move Straight" << endl;
                 }
				 
   // If everything is blocked in the forward direction lets backupSensorThreshold
   // and run the Bubble rebound/VFH routing
   else if((cm[0] < sideSensorThreshold && cm[2] < sideSensorThreshold && 
                 cm[3] < fowardheadThreshold && cm[1] < lcThreshold) || 
                 (leftIRdistance < lcThreshold && rightIRdistance < lcThreshold)){
                        nextMove = "Backup";
                        telem << "(DC) Everything blocked Next Move Backup" << endl;
		  }

   // Do any of the front facing range sensors detect an obstacle closer than their
   // threshold?  If so, then prepare to turn left or right.
   else if (cm[3] < fowardheadThreshold || cm[1] < lcThreshold || 
		leftIRdistance < lcThreshold || rightIRdistance < lcThreshold)
   {
      nextMove = "LeftRight";
      telem << "(DC) Next Move LeftRight [fwd sensors blocked]" << endl;
   }

   // What about the two angled looking  detectors?
   else if (cm[0] < sideSensorThreshold && cm[2] <  sideSensorThreshold)
   {
       nextMove = "LeftRight";
       telem << "(DC) Next Move LeftRight [both side sensors blocked]" << endl;
   }
   
   //
   // if left facing sensor (right side pointing to the left)
   // is block turn to the right
   //
   else if (cm[2] < sideSensorThreshold || leftIRdistance < lcThreshold)
   {
      nextMove = "Right";
      telem << "(DC) Next Move Right" << endl;
   }
   //
   // if right facing sensor (left side pointing to the right)
   // is block turn to the left
   //
   else if(cm[0] < sideSensorThreshold || rightIRdistance < lcThreshold)
   {
      nextMove = "Left";
      telem << "(DC) Next Move Left" << endl;
   }

   // Reset the closest obstacle distance to a large value.
   minDistance = 10000;
   
  
   // If the closest object is too close, then get ready to back up.
   //if (minDistance < backupSensorThreshold) nextMove = "Backup";
  
   // If we can't go forward, stop and take the appropriate evasive action.
   if (nextMove != "Straight")
   {
       // lets stop and figure out what's next
       brake();
	   
       if (nextMove == "Backup")
       {
          // Keep track of the last action.
          lastMove = "Backup";
		  
          moveBackward();
          delay(250);
          coastBrake();
          delay(250);	//delay half a second
		  
	  // Run Bubble Rebound Algorithm
          Select_Direction();
	  
	  // Remember last direction
          if(clockwise == true) {
            lastMove = "Right";
          } else {
            lastMove = "Left";
	    //return;
          }
		  
        } else {
          // Make sure we don't oscillate back and forth in a corner.
          if (lastMove == "Left" || lastMove == "Right") 
                                             nextMove = lastMove;
          
          if (nextMove == "Left")  
                          nextTurn = -1;
          if (nextMove == "Right") 
                          nextTurn = 1;
          if (nextMove == "LeftRight") {
              if(random(1,10) > 5 ){ 
			nextTurn = 1;
	      } else {
			nextTurn = -1; }
          }
          //Random left or right.
           
          // Keep track of the last turn.
          if (nextTurn == -1) 
              lastMove = "Left";
          else 
              lastMove = "Right";
          
	  if (nextTurn == -1) {
	      body_lturn();
	      delay(275);     //was 1500, 700, 225 - calc at 275 change to 325
	      brake();
	   } else {
	      body_rturn();
	      delay(275);     //was 1500, 700, 225 - calc at 275 change to 325
	      brake(); }

          //return;
        }
		
	} else {
       // If no obstacles are detected close by, keep going straight ahead.
       //moveForward();
       lastMove = "Straight";

       while(cm[3] > fowardheadThreshold && cm[0] > sideSensorThreshold && 
                 cm[1] > lcThreshold && cm[2] > sideSensorThreshold &&
                 leftIRdistance > lcThreshold && rightIRdistance > lcThreshold) {

          moveForward();
           
          rpm_r_index = 0;  rpm_l_index = 0;
          rpm_r_avg = 0;    rpm_l_avg = 0;
          //currentTime = millis();
        
          //while(!IsTime(&currentTime, interval))
          //Read encoders and calculate RPM
          encoder_l();
          encoder_r();
          if(rpm_r !=0) {
             rpm_r_index++; 
             rpm_r_avg = rpm_r_avg + rpm_r;           
           }
          if(rpm_l != 0) { 
             rpm_l_index++;
             rpm_l_avg = rpm_l_avg + rpm_l;
          }
          if(rpm_l != 0 || rpm_r !=0) {
            ///telem.print("(T) AVERAGE (L/R):  "); telem.print(rpm_l_avg/rpm_l_index); 
            //telem.print("    "); telem.println(rpm_r_avg/rpm_r_index);
            //telem.println();
          telem << "(DC) Average (L/R):  " << rpm_l_avg/rpm_l_index << " / " << rpm_r_avg/rpm_r_index << endl;
          }

	  telem << endl << "(DC) End of Tests Updating Sensors" << endl;          
          read_sensors();   
          oneSensorCycle();          
       }    
      brake();
    }
   return;
}   


