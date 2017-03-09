 void rc_control() {
  interrupts();
  
	while( rc_mode_toggle == 1) {
		usb.Task();

		if (PS4.connected()) {
		
		//telem.println();
		// create local variables to hold a local copies of the channel inputs
		// these are declared static so that thier values will be retained
		// between calls to loop.
		static uint16_t unThrottleIn;
		static uint16_t unSteeringIn;
  
		// local copy of update flags
		static uint8_t bUpdateFlags;
		
		unThrottleIn = PS4.getAnalogHat(LeftHatY);
		unSteeringIn = PS4.getAnalogHat(RightHatX);

    if (PS4.getButtonClick(PS)) {
      telem.print(F("\r\nPS Disconnected\n"));
      PS4.disconnect();
      set_speed(speed);
      telem.println("toggle RC Mode off\n"); 
      toggleRC();
      return;
    }

		//Tests to determine if throttle or steering is in the specified dead zone
		tDeadZoneRange = rangeTest(unThrottleIn, ThrottleDeadBandMin, ThrottleDeadBandMax);
		sDeadZoneRange = rangeTest(unSteeringIn, SteeringDeadBandMin, SteeringDeadBandMax);

		// do any processing from here onwards
		// only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
		// variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
		// the interrupt routines and should not be used in loop

		// we are checking to see if the channel value has changed, this is indicated 
		// by the flags. For the simple pass through we don't really need this check,
		// but for a more complex project where a new signal requires significant processing
		// this allows us to only calculate new values when we have new inputs, rather than
		// on every cycle.
		//if(bUpdateFlags & THROTTLE_FLAG)
		//{
		  // A good idea would be to check the before and after value, 
		  // if they are not equal we are receiving out of range signals
		  // this could be an error, interference or a transmitter setting change
		  // in any case its a good idea to at least flag it to the user somehow
		  unThrottleIn = constrain(unThrottleIn, unThrottleMin, unThrottleMax);
      
		  if(unThrottleIn < unThrottleCenter) 
		  { 
			  gThrottle = map(unThrottleIn, unThrottleCenter, unThrottleMin, PWM_MIN, PWM_MAX );
			  throttleLeft = throttleRight = gThrottle;
			  gThrottleDirection = DIRECTION_FORWARD;
			  //telem.print("Dir_FWD:"); telem.println(gThrottle);
		  } else  {
			  gThrottle = map(unThrottleIn, unThrottleCenter, unThrottleMax, PWM_MIN, PWM_MAX );
			  throttleLeft = throttleRight = gThrottle;
			  gThrottleDirection = DIRECTION_BACKWARD;
			  //telem.print("Dir_REV:"); telem.println(gThrottle);
		  }
		//}
   
		gDirection = gThrottleDirection;
  
		if(gThrottle < IDLE_MAX) {
			gGear = GEAR_IDLE;
		}
		
		if(gThrottle > IDLE_MAX) {
			gGear = GEAR_FULL;
		}
		
		if(tDeadZoneRange == 1 && sDeadZoneRange == 1){
			gGear = GEAR_NEUTRAL;
		}

		//telem.print("GEAR: "); telem.println(gGear);
		
		//if(bUpdateFlags & STEERING_FLAG)
		//{
			throttleLeft = gThrottle;
			throttleRight = gThrottle;

			gDirection = gThrottleDirection;
			//telem.print("Direction: "); telem.println(gDirection);
    
			// see previous comments regarding trapping out of range errors
			// this is left for the user to decide how to handle and flag
			unSteeringIn = constrain(unSteeringIn, unSteeringMin, unSteeringMax);

			// if idle spin on spot
			switch(gGear)
			{
				case GEAR_NEUTRAL:
					gDirection = DIRECTION_STOP;
					break;
				case GEAR_IDLE:  
					// same changes for steering as for throttle
					if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
					{
						gDirection = DIRECTION_LEFT;
						// use steering to set throttle
						throttleRight = throttleLeft = map(unSteeringIn, unSteeringCenter, unSteeringMin, PWM_MIN, PWM_MAX);
						//telem.print("Rotate Leftt: "); telem.println(throttleRight);
					} else if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
					{
						gDirection = DIRECTION_RIGHT;
						// use steering to set throttle
						throttleRight = throttleLeft = map(unSteeringIn, unSteeringCenter, unSteeringMax, PWM_MIN, PWM_MAX);
						//telem.print("Rotate Right: "); telem.println(throttleRight);
					}
					break;
					// if not idle proportionally restrain inside track to turn vehicle around it
				case GEAR_FULL:
					if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
					{
						throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,gThrottle,PWM_MIN);
						throttleRight = throttleRight;
						if(gThrottle < 100) {
							throttleRight = throttleRight;
							throttleLeft = throttleLeft;
						}
						//telem.print("1. Turn Right: "); telem.print(throttleRight);
						//telem.print(" LEFT: "); telem.println(throttleLeft);
					} else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND)) {
						throttleRight = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MIN,gThrottle);
						throttleLeft = throttleLeft;
						if(gThrottle < 100) {
							throttleRight = throttleRight;
							throttleLeft =  throttleLeft;
						}
						//telem.print("2. Turn Left: "); telem.print(throttleRight);
						//telem.print("  LEFT: "); telem.println(throttleLeft);
					}
          telem << endl;
					break;        
			}
		//} 

		// if idle spin on spot

		if(telem.available() > 0 ) {
			int val = telem.read();  //read telem input commands  
			if(val == 'p') {
        PS4.disconnect();
        set_speed(speed);
				telem.println("toggle RC Mode off"); 
				toggleRC();
				return;
			}
		}

		//Send telemetry data to radio
    noInterrupts();
		send_telemetry();
    interrupts();

      
		//telem.print("Direction Updated: "); telem.println(gDirection);
		//telem.println();
		switch(gDirection)
		{
		  case DIRECTION_FORWARD:  
			  if(gGear == GEAR_FULL) {
          motorSpeed_right = throttleRight;
          motorSpeed_left = throttleLeft;
					mForward();
				}
			  break;
		  case DIRECTION_BACKWARD:
        motorSpeed_right = throttleRight;
        motorSpeed_left = throttleLeft;
				mBackward();
			  break;
		  case DIRECTION_LEFT:
        motorSpeed_right = throttleRight;
        motorSpeed_left = throttleLeft;
			  mLeft();
			  break;
		  case DIRECTION_RIGHT:
        motorSpeed_right = throttleRight;
        motorSpeed_left = throttleLeft;
			  mRight();
			  break;
		  case DIRECTION_STOP:
			  mStop();
			  break;
		}

		bUpdateFlags = 0;

		}
	}
 }


bool rangeTest(uint16_t number, uint16_t lower, uint16_t upper) {
  // use a < for an inclusive lower bound and exclusive upper bound
  // use <= for an inclusive lower bound and inclusive upper bound
  // alternatively, if the upper bound is inclusive and you can pre-calculate
  //  upper-lower, simply add + 1 to upper-lower and use the < operator.
  if ((unsigned)(number-lower) <= (upper-lower)) {
    return true;
    } else {
      return false;
    }
}





