void rightBumper(){
  rState = 1;
  bumperBehavior();
}

void leftBumper(){
  lState = 1;
  bumperBehavior();
}

void bumperBehavior(){
  noInterrupts();

  telem << "Switch Interrupt: " << rState << ", " << lState << endl;

  mStop();
  
  set_speed(backup_high);
  mBackward();
  delay(400);
   
  if(rState == 1 && lState == 0) {
    mLeft();
    delay(400);
    rState = 0;  
  }

  if(lState == 1 && rState == 0){
    mRight();
    delay(400);
    lState = 0;    
  }

  if(lState == 1 && rState == 1){
    mRight();
    delay(400);
    lState = 0;
    rState = 0;  
  }
  interrupts();
}

