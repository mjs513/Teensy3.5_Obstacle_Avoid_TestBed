void stasis(){
  if(motor_on == 0 && (ticksLR == 0 || ticksRR == 0)){
      telem.println("I am possibly stuck - heheheh !");
      stasis_err = stasis_err + 1;
  } 
  
  if(stasis_err > 4)  stasis_flag = 1;
}

void stasis_correction(){
    stasis_err = 0;
    stasis_flag = 0;
    mStop();
    set_speed(backup_high);
    mBackward();
    smartDelay(450);
    mStop();
    
    if( random(10) > 4){
      mRight();
      smartDelay(right_45);
      mStop();
    } else {
      mLeft();
      smartDelay(left_45);
      mStop();
    }

}

