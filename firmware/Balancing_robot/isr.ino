///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) {
  ///////////
  // Note:
  // The binary operations used to set the left and right motor directions are not identical.
  // This is needed because the motors face in opposite directions, hence the forward/backward bit pattern is opposite.
  ///////////
  // Variables for the timer
  volatile static int throttle_counter_left_motor, throttle_left_motor_memory;
  volatile static int throttle_counter_right_motor, throttle_right_motor_memory;

  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttle_counter_left_motor > throttle_left_motor_memory) {           //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if (throttle_left_motor_memory < 0) {                                   //If the throttle_left_motor_memory is negative
      PORTD |= LEFTMOTORDIR;                                                //Set output 3 HIGH to REVERSE the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD &= ~LEFTMOTORDIR;                                            //Set output 3 LOW for a FORWARD direction of the stepper motor
  }
  else if (throttle_counter_left_motor == 1)PORTD |=  LEFTMOTORSTEP;        //Set output 2 HIGH to create a pulse for the stepper controller
  else if (throttle_counter_left_motor == 2)PORTD &= ~LEFTMOTORSTEP;        //Set output 2 LOW because the pulse only has to last for 20us

  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttle_counter_right_motor > throttle_right_motor_memory) {         //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if (throttle_right_motor_memory < 0) {                                  //If the throttle_right_motor_memory is negative
      PORTD &= ~RIGHTMOTORDIR;                                              //Set output 5 LOW to REVERSE the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD |= RIGHTMOTORDIR;                                            //Set output 5 HIGH for a FORWARD direction of the stepper motor
  }
  else if (throttle_counter_right_motor == 1)PORTD |=  RIGHTMOTORSTEP;      //Set output 4 HIGH to create a pulse for the stepper controller
  else if (throttle_counter_right_motor == 2)PORTD &= ~RIGHTMOTORSTEP;      //Set output 4 LOW because the pulse only has to last for 20us
}
