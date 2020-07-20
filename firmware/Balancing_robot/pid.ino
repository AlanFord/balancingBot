
float pid_calculator(bool reset, float pid_output, float pid_setpoint, float angle_gyro, float self_balance_pid_setpoint) {
  float pid_error_temp;
  static float pid_i_mem = 0;
  static float pid_last_d_error = 0;
  if (reset ) {
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;
    pid_last_d_error = 0;                                                         //Reset the I-controller memory
  }
  else {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //PID controller calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //The balancing robot is angle driven. First the difference between the desired angle (setpoint) and actual angle (process value)
    //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
    //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
    pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
    if (pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

    pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
    //if (pid_i_mem > 400)pid_i_mem = 400;                                      //Limit the I-controller to the maximum controller output
    //else if (pid_i_mem < -400)pid_i_mem = -400;
    //Calculate the PID output value
    pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
    //if (pid_output > 400)pid_output = 400;                                    //Limit the PI-controller to the maximum controller output
    //else if (pid_output < -400)pid_output = -400;

    pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

    if (pid_output < 5 && pid_output > -5)pid_output = 0;                     //Create a dead-band to stop the motors when the robot is balanced

  }
  return pid_output;
}

void direction_control(float pid_output, byte received_byte, control_params &control_parameters) {
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Direction Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  control_parameters.pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  control_parameters.pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if (received_byte & TURN_LEFT) {                                          //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    control_parameters.pid_output_left += turning_speed;                                       //Increase the left motor speed
    control_parameters.pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if (received_byte & TURN_RIGHT) {                                          //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    control_parameters.pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    control_parameters.pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if (received_byte & MOVE_FORWARD) {                                          //If the third bit of the receive byte is set change the left and right variable to move the robot forward
    if (control_parameters.pid_setpoint > -2.5)control_parameters.pid_setpoint -= 0.05;                           //Slowly change the setpoint angle so the robot starts leaning forewards
    if (pid_output > max_target_speed * -1)control_parameters.pid_setpoint -= 0.005;           //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if (received_byte & MOVE_REVERSE) {                                          //If the forth bit of the receive byte is set change the left and right variable to move the robot backward
    if (control_parameters.pid_setpoint < 2.5)control_parameters.pid_setpoint += 0.05;                            //Slowly change the setpoint angle so the robot starts leaning backwards
    if (pid_output < max_target_speed)control_parameters.pid_setpoint += 0.005;                //Slowly change the setpoint angle so the robot starts leaning backwards
  }

  if (!(received_byte & STABILIZE)) {                                       //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if (control_parameters.pid_setpoint > 0.5)control_parameters.pid_setpoint -= 0.05;                            //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if (control_parameters.pid_setpoint < -0.5)control_parameters.pid_setpoint += 0.05;                      //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else control_parameters.pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// convert the pid speed value to a pulse interval
// if pid speed value is 0, set the pulse interval to 0 (special value for ISR)
// if |pulse interval| > MAX_PULSE_INTERVAL, limit pulse interval to +/- MAX_PULSE_INTERVAL
// if |pulse interval| < MIN_PULSE_INTERVAL, limit pulse interval to +/- MIN_PULSE_INTERVAL

/////////////////////////////////////////////////////////////////////////////////////////////
void calculate_motor_pulse_interval(float pid_output_left, float pid_output_right) {
  const int inverted_gain = 4000;
  // const int MAX_PULSE_INTERVAL = INT_MAX-100;
  const int MAX_PULSE_INTERVAL = 256;
  const int MIN_PULSE_INTERVAL = 9;
  int left_motor, right_motor;
  float temp_value;

  if (pid_output_left == 0)
    left_motor = 0;
  else {
    temp_value = (1./pid_output_left) * inverted_gain;
    if (temp_value < -MAX_PULSE_INTERVAL)
      left_motor = -MAX_PULSE_INTERVAL;
    else if (temp_value > MAX_PULSE_INTERVAL)
      left_motor = MAX_PULSE_INTERVAL;
    else
      left_motor = temp_value;
    if (abs(left_motor)<MIN_PULSE_INTERVAL)
      left_motor =  ((left_motor > 0) - (left_motor < 0)) * MIN_PULSE_INTERVAL;
  }

  if (pid_output_right == 0)
    right_motor = 0;
  else {
    temp_value = (1./pid_output_right) * inverted_gain;
    if (temp_value < -MAX_PULSE_INTERVAL) 
      right_motor = -MAX_PULSE_INTERVAL;
    else if (temp_value > MAX_PULSE_INTERVAL)
      right_motor = MAX_PULSE_INTERVAL;
    else
      right_motor = temp_value;
    if (abs(right_motor)<MIN_PULSE_INTERVAL)
      right_motor =  ((right_motor > 0) - (right_motor < 0)) * MIN_PULSE_INTERVAL;
  }

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  noInterrupts();
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
  interrupts();
}

