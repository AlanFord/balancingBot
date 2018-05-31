/////////////////////////////////////////////////////////////////////////////////////
/// \file motor.cpp
/// \brief Throttle and motor control functions for the stepper motors
/// that drive the robot
///
/////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "motor.h"
#include "DebugUtils.h"

// these values are used to communicate between motor_pulse_calculations() and the interrupt handler
static int throttle_left_motor, throttle_right_motor;

////////////////////////////////////////////////////////
///  \brief Turning speed (20).
////////////////////////////////////////////////////////
#define turning_speed 30                                   
////////////////////////////////////////////////////////
///  \brief Max target speed (100).
////////////////////////////////////////////////////////
#define max_target_speed 150                                


///////////////////////////////////////////////////////////////////////////////
/// \brief TIMER2 interrupt handler used to update the stepper motor control lines
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  static int throttle_counter_left_motor = 0, 
            throttle_counter_right_motor = 0, 
            throttle_left_motor_memory = 0, 
            throttle_right_motor_memory = 0;
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory) {            //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}


///////////////////////////////////////////////////////////////////////////////
/// \brief Adjust pid_output data for motion control commands 
///
/// \param[in] received_byte - control byte received from the nunchuck
/// \param[in,out] self_balance_pid_setpoint - pid referenc value for stable upright position
/// \param[in] pid_output - control byte received from the nunchuck
/// \param[in,out] pid_setpoint - pid setpoint
///
/// \return void 
///////////////////////////////////////////////////////////////////////////////
///      >>>>>>>>>>>>>>>>>>>>>>>UPDATE<<<<<<<<<<<<<<<<<<<<<<<<<<<
void control_calculations(byte received_byte, float& self_balance_pid_setpoint, float pid_output, float& pid_setpoint, float& pid_output_left, float& pid_output_right) {
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  // Turn Left
  if(received_byte & BOT_LEFT) {                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
    DEBUG_PRINT("Turning Left");
  }
  // Turn Right
  if(received_byte & BOT_RIGHT) {                                           //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
    DEBUG_PRINT("Turning Right");
  }
  // Move Forward
  if(received_byte & BOT_FORWARD) {                                         //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    DEBUG_PRINT("Go Forward");
  }
  // Move Backward
  if(received_byte & BOT_REVERSE) {                                         //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
    DEBUG_PRINT("Go Backward");
  }   
  // No motion - slowly return to stable/vertical
  if(!(received_byte & BOT_STABLE)) {                                       //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0) {                                                   //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }  
}



/* 
 *  For file layout considerations, see 
 *  https://arduino.stackexchange.com/questions/13178/classes-and-objects-how-many-and-which-file-types-do-i-actually-need-to-use-the/13182#13182
 */


///////////////////////////////////////////////////////////////////////////////
/// \brief Calculates motor pulse outputs from the pid output 
///
/// \return void 
///////////////////////////////////////////////////////////////////////////////
///      >>>>>>>>>>>>>>>>>>>>>>>UPDATE<<<<<<<<<<<<<<<<<<<<<<<<<<<
void motor_pulse_calculations(float pid_output_left, float pid_output_right) {
  int left_motor;
  int right_motor;
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)
    pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)
    pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)
    pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)
    pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)
    left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)
    left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)
    right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)
    right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;  
}

