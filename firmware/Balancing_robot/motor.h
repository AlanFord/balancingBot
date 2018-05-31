/////////////////////////////////////////////////////////////////////////////////////
/// \file motor.h
/// \brief stepper motor driver header file
/////////////////////////////////////////////////////////////////////////////////////

#ifndef __MOTOR_H__
#define __MOTOR_H__


#include <Arduino.h>

ISR(TIMER2_COMPA_vect);
void control_calculations(byte received_byte, float& self_balance_pid_setpoint, float pid_output, float& pid_setpoint, float& pid_output_left, float& pid_output_right);
void motor_pulse_calculations(float pid_output_left, float pid_output_right);

///////////////////////////////////////////////////////////////////////////////////////
// Radio signal data for bot control
#define RADIO_NULL  B00000000
#define BOT_LEFT    B00000001
#define BOT_RIGHT   B00000010
#define BOT_FORWARD B00000100
#define BOT_REVERSE B00001000
#define BOT_STABLE  B00001100
///////////////////////////////////////////////////////////////////////////////////////


#endif // __MOTOR_H__



