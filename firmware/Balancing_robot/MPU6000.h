/////////////////////////////////////////////////////////////////////////////////////
/// \file MPU6000.h
/// \brief MPU6050 gyro header file
/////////////////////////////////////////////////////////////////////////////////////


#ifndef __MPU6000_H__
#define __MPU6000_H__


#include <Arduino.h>
#include <Wire.h>   //Include the Wire.h library so we can communicate with the gyro


void  calibrate_gyro(int gyro_address);
void  wake_gyro(int gyro_address);
void  retrieve_gyro_pitch_and_yaw(int gyro_address, int& gyro_pitch_data_raw, int& gyro_yaw_data_raw);
int   retrieve_gyro_acceleration(int gyro_address);
bool  initialize_angle(int gyro_address);
float get_angle_gyro(int gyro_address);

////////////////////////////////////////////////////////
///  \brief conversion factor in Degrees/Radian
////////////////////////////////////////////////////////
#define DEGREEPERRADIAN  57.296


#endif // __MPU6000_H__

