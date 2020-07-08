/////////////////////////////////////////////////
// First, some magic numbers:
#define DEGREES_PER_RADIAN 57.296
#define GYRO_DATA_ADDRESS 0x43
#define GYRO_DATA_LENGTH 4
#define ACCEL_DATA_ADDRESS 0x3F
#define ACCEL_DATA_LENGTH 2
#define ACCEL_SENSITIVITY 8192.0   // LSB/g

// (0.004sec/cycle) / (131 LSB/deg/sec)] = 0.0000305 deg/cycle/LSB
#define GYRO_CONVERSION ((loop_time_us*1.E-6)/131.)
/////////////////////////////////////////////////


void initialize_gyro() {
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro
}

void calibrate_gyro() {
  for (int receive_counter = 0; receive_counter < 500; receive_counter++) {     //Create 500 loops
    if (receive_counter % 15 == 0)digitalWrite(LED_PIN, !digitalRead(LED_PIN));       //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 4 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();           //Combine two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();         //Combine two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
}
// The gyro is mounted on the bot with the chip on it's
// side and the "dot" on the front, lower, left corner.
// This places the +X axis extending vertically from the 
// top of the frame, and +Y extending to the right of the
// frame (orientation based on looking out from the front
// of the bot).  Rotating forward results in a -Y (pitch) rotation angle
// and turning right results in a -X (yaw) rotation angle.

void get_gyro_angle(float angle_acc, float &angle_gyro) {
  float gyro_yaw_data_raw;
  float gyro_pitch_data_raw;
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(GYRO_DATA_ADDRESS);                                            //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, GYRO_DATA_LENGTH);                         //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();                       //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();                     //Combine the two bytes to make one integer

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * GYRO_CONVERSION;                      //Calculate the angle traveled during this loop and add this to the angle_gyro variable

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board.
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;            //Correct the drift of the gyro angle with the accelerometer angle
}

// calculating an acceleropmeter angle
// The accelerometer is mounted on the bot with the chip on it's
// side and the "dot" on the front, lower, left corner.
// This places the +Z axis extending from the front of the frame, 
// and a forward tilt results in an gravitational acceleration 
// in +Z while a backward tilt results in a gravitational 
// acceleration in the -Z direction.
//
// The accelerometer has a sensitivity of 8192 LSB/g.
// An acceleration of +1g gives a return value of +8192, and
// an angle of asin(1) -> pi/2 radians (+90 degrees).
// An acceleration of 0g gives a return value of 0, and
// an angle of asin(0) -> 0 radians (0 degrees).
// An acceleration of -1g gives a return value of -1, and 
// an angle of asin(-1) -> -pi/2 radians (-90 degrees).
float get_accelerometer_angle() {
  float accelerometer_data_raw;
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(ACCEL_DATA_ADDRESS);                                           //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, ACCEL_DATA_LENGTH);                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();                  //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;
  // note: 57.296 is degrees/radian
  return asin((float)accelerometer_data_raw / ACCEL_SENSITIVITY) * DEGREES_PER_RADIAN;        //Calculate the current angle according to the accelerometer
}

