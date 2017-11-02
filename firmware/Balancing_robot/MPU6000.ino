static long gyro_yaw_calibration_value;
static long gyro_pitch_calibration_value;
static int acc_calibration_value = -600;


///////////////////////////////////////////////////////////////////////////////
/// \brief Generate default bias values for the gyro pitch and yaw based on 500 
/// initial samples.
///
/// \param[in] gyro_address I2C address of the gyro
/// \param[out] gyro_pitch_calibration_value 
/// \param[out] gyro_yaw_calibration_value
/// \return void
///////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(int gyro_address) {
  int receive_counter;
  pinMode(13, OUTPUT);                                                      //Configure digital port 6 as output
  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
}



///////////////////////////////////////////////////////////////////////////////
/// \brief Wakes the MPU-6050 gyro, which is initially in sleep mode.  Gyro set to
/// 250 degrees/sec full scale. Accelerometer set to +/-4g full scale
///
/// \param[in] I2C address of the gyro
/// \return void
///////////////////////////////////////////////////////////////////////////////
void wake_gyro(int gyro_address) {
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


///////////////////////////////////////////////////////////////////////////////
/// \brief Read current pitch and yaw data from the MPU-6050
///
/// \param[in] gyro_address I2C address of the gyro
/// \param[out] raw pitch data from the gyro
/// \param[out] raw yaw data from the gyro
/// \return void
///////////////////////////////////////////////////////////////////////////////
void retrieve_gyro_pitch_and_yaw(int gyro_address, int& gyro_pitch_data_raw, int& gyro_yaw_data_raw){
  // This function retrieves the gyroscope measurements for the "x" and "y" axes
  // seems like the "x" axis corresponds to the yaw and the "y" axis corresponds to the pitch
  // Note that the orientation of the gyro when the bot is vertical places the gyros "y" axis vertical wrt the floor/ground
  // Note that the orientation of the gyro when the bot is vertical places the gyros "x" axis out the left side of the bot
  // Having initialized the gyro to +/- 250deg/sec mode, the returned values will range from +/- 131
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 0x43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = (Wire.read()<<8)|Wire.read();                           //Combine the two bytes to make one integer
  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  gyro_pitch_data_raw = (Wire.read()<<8)|Wire.read();                         //Combine the two bytes to make one integer
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
}


///////////////////////////////////////////////////////////////////////////////
/// \brief Read current acceleration from the MPU-6050
///
/// \param[in] gyro_address I2C address of the gyro
/// \return raw accelerator data
///////////////////////////////////////////////////////////////////////////////
int retrieve_gyro_acceleration(int gyro_address){
  int accelerometer_data_raw;
  // This function retrieves the "Z" acceleration from gyro registers 0x3F through 0x40
  // Note that the orientation of the gyro when the bot is vertical places the gyros "z" axis horizontal to the floor/ground
  // Having initialized the gyro to +/-4g mode, the returned values will range from +/- 8192
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = (Wire.read()<<8)|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;
  return accelerometer_data_raw;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Calculates the tilt angle from the accelerometer and the gyro 
/// nRF24L01 radio. 
///
/// \param[in] gyro_address I2C address of the gyro
/// \return tilt angle 
///////////////////////////////////////////////////////////////////////////////
///      >>>>>>>>>>>>>>>>>>>>>>>UPDATE<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_angle_gyro(int gyro_address) {
  const float angle_gyro;
  float angle_acc;
  int accelerometer_data_raw, gyro_pitch_data_raw, gyro_yaw_data_raw;
  //Angle calculations
  accelerometer_data_raw = retrieve_gyro_acceleration(gyro_address);                        //acc data is limited to +/-8200
  angle_acc = asin((float)accelerometer_data_raw/8200.0)* DEGREEPERRADIAN;  //Calculate the current angle in degrees [-90,+90] according to the accelerometer
  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  DEBUG_PRINT("raw data is " + accelerometer_data_raw);
  DEBUG_PRINT(String("Angle is ").concat(angle_acc));
  retrieve_gyro_pitch_and_yaw(gyro_address, gyro_pitch_data_raw, gyro_yaw_data_raw);
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the angle traveled during this loop and add this to the angle_gyro variable
                                                                            // 0.000031 == (1/131 deg/sec/LSB) * 0.004 sec/loop = 0.00003053
  
  //MPU-6050 offset compensation
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle
  return angle_gyro;
}


