///////////////////////////////////////////////////////////////////////////////////////
//   MPU-6050 test for the Balancing Robot
//
//   This routine is designed to test for the MPU-6050 gyro embodied in the
//   breakout board Kootek GY-521 available on Amazon.
//   For a schematic see:
//   https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/MPU6050-V1-SCH-1024x599.jpg
//   Datasheets:
//   https://playground.arduino.cc/Main/MPU-6050
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
//
//   The MPU-6050 can be accessed with I2C at 400 kHz.
//   I2C address defined by pin9 (AD0) on the MPU-6050 package.
//   AD0=0 (GND), 0b1101000 or 0x68
//   AD0=1 (VDD), 0b1101001 or 0x69
//
///////////////////////////////////////////////////////////////////////////////////////
// Wiring Configuration for the Build,RVA Version
// Arduinio Pro Mini
//
// Arduino                      Gyro
// A4                           SDA
// A5                           SCL
// GND                          GND
// +5V                          VCC
//
///////////////////////////////////////////////////////////////////////////////////////
//   Expecting a response on address 0x68 (defined by MPU-650 pin AD0 floating)
//   Address can be changed to 0x69 by raising MPU-650 pin AD0 floating to logic high
//
//   Wire Library Two Wire Interface Registers:
//   TWCR - Two Wire Control Register - Controls the actions of the TWI module
//   TWSR - Two Wire Status Register - Reports the status of the TWI actions
//   TWDR - Two Wire Data/Address Register - Contains transmit or received data
//   TWBR - Two Wire Bit Rate Register - Controls the frequency of the clock (SCL)
///////////////////////////////////////////////////////////////////////////////////////
#define ADO 0
#include <Wire.h>
#include "mpu6000.h"
#include <printf.h>

int counter;
int receive_counter;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value,accel_calibration_value;

void setup()
{
  initialize_serial();
  initialize_I2C();                       // the I2C bus is used to communicate with the gyro
  wake_gyro();
  calibrate_gyro();
  Serial.print("TWBR value is ");
  Serial.println(TWBR);
}

void loop()
{
  Serial.print("Balance value: ");
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS,2);
  Serial.println((Wire.read()<<8|Wire.read())*-1);
  delay(20);
  Serial.println("Printing raw gyro values");
  for(counter = 0; counter < 20; counter++ )
  {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS,6);
    while(Wire.available() < 6);
    Serial.print("Gyro X = "); 
    Serial.print(Wire.read()<<8|Wire.read());
    Serial.print(" Gyro Y = "); 
    Serial.print(Wire.read()<<8|Wire.read());
    Serial.print(" Gyro Z = "); 
    Serial.println(Wire.read()<<8|Wire.read());
  }
  Serial.println("");
}

///////////////////////////////////////////////////////////////////////////////////////
//   For register descriptions, see
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
///////////////////////////////////////////////////////////////////////////////////////
void wake_gyro() {
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(MPU6050_ADDRESS);                                  //Start communication with the address found during search.
  Wire.write(PWR_MGMT_1);                                                   //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x01);                                                         //Set the gyro clock to use the PLL with X axis gyroscope reference 
  Wire.endTransmission();                                                   //End the transmission with the gyro.

  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(MPU6050_ADDRESS);                                  //Start communication with the address found during search.
  Wire.write(GYRO_CONFIG);                                                  //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(FSR_250_DEGS);                                                 //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(MPU6050_ADDRESS);                                  //Start communication with the address found during search.
  Wire.write(ACCEL_CONFIG);                                                 //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(FSR_4G);                                                       //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  //Set some filtering to improve the raw data.
  Wire.beginTransmission(MPU6050_ADDRESS);                                  //Start communication with the address found during search
  Wire.write(CONFIG);                                                       //We want to write to the CONFIG register (1A hex)
  Wire.write(DLPF_43HZ);                                                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}


void calibrate_gyro() {
  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    Wire.beginTransmission(MPU6050_ADDRESS);                                //Start communication with the gyro
    Wire.write(GYRO_XOUT_H);                                                //Start reading with the X gyro register
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(MPU6050_ADDRESS, 4);                                   //Request 4 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer (X gyro)
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer (Y gyro)
    
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCEL_ZOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS,2);
    accel_calibration_value += Wire.read()<<8|Wire.read();

    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
  accel_calibration_value /= 500;                                           //Divide the total value by 500 to get the avarage acceleration
}


void initialize_serial(){
  Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
  printf_begin();
}
  
void initialize_I2C() {
  Wire.begin();                                                             //Start the I2C bus as master
  Wire.setClock(400000);                                                    // Set to "fast" mode, 400kHz
  delay(20);                                                                //Short delay
}









