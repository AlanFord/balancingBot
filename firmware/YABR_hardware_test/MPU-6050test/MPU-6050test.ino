///////////////////////////////////////////////////////////////////////////////////////
//   MPU-6050 test for the Balancing Robot
//
//   This routine is designed to test for the MPU-6050 gyro embodied in the
//   breakout board Kootek GY-521 available on Amazon.
//   For a schematic see:
//   https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/MPU6050-V1-SCH-1024x599.jpg
//
//   Wiring:
//   Assuming UNO (other boards may vary):
//   UNO        MPU-6050
//   Analog 4 = SDA (4.7K pullup not required)
//   Analog 5 = SCL (4.7K pullup not required)
//   GND      = GND
//   +5V      = VCC
//
//   Expecting a response on address 0x68 (defined by MPU-650 pin AD0 floating)
//   Address can be changed to 0x69 by raising MPU-650 pin AD0 floating to logic high
//
//   Wire Library Tow Wire Interface Registers:
//   TWCR - Two Wire Control Register - Controls the actions of the TWI module
//   TWSR - Two Wire Status Register - Reports the status of the TWI actions
//   TWDR - Two Wire Data/Address Register - Contains transmit or received data
//   TWBR - Two Wire Bit Rate Register - Controls the frequency of the clock (SCL)
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "mpu6000.h"

byte error, MPU_6050_found, lowByte, highByte;
int address;
int nDevices;
int counter;

void setup()
{
  Wire.begin();
  // TODO - figure out what does TWBR does
  // Seems to jack up the SCL frequency by a factor of 4 on a 
  // 16 MHz UNO with 100kHz TWI (nominal) [has a default TWBR of 72]
  // see page 213 of http://www.atmel.com/images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet_Complete.pdf
  // Page 308 also lists max fscl as 400kHz
  // Also see notes in /Applications/Arduino.app/Contents/Java/hardware/arduino/avr/libraries/Wire/src/utility/twi.c
  // Could also be performed with the Wire.setClock() command. See
  // https://www.arduino.cc/en/Reference/WireSetClock
  // MPU-6050 datasheet indicates an I2C SCL frequency of 400 kHz (see page 33 of
  // https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf )
  //TWBR = 12;
  Wire.setClock(400000);
  Serial.begin(9600);
  Serial.print("TWBR value is ");
  Serial.println(TWBR);
}

void loop()
{
  Serial.println("Scanning I2C bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
      if(address == 0x68 || address == 0x69)  // the only two valid addresses for the MPU-6050
      {
        Serial.println("This could be a MPU-6050");
        Wire.beginTransmission(address);
        Wire.write(WHO_AM_I_MPU6050);
        Wire.endTransmission();
        Serial.println("Send Who am I request...");
        Wire.requestFrom(address, 1);
        while(Wire.available() < 1);
        lowByte = Wire.read();
        if(lowByte == 0x68)
        {
          Serial.print("Who Am I responce is ok: 0x");
          Serial.println(lowByte, HEX);
        }
        else
        {
          Serial.print("Wrong Who Am I responce: 0x");
          if (lowByte<16)Serial.print("0");
          Serial.println(lowByte, HEX);
        }
        if(lowByte == 0x68)
        {
          MPU_6050_found = 1;
          Serial.println("Starting Gyro....");
          set_gyro_registers();
        }
      }
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  if(MPU_6050_found)
  {
    Serial.print("Balance value: ");
    Wire.beginTransmission(address);
    Wire.write(ACCEL_ZOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(address,2);
    Serial.println((Wire.read()<<8|Wire.read())*-1);
    delay(20);
    Serial.println("Printing raw gyro values");
    for(counter = 0; counter < 20; counter++ )
    {
      Wire.beginTransmission(address);
      Wire.write(GYRO_XOUT_H);
      Wire.endTransmission();
      Wire.requestFrom(address,6);
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
  else Serial.println("No MPU-6050 device found at address 0x68 or 0x69");
  while(1);
}

///////////////////////////////////////////////////////////////////////////////////////
//   For register descriptions, see
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
///////////////////////////////////////////////////////////////////////////////////////

void set_gyro_registers(){
  //Setup the MPU-6050
  Wire.beginTransmission(address);                                     //Start communication with the address found during search.
  Wire.write(PWR_MGMT_1);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  
  Wire.beginTransmission(address);                                     //Start communication with the address found during search.
  Wire.write(GYRO_CONFIG);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(FSR_250_DEGS);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  Wire.beginTransmission(address);                                     //Start communication with the address found during search.
  Wire.write(ACCEL_CONFIG);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(FSR_4G);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  Wire.beginTransmission(address);                                     //Start communication with the address found during search
  Wire.write(CONFIG);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(DLPF_43HZ);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}









