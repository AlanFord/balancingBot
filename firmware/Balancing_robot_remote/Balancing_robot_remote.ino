// TODO:  check if CC should be 5V or 3V for the Radio
///////////////////////////////////////////////////////////////////////////////////////
// Wiring Configuration for the Build,RVA Version
// Arduinio UNO
//
// Arduino                      Nunchuck (Your wire colors may be different)
// A4                           SDA (yellow)
// A5                           SCL (white)
// GND                          GND (red)
// +5V                          VCC (green)
//
// Arduino                      Radio
// D9                           CE
// D10                          CSN
// D11                          MOSI
// D12                          MISO
// D13                          SCK
// +3.3V                        VCC  (or is it 5V??)
// GND                          GND
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>   //Include the Wire.h library so we can communicate with the Nunchuck
#include <SPI.h>    //Include the SPI.h library for the radio
#include "RF24.h"   //Don't know where this is coming from, but it's for the radio

///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
byte radio_address[] = "1Node";
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
// User Config for the Nunchuck
int nunchuk_address = 0x52;                                         //Nunchuk I2C address (0x52)
byte received_data[6], send_byte;                                   //Declare some global byte variables
///////////////////////////////////////////////////////////////////////////////////////

void setup(){
  // Serial.begin(9600);                                               //Start the serial port at 9600 kbps
  radio.begin();                                                    //Start the SPI nRF24L01 radio
  radio.openWritingPipe(radio_address);                             // Open a writing pipe on the radio
  
  Wire.begin();                                                     //Start the I2C as master
  TWBR = 12;                                                        //Set the I2C clock speed to 400kHz
  Wire.begin();                                                     //Start the I2C bus as master
  delay(20);                                                        //Short delay
  Wire.beginTransmission(nunchuk_address);                          //Start communication with the Nunchuck
  Wire.write(0xF0);                                                 //We want to write to register (F0 hex)
  Wire.write(0x55);                                                 //Set the register bits as 01010101
  Wire.endTransmission();                                           //End the transmission
  delay(20);                                                        //Short delay
  Wire.beginTransmission(nunchuk_address);                          //Start communication with the Nunchuck
  Wire.write(0xFB);                                                 //We want to write to register (FB hex)
  Wire.write(0x00);                                                 //Set the register bits as 00000000
  Wire.endTransmission();                                           //End the transmission
  delay(20);                                                        //Short delay
}

void loop(){
  Wire.beginTransmission(nunchuk_address);                          //Start communication with the Nunchuck.
  Wire.write(0x00);                                                 //We want to start reading at register (00 hex)
  Wire.endTransmission();                                           //End the transmission
  Wire.requestFrom(nunchuk_address,6);                              //Request 6 bytes from the Nunchuck
  for(byte i = 0; i < 6; i++) received_data[i] = Wire.read();       //Copy the bytes to the received_data array
  send_byte = B00000000;                                            //Set the send_byte variable to 0
  if(received_data[0] < 80)send_byte |= B00000001;                  //If the variable received_data[0] is smaller then 80 set bit 0 of the send byte variable
  if(received_data[0] > 170)send_byte |= B00000010;                 //If the variable received_data[0] is larger then 170 set bit 1 of the send byte variable
  if(received_data[1] < 80)send_byte |= B00001000;                  //If the variable received_data[1] is smaller then 80 set bit 3 of the send byte variable
  if(received_data[1] > 170)send_byte |= B00000100;                 //If the variable received_data[1] is larger then 170 set bit 2 of the send byte variable
  //if(send_byte)Serial.print((char)send_byte);                       //Send the send_byte variable if it's value is larger then 0
  if(send_byte)radio.write(&send_byte,sizeof(send_byte));           //Send the send_byte variable if it's value is larger then 0
  delay(40);                                                        //Create a 40 millisecond loop delay
}



