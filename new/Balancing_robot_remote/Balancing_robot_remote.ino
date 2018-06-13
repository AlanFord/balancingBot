// TODO:  why read 6 bytes from the nunchuck when 4 will do?
// TODO:  do we really need to set TWBR when initallizing I2C?  The library seems to do it already.
// TODO:  implement BOT_STABLE
///////////////////////////////////////////////////////////////////////////////////////
// Wiring Configuration for the Build,RVA Version
// Arduinio UNO
//
// Arduino                      Nunchuck (Your wire colors may be different)
// A4                           SDA (yellow)
// A5                           SCL (white)
// GND                          GND (red)
// +3.3V                        VCC (green)
//
// Arduino                      Radio
// D9                           CE
// D10                          CSN
// D11                          MOSI
// D12                          MISO
// D13                          SCK
// +5V                          VCC
// GND                          GND
//
///////////////////////////////////////////////////////////////////////////////////////
//   Nunchuck Wiring
//   Note that the wiring colors may vary from what is documented here,
//   depending on the hardware vendor.
//
//   Nunchuck Wiring (looking into the nunchuck connector):
//
//   white(scl)                   red(gnd)
//   -----------                 -------------
//   |         |                 |           |
//   |         | ----------------|           |
//   |                                       |
//   |---------------------------------------|
//   green(vcc)   black(detect)   yellow(sda)
//
//   I2C data: see http://dangerousprototypes.com/blog/2009/08/19/bus-pirate-wii-nunchuck-quick-guide/
//   http://robotshop.com/letsmakerobots/wii-nunchuckarduino-tutorial
//   https://create.arduino.cc/projecthub/infusion/using-a-wii-nunchuk-with-arduino-597254
//
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>   //Include the Wire.h library so we can communicate with the Nunchuck
#include <SPI.h>    //Include the SPI.h library for the radio
#include "RF24.h"   //Don't know where this is coming from, but it's for the radio
#include <printf.h>

///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
byte radio_address[] = "1robt";         // this is the "pipe" we will be using
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
// User Config for the Nunchuck
#define NUNCHUCK_ADDR 0x52                            //Nunchuk I2C address (0x52)
#define LOW_DEADZONE 80
#define HIGH_DEADZONE 170
byte received_data[6], send_byte;                                   //Declare some global byte variables
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
// Radio signal data for bot control
#define RADIO_NULL  B00000000
#define BOT_LEFT    B00000001
#define BOT_RIGHT   B00000010
#define BOT_FORWARD B00000100
#define BOT_REVERSE B00001000
#define BOT_STABLE  B00001100
///////////////////////////////////////////////////////////////////////////////////////


void setup(){
  initialize_serial();
  initialize_radio();
  initialize_I2C();
  wake_nunchuck();
}

void loop(){
  bool status;
  Wire.beginTransmission(NUNCHUCK_ADDR  );                          //Start communication with the Nunchuck.
  Wire.write(0x00);                                                 //We want to start reading at register (00 hex)
  Wire.endTransmission();                                           //End the transmission
  Wire.requestFrom(NUNCHUCK_ADDR,6);                                //Request 6 bytes from the Nunchuck
  for(byte i = 0; i < 6; i++) received_data[i] = Wire.read();       //Copy the bytes to the received_data array
  send_byte = RADIO_NULL;                                           //Set the send_byte variable to 0
  if(received_data[0] < LOW_DEADZONE)                               //Turn left
    send_byte |= BOT_LEFT;                                          //If the variable received_data[0] is smaller then 80 set bit 0 of the send byte variable
  if(received_data[0] > HIGH_DEADZONE)                              //Turn right
    send_byte |= BOT_RIGHT;                                         //If the variable received_data[0] is larger then 170 set bit 1 of the send byte variable
  if(received_data[1] < LOW_DEADZONE)                               //Roll backwards
    send_byte |= BOT_REVERSE;                                       //If the variable received_data[1] is smaller then 80 set bit 3 of the send byte variable
  if(received_data[1] > HIGH_DEADZONE)                              //Roll forward
    send_byte |= BOT_FORWARD;                                       //If the variable received_data[1] is larger then 170 set bit 2 of the send byte variable
  if (send_byte)
  {
    radio.write(&send_byte,sizeof(send_byte));                       //Send the send_byte variable if it's value is larger then 0
  }
  delay(40);                                                        //Create a 40 millisecond loop delay
}

void initialize_radio() {
  if (!radio.begin())                                               //Start the SPI nRF24L01 radio
  {
    Serial.println("Error: Radio failed to initialize");                                                    
  }
  radio.stopListening();                                            // prepare to open a pipe for writing
  radio.openWritingPipe(radio_address);                             // Open a writing pipe on the radio
}

void initialize_serial(){
  Serial.begin(9600);                                               //Start the serial port at 9600 kbps
  printf_begin();
}

void initialize_I2C() {
  Wire.begin();                                                             //Start the I2C bus as master
  //TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
  delay(20);                                                                //Short delay
}

void wake_nunchuck(){
  Wire.beginTransmission(NUNCHUCK_ADDR);                          //Start communication with the Nunchuck
  Wire.write(0xF0);                                                 //We want to write to register (F0 hex)
  Wire.write(0x55);                                                 //Set the register bits as 01010101
  Wire.endTransmission();                                           //End the transmission
  delay(20);                                                        //Short delay
  Wire.beginTransmission(NUNCHUCK_ADDR);                          //Start communication with the Nunchuck
  Wire.write(0xFB);                                                 //We want to write to register (FB hex)
  Wire.write(0x00);                                                 //Set the register bits as 00000000
  Wire.endTransmission();                                           //End the transmission
  delay(20);                                                        //Short delay  
}



