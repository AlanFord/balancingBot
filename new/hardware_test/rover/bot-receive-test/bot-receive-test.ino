///////////////////////////////////////////////////////////////////////////////////////
//   Transceiver test for the Balancing Robot
//
//   This routine is designed to test the SPI interface of a WRL-00691, a nRF24L01+ breakout.
//
///////////////////////////////////////////////////////////////////////////////////////
// Wiring Configuration for the Build,RVA Version
// Arduinio Pro Mini
//
// Arduino                      Motor Driver
// D2                           Left Driver (Step??)
// D3                           Left Driver (Direction??)
// D4                           Right Driver (Step??)
// D5                           Right Driver (Direction??)
//
// Arduino                      Voltage Divider
// A0
//
// Arduino                      Voltage Regulator
// ACC                          +5V
// GND                          GND
//
// Arduino                      Gyro
// A4                           SDA
// A5                           SCL
// GND                          GND
// +5V                          VCC
//
// Arduino                      Radio
// D9                           CE
// D10                          CSN
// D11                          MOSI
// D12                          MISO
// D13                          SCK
// +3.3V                        VCC
// GND                          GND
///////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>    //Include the SPI.h library for the radio
#include "RF24.h"   //Don't know where this is coming from, but it's for the radio
#include <printf.h>

///////////////////////////////////////////////////////////////////////////////////////
// Radio signal data for bot control
#define RADIO_NULL  B00000000
#define BOT_LEFT    B00000001
#define BOT_RIGHT   B00000010
#define BOT_FORWARD B00000100
#define BOT_REVERSE B00001000
#define BOT_STABLE  B00001100
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
byte radio_address[] = "1robt";
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declaring global variables
byte received_byte;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);

  // initializing radio at the end of "setup", as it uses the same pin (D13) as the led
  initialize_radio();
 }

void loop() {
  // receive data from the radio (i.e. from the remote controller)
  if(radio.available()){
    received_byte = 0;
    radio.read(&received_byte,sizeof(received_byte));
    if(received_byte & BOT_LEFT) {                                            
      Serial.println("Turning Left");
    }
    if(received_byte & BOT_RIGHT) {                                           
      Serial.println("Turning Right");
    }
    if(received_byte == BOT_FORWARD) {                                         
      Serial.println("Go Forward");
    }
    if(received_byte == BOT_REVERSE) {                                         
      Serial.println("Go Backward");
    }   
    if((received_byte == BOT_STABLE)) {                                       
      Serial.println("Bot Stable");
    }
  }
}

void initialize_radio() {
  radio.begin();                                                            //Start the SPI nRF24L01 radio
  radio.openReadingPipe(1,radio_address);                                   // Open a reading pipe on the radio
  radio.startListening();
}
  















