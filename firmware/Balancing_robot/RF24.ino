

#include "RF24.h"   //Don't know where this is coming from, but it's for the radio


///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
///////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// \brief initializes the RF24 radio transceiver over the SPI bus
///
/// \param[in] SPI address of radio
/// \return void
///////////////////////////////////////////////////////////////////////////////
void initialize_radio(byte radio_address) {
  radio.begin();                                                            //Start the SPI nRF24L01 radio
  radio.openReadingPipe(1,radio_address);                                   // Open a reading pipe on the radio
  radio.startListening();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Retrieves a radio command from the wii nunchuck over the 
/// nRF24L01 radio. 
///
/// \return radio data from the wii nunchuck 
///////////////////////////////////////////////////////////////////////////////
byte get_radio_data(void) {
  static byte received_byte;
  static int receive_counter = 0;
  // receive data from the radio (i.e. from the remote controller)
  if(radio.available()){
    radio.read(&received_byte,sizeof(received_byte));
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  if(receive_counter <= 25)
    receive_counter ++;                                                     //The received byte will be valid for 25 program loops (100 milliseconds)
  else {
    received_byte = 0x00;                                                   //After 100 milliseconds the received byte is deleted
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  return received_byte;  
}

