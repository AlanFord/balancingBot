
/////////////////////////////////////////////////
// First, some magic numbers:
#define BUFFER_TIME 100  //milliseconds, length of time radio data is considered valid
#define BUFFER_CYCLES (BUFFER_TIME/(loop_time_us/1000)) // number of loop cycles in 1 BUFFER_TIME

///////////////////////////////////////////////////////////////////////////////
/// \brief initializes the RF24 radio transceiver over the SPI bus
///
/// \param[in] radio_address - SPI address of radio
/// \return void
///////////////////////////////////////////////////////////////////////////////
void initialize_radio() {
  radio.begin();                                                            //Start the SPI nRF24L01 radio
  radio.openReadingPipe(1, radio_address);                                  // Open a reading pipe on the radio
  radio.startListening();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Retrieves a radio command from the wii nunchuck over the
/// nRF24L01 radio. Sets the global variable receive_byte
///
/// \return byte
///////////////////////////////////////////////////////////////////////////////
byte get_radio_data() {
  static int receive_counter;
  static byte received_byte;
  // receive data from the radio (i.e. from the remote controller)
  if (radio.available()) {
    radio.read(&received_byte, sizeof(received_byte));
    receive_counter = 0;                             //Reset the receive_counter variable
  }
  if (receive_counter <= BUFFER_CYCLES)
    receive_counter ++;                              //The received byte will be valid 100 milliseconds
  else
    received_byte = 0x00;                            //After 100 milliseconds the received byte is deleted
  return received_byte;
}
