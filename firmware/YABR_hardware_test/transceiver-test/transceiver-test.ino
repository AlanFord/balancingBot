///////////////////////////////////////////////////////////////////////////////////////
//   Transceiver test for the Balancing Robot
//
//   This routine is designed to test the SPI interface of a WRL-00691, a nRF24L01+ breakout.
//
//
//   WRL-00691 <---------------------> Arduino Wiring:
//
//   Assuming UNO (other boards may vary):
//   UNO                    WRL-00691
//   GND                    GND
//   Pin 8                  IRQ  (active low)
//   Pin 12 (MISO)          MISO (3.3V-5V SPI slave output)
//   Pin 11 (MOSI)          MOSI (3.3V-5V SPI slave input)
//   Pin 13 (SCK)           SCK  (3.3V-5V SPI clock)
//   Pin 10 (SS)            CSN  (3.3V-5V SPI chip select)
//   Pin 9                  CE   (3.3V-5V chip enable)
//   +3.3V                  VCC  (3.3V-7V)
//
//
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#define NUNCHUCK_ADDR 0x52

byte error, nunchuck_found, lowByte, highByte;
int address;
int nDevices;

void setup()
{
  Wire.begin();
  Wire.setClock(400000);  // set the I2C speed to 400kb/sec
  Serial.begin(9600);
  nunchuck_found = 0;
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
      if(address == NUNCHUCK_ADDR)
      {
        Serial.println("This could be a Nunchuck");
        Serial.println("Trying to initialise the device...");
        init_no_encryt();
        Serial.println("Sending joystick data request...");
        Wire.beginTransmission(NUNCHUCK_ADDR);
        Wire.write(0x00);                           // reset the address to 0x00
        Wire.endTransmission();
        Wire.requestFrom(NUNCHUCK_ADDR,1);          // request one byte from the nunchuck
        while(Wire.available() < 1);                // wait for at least 1 byte to be available
        lowByte = Wire.read();                      // read a byte
        if(lowByte > 100 && lowByte < 160)
        {
          Serial.print("Data response normal: ");
          Serial.println(lowByte);
          nunchuck_found = 1;
        }
        else
        {
          Serial.print("Data response is not normal: ");
          Serial.println(lowByte);
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
  if(nunchuck_found)
  {
    Serial.println("Printing raw Nunchuck values");
    for(address = 0; address < 20; address++ )
    { 
      Wire.beginTransmission(NUNCHUCK_ADDR);
      Wire.write(0x00);                     // reset the address to 0x00
      Wire.endTransmission();
      Wire.requestFrom(NUNCHUCK_ADDR,2);    // request two bytes from the nunchuck
      while(Wire.available() < 2);          // wait for at least 2 bytes to be available
      Serial.print("Joystick X = "); 
      Serial.print(Wire.read());            // read and print a byte
      Serial.print(" Joystick y = ");
      Serial.println(Wire.read());          // read and print a byte
      delay(100);
    }
  }
  else Serial.println("No Nunchuck device found at address 0x52");
  while(1);
}

void init_no_encryt()
{
  // Initialize the nunchuck without ecryption
  Wire.beginTransmission(NUNCHUCK_ADDR);
  Wire.write(0xF0);
  Wire.write(0x55);
  Wire.endTransmission();
  delay(20);
  Wire.beginTransmission(NUNCHUCK_ADDR);
  Wire.write(0xFB);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(20);
}
















