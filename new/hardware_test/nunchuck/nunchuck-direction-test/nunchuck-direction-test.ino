///////////////////////////////////////////////////////////////////////////////////////
//   Wii Nunchuck test for the Balancing Robot
//
//   This routine is designed to test the I2C interface of a Wii Nunchuck.
//   Note that
//   the wiring colors may vary from what is documented here, depending on the hardware
//   vendor.
//   
//   Nominally the values sent by the nunchuck for X qnd Y will vary from 0 to 255,
//   left to right (X) and reverse to forward (Y)
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
//   Arduino Wiring:
//   Assuming UNO (other boards may vary):
//   UNO        Nunchuck
//   Analog 4 = SDA (4.7K pullup not required)
//   Analog 5 = SCL (4.7K pullup not required)
//   GND      = GND
//   +3.3V    = VCC
//
//
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <printf.h>

#define NUNCHUCK_ADDR 0x52
#define LOW_DEADZONE 80
#define HIGH_DEADZONE 170

byte error, nunchuck_found, lowByte, highByte;
int address;
int nDevices;
int x, y;

void initialize_serial(){
    Serial.begin(9600);                                               //Start the serial port at 9600 kbps
    printf_begin();
}

void initialize_I2C() {
    Wire.begin();                                                             //Start the I2C bus as master
    //TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
    delay(20);                                                                //Short delay
}

void setup()
{
  initialize_I2C();
  initialize_serial();
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
    Serial.println("Printing Nunchuck Move Directions");
    while(1)
    {
      Wire.beginTransmission(NUNCHUCK_ADDR);
      Wire.write(0x00);                     // reset the address to 0x00
      Wire.endTransmission();
      Wire.requestFrom(NUNCHUCK_ADDR,2);    // request two bytes from the nunchuck
      while(Wire.available() < 2);          // wait for at least 2 bytes to be available
      x = Wire.read();                      // x position, 0 to 255
      y = Wire.read();                      // y position, 0 to 255
      if (x < LOW_DEADZONE) {
        Serial.println("Left");
      }
      else if (x > HIGH_DEADZONE) {
        Serial.println("Right");        
      }
      else if (y < LOW_DEADZONE) {
        Serial.println("Reverse");        
      }
      else if (y > HIGH_DEADZONE) {
        Serial.println("Forward");        
      }
      delay(250);      
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
















