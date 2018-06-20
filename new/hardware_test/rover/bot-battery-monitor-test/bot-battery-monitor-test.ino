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

///////////////////////////////////////////////////////////////////////////////////////
// Initial Results:
// Power Supply     Arduino
//     (V)          (counts)
//      0             832   -- anomalous - don't use - probably driven by arduino voltage
//     9.06           668
//    10.09           752
//    11.09           832
//    12.07           912
// Resulting Curve
// counts = (Vbat * 81.1) - 66
// Vbat = (counts + 66) / 81.1
// At 0 counts, Vbat = 0.814 V.  This should be the diode drop.
// If the goal is to alarm at Vbat = 10.5 V, counts will be 786.
///////////////////////////////////////////////////////////////////////////////////////     

#include <printf.h>
int sum = 0;
int counter;

void setup()
{
  Serial.begin(9600);
 }

void loop() {
  for (counter=0; counter <10; counter++){
    sum = sum + analogRead(0);  
  }
  Serial.print(analogRead(0));
  Serial.print(", ");
  Serial.println(sum/10);
  sum = 0;
  delay(1000);
}
  















