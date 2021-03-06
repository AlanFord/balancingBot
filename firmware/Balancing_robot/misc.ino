/////////////////////////////////////////////////
// First, some magic numbers:
// For a 3S (3 cell) LIPO battery, a low-low voltage is 9V.  Hence
//   anything below 8V indicates no battery is attached (keep working in test mode).
// Recommended stop-and-charge voltage is 10.5V  
#define MIN_OPERATING_VOLTAGE 10500  //10.5V in mV
#define NO_BATTERY_ATTACHED_MAX_VOLTAGE 8000  //8V in mV

// Voltage Measurement Constants
//850 mV is the voltage compensation for the diode.
//Resistor voltage divider => 21.5k/(33k + 21.5k) = 1/2.535
//Arduino analog input A0 reads 0 at 0V and 1023 at 5V, 
// or 0.00489V/division
// so,
//voltage divider voltage = A0 * 0.00489V/division
//battery voltage = (A0 * 0.00489V/division)*2.535 + 0.85V
// or
//battery voltage = (A0 * 0.0124) + 0.85V
// in millivolts (integers make things easier)
// battery mV = (A0 *12.4) + 850
#define VOLTAGE_MULTIPLIER 12.4
#define VOLTAGE_BIAS 850

/////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
/// \brief Initialize the I2C bus and set the associated clock frequency.
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
void initialize_i2c() {
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
}

void initialize_timer() {
  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
}

bool battery_voltage_is_low() {
  int voltage;
  //Load the battery voltage to the voltage variable.
  //850 mV is the voltage compensation for the diode.
  //Resistor voltage divider => 21.5k/(33k + 21.5k) = 1/2.535
  //Arduino analog input A0 reads 0 at 0V and 1023 at 5V, 
  // or 0.00489V/division
  // so,
  //voltage divider voltage = A0 * 0.00489V/division
  //battery voltage = (A0 * 0.00489V/division)*2.535 + 0.85V
  // or
  //battery voltage = (A0 * 0.0124) + 0.85V
  // in millivolts (integers make things easier)
  // battery mV = (A0 *12.4) + 850
  voltage = (analogRead(0) * VOLTAGE_MULTIPLIER) + VOLTAGE_BIAS;
  if (voltage < MIN_OPERATING_VOLTAGE && voltage > NO_BATTERY_ATTACHED_MAX_VOLTAGE) { 
    return true;
  }
  return false;
}

