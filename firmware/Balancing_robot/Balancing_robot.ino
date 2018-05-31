
/////////////////////////////////////////////////////////////////////////////////////
/// \file Balancing_robot.ino
/// \brief Control code for two wheeled balancing robot
///
/////////////////////////////////////////////////////////////////////////////////////


// TODO: see if something other than D13 can be used for low battery light
// TODO: adjust the voltage divider math for new diodes and resistors
// TODO: check if radio should be 5V or 3.3V
// TODO:  do we really need to set TWBR when initallizing I2C?  The library seems to do it already.
// TODO: turn left, turn right seems messed up.  Verify codes with remote
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

#include "radio.h"
#include "MPU6000.h"
#include "motor.h"

#define DEBUG
#include "DebugUtils.h"

static byte radio_address[] = "1robt";




// User Config for the gyro
////////////////////////////////////////////////////////
///  \brief MPU-6050 I2C address (0x68 or 0x69).
////////////////////////////////////////////////////////
#define gyro_address 0x68


////////////////////////////////////////////////////////
///  \brief Gain setting for the P-controller (15).
////////////////////////////////////////////////////////
#define pid_p_gain 15                                
////////////////////////////////////////////////////////
///  \brief Gain setting for the I-controller (1.5).
////////////////////////////////////////////////////////
#define pid_i_gain 1.5                                      
////////////////////////////////////////////////////////
///  \brief Gain setting for the D-controller (30).
////////////////////////////////////////////////////////
#define pid_d_gain 30                                      


static unsigned long loop_timer;


static float self_balance_pid_setpoint;
static float pid_error_temp;
static float pid_i_mem;
static float gyro_input;
static float pid_output;
static float pid_last_d_error;


//Define the states of the machine
#define RESETTING   0
#define CONTROLLING 1
#define LOW_BAT     2
#define TILTED      3
#define CHECK_ANGLE 4
uint16_t fsm_state = RESETTING;



///////////////////////////////////////////////////////////////////////////////
/// \brief Initializes the serial link, the I2C bus, and the gyro (on the I2C bus).  Subsequently
/// calibrates the gyro, initializes the main timer, initializes the stepper motor control
/// lines, and finally initializes the wireless radio link.
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
void setup(){
  initialize_serial(9600);
  initialize_I2C();  // the I2C bus is used to communicate with the gyro
  wake_gyro(gyro_address);
  calibrate_gyro(gyro_address);
  initialize_timer();  //timer will be used to send pulse trains to the stepper motors
  
  // initialize pins used for stepper motor control
  pinMode(2, OUTPUT);                                                       //Configure digital port 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital port 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital port 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital port 5 as output
  
   // initializing radio at the end of "setup", as it uses the same pin (D13) as the led
  initialize_radio(radio_address);
 
  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time
  DEBUG_PRINT("initialized . . .");
}



///////////////////////////////////////////////////////////////////////////////
/// \brief Main program loop
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
void loop(){
  float pid_setpoint;
  float pid_output_left;
  float pid_output_right;
  float angle_gyro;
  byte received_byte, low_bat;
  byte start;

  //state machine
  switch (fsm_state)
  {
    case LOW_BAT:
      digitalWrite(13, HIGH);   //Turn on the led if battery voltage is to low
      resetPID();               //power down motors
      break;
    case TILTED:
      resetPID();               //power down motors
      fsm_state = RESETTING;
      break;
    case RESETTING:
      if (!battery_voltage_is_acceptable()){
        fsm_state = LOW_BAT;
        break;
      }
      if(initialize_angle(gyro_address)) {
        fsm_state = CHECK_ANGLE;
      }
      break;
    case CHECK_ANGLE:
      angle_gyro = get_angle_gyro(gyro_address);
      if (!battery_voltage_is_acceptable()){
        fsm_state = LOW_BAT;
        break;
      }
      if(angle_gyro > 30 || angle_gyro < -30) {   //If the robot tips over
        fsm_state = TILTED;
        break;
      }
      else {
        fsm_state = CONTROLLING;
        break;        
      }
      break;      
    case CONTROLLING:
      received_byte = get_radio_data();
      //PID controller calculations
      //The balancing robot is angle driven. First the difference between the desired angle (setpoint) and actual angle (process value)
      //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
      //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
      pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
      if(pid_output > 10 || pid_output < -10) {
        pid_error_temp += pid_output * 0.015 ;    
      }
      pid_output = PID_calculation(pid_error_temp, low_bat);
      control_calculations(received_byte, self_balance_pid_setpoint, pid_output, pid_setpoint, pid_output_left, pid_output_right);
      motor_pulse_calculations(pid_output_left, pid_output_right);
      break;
    default:
      break;
  }
  //Loop time timer
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Determines the battery supply voltage based on ADC measured voltage 
/// from a resistor divider.
///
/// \details The voltage on Analog Pin A0 is formed by a resistor divider of 3.3k
/// and 2.2k ohms.  The measured voltage is 1/2.5 times the supply voltage.
/// Thus a supply voltage of 5V*2.5 == 12.5V == 1023 on analogRead(0).  Multiplying
/// the analogRead value by 12.5/1023*1000 == 12.22 results the supply voltage in
/// millivolts.  The voltage drop across the diode is roughly 0.85V, so the battery 
/// voltage in millivolts can be determined by (analogRead(0) * 12.22) + 850
/// Acceptable range of voltage for a LiPo battery is between 10.5V and 8.0V,
/// or 10500 mV and 8000 mV.
/// 
/// \return void
///////////////////////////////////////////////////////////////////////////////
bool battery_voltage_is_acceptable(void) {
  int battery_mV;
  const int lowVoltage = 8000;
  const int highVoltage = 10500;
  const int diodeDrop = 850;
  const int voltageScaler = 12.22;
  battery_mV = (analogRead(0) * voltageScaler) + diodeDrop;
  if(battery_mV < highVoltage && battery_mV > lowVoltage) {
    return true;
  } 
  else {
    return false;
  }
}


///////////////////////////////////////////////////////////////////////////////
/// \brief Performs PID calculations given the pid error 
///
/// \param[in] pid_error_temp error value to be used in PID calculations
/// \param[in] low_bat low battery indicator
///
/// \return tilt angle 
///////////////////////////////////////////////////////////////////////////////
///      >>>>>>>>>>>>>>>>>>>>>>>UPDATE<<<<<<<<<<<<<<<<<<<<<<<<<<<
float PID_calculation(float pid_error_temp, byte low_bat) {
  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400) {
    pid_i_mem = 400;                                                        //Limit the I-controller to the maximum controller output
  }
  else if (pid_i_mem < -400) {
    pid_i_mem = -400;
  }
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400) {
    pid_output = 400;                                                       //Limit the PI-controller to the maximum controller output
  }
  else if(pid_output < -400) {
    pid_output = -400;
  }

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5) {
    pid_output = 0;                                                         //Create a dead-band to stop the motors when the robot is balanced
  }
  return pid_output;  
}

float resetPID() {
  pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
  pid_i_mem = 0;                                                          //Reset the I-controller memory
  self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  return pid_output;  
}






// >>>>>> Initialization routines <<<<<<<<<<


  
///////////////////////////////////////////////////////////////////////////////
/// \brief initializes the I2C bus, including a short delay to let the link settle.
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
void initialize_I2C() {
  Wire.begin();
  //TWBR = 12;                                                                
  delay(20);
}


///////////////////////////////////////////////////////////////////////////////
/// \brief Initialize a 20us interrupt based on TIMER2
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
void initialize_timer(){
  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
}


///////////////////////////////////////////////////////////////////////////////
/// \brief Initialize serial port to 9600 baud. 
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
void initialize_serial(int baudRate){
  Serial.begin(baudRate);                                               //Start the serial port at 9600 kbps
  //printf_begin();
}
























