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

#include <Wire.h>   //Include the Wire.h library so we can communicate with the gyro
#include <SPI.h>    //Include the SPI.h library for the radio
//#include <printf.h>

#define DEBUG
#include "DebugUtils.h"

static byte radio_address[] = "1robt";


////////////////////////////////////////////////////////
///  \brief conversion factor in Degrees/Radian
////////////////////////////////////////////////////////
const double DEGREEPERRADIAN  = 57.296;


// User Config for the gyro
////////////////////////////////////////////////////////
///  \brief MPU-6050 I2C address (0x68 or 0x69).
////////////////////////////////////////////////////////
const int gyro_address = 0x68;


////////////////////////////////////////////////////////
///  \brief Gain setting for the P-controller (15).
////////////////////////////////////////////////////////
const float pid_p_gain = 15;                                 
////////////////////////////////////////////////////////
///  \brief Gain setting for the I-controller (1.5).
////////////////////////////////////////////////////////
const float pid_i_gain = 1.5;                                      
////////////////////////////////////////////////////////
///  \brief Gain setting for the D-controller (30).
////////////////////////////////////////////////////////
const float pid_d_gain = 30;                                       
////////////////////////////////////////////////////////
///  \brief Turning speed (20).
////////////////////////////////////////////////////////
const float turning_speed = 30;                                   
////////////////////////////////////////////////////////
///  \brief Max target speed (100).
////////////////////////////////////////////////////////
const float max_target_speed = 150;                                


unsigned long loop_timer;


int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
float angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
//byte start;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
  float angle_gyro;
  byte received_byte, low_bat;
  byte start;

  if (!battery_voltage_is_acceptable()){
    digitalWrite(13, HIGH);                                                 //Turn on the led if battery voltage is to low
    low_bat = 1;                                                            //Set the low_bat variable to 1    
  }
  //check_battery_voltage();  // WARNING- may conflict with the SPI SCK function!
  
  received_byte = get_radio_data();

  angle_gyro = get_angle_gyro(gyro_address);
  
  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1) {   //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = resetPID();
    start = 0;                                                              //Set the start variable to 0
  }
  else {
    //PID controller calculations
    //The balancing robot is angle driven. First the difference between the desired angle (setpoint) and actual angle (process value)
    //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
    //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
    pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
    if(pid_output > 10 || pid_output < -10) {
      pid_error_temp += pid_output * 0.015 ;    
    }
    pid_output = PID_calculation(pid_error_temp, low_bat);
  }

  control_calculations(received_byte);

  motor_pulse_calculations();
  
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

///////////////////////////////////////////////////////////////////////////////////////
// Radio signal data for bot control
#define RADIO_NULL  B00000000
#define BOT_LEFT    B00000001
#define BOT_RIGHT   B00000010
#define BOT_FORWARD B00000100
#define BOT_REVERSE B00001000
#define BOT_STABLE  B00001100
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// \brief Adjust pid_output data for motion control commands 
///
/// \param[in] received_byte control byte received from the nunchuck
///
/// \return void 
///////////////////////////////////////////////////////////////////////////////
///      >>>>>>>>>>>>>>>>>>>>>>>UPDATE<<<<<<<<<<<<<<<<<<<<<<<<<<<
void control_calculations(byte received_byte) {
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  // Turn Left
  if(received_byte & BOT_LEFT) {                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
    DEBUG_PRINT("Turning Left");
  }
  // Turn Right
  if(received_byte & BOT_RIGHT) {                                           //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
    DEBUG_PRINT("Turning Right");
  }
  // Move Forward
  if(received_byte & BOT_FORWARD) {                                         //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    DEBUG_PRINT("Go Forward");
  }
  // Move Backward
  if(received_byte & BOT_REVERSE) {                                         //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
    DEBUG_PRINT("Go Backward");
  }   
  // No motion - slowly return to stable/vertical
  if(!(received_byte & BOT_STABLE)) {                                       //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0) {                                                   //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }  
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Calculates motor pulse outputs from the pid output 
///
/// \return void 
///////////////////////////////////////////////////////////////////////////////
///      >>>>>>>>>>>>>>>>>>>>>>>UPDATE<<<<<<<<<<<<<<<<<<<<<<<<<<<
void motor_pulse_calculations() {
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)
    pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)
    pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)
    pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)
    pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)
    left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)
    left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)
    right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)
    right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;  
}




// >>>>>> Initialization routines <<<<<<<<<<

///////////////////////////////////////////////////////////////////////////////
/// \brief TIMER2 interrupt handler used to update the stepper motor control lines
///
/// \return void
///////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory) {            //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}

  
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
























