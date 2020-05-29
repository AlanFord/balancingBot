///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

// #define BR_DEBUG

#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

#include "RF24.h"   //from the RF24 Arduino library

///////////////////////////////////////////////////////////////////////////////////////
// User Config for the GPIO Pins
// Other pins (such as I2C, UART, and SPI pins) may be used internally by various libraries
///////////////////////////////////////////////////////////////////////////////////////
const int RIGHTMOTORSTEP_PIN   =  2;
const int RIGHTMOTORDIR_PIN    =  3;
const int LEFTMOTORSTEP_PIN    =  4;
const int LEFTMOTORDIR_PIN     =  5;
const int RADIO_CHIP_ENABLE_PIN = 9;
const int RADIO_CHIP_SELECT_PIN = 10;
const int LED_PIN = 17;


///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(RADIO_CHIP_ENABLE_PIN,RADIO_CHIP_SELECT_PIN);
///////////////////////////////////////////////////////////////////////////////////////
const byte radio_address[] = "1robt";

// User Config for the gyro
////////////////////////////////////////////////////////
///  \brief MPU-6050 I2C address (0x68 or 0x69).
////////////////////////////////////////////////////////
const int gyro_address = 0x68;                                     
const int acc_calibration_value = -503;                            

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



const float turning_speed = 30;                                    //Turning speed (20)
const float max_target_speed = 150;                                //Max target speed (100)


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

static unsigned long loop_timer;

float angle_gyro, angle_acc, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, pid_last_d_error;
float pid_output, pid_output_left, pid_output_right;


const uint8_t LEFTMOTORDIR     = (0b00000001 << LEFTMOTORDIR_PIN);
const uint8_t LEFTMOTORSTEP    = (0b00000001 << LEFTMOTORSTEP_PIN);
const uint8_t RIGHTMOTORDIR    = (0b00000001 << RIGHTMOTORDIR_PIN);
const uint8_t RIGHTMOTORSTEP   = (0b00000001 << RIGHTMOTORSTEP_PIN);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  #ifdef BR_DEBUG
  Serial.begin(9600);
  #endif

  // set input/output modes for gpio pins
  pinMode(RIGHTMOTORSTEP_PIN, OUTPUT);                            
  pinMode(RIGHTMOTORDIR_PIN, OUTPUT);                            
  pinMode(LEFTMOTORSTEP_PIN, OUTPUT);                            
  pinMode(LEFTMOTORDIR_PIN, OUTPUT);                            
  pinMode(LED_PIN, OUTPUT);                      

  // I2C will be used to communicate with the gyro
  initialize_i2c();

  initialize_gyro();

  // setting LED_PIN to OUTPUT >>MUST<<
  // be completed before calling calibrate_gyro() 
  calibrate_gyro();

  // Note: the radio uses the SPI bus
  initialize_radio();

  // Timer is used to by interrupt service to control the motors
  initialize_timer();

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  get_radio_data();
  
  if ( battery_voltage_is_low() ) {
    digitalWrite(LED_PIN, HIGH);                                            //Turn on the led if battery voltage is to low
    low_bat = 1;                                                            //Set the low_bat variable to 1
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  angle_acc = get_accelerometer_angle();

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  get_gyro_angle(angle_acc);


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angle (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to move the robot forward
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to move the robot backward
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  ///////////
  // Note:
  // The binary operations used to set the left and right motor directions are not identical.
  // This is needed because the motors face in opposite directions, hence the forward/backward bit pattern is opposite.
  ///////////
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      PORTD |= LEFTMOTORDIR;                                                //Set output 3 HIGH to REVERSE the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD &= ~LEFTMOTORDIR;                                            //Set output 3 LOW for a FORWARD direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTD |=  LEFTMOTORSTEP;         //Set output 2 HIGH to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= ~LEFTMOTORSTEP;         //Set output 2 LOW because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD &= ~RIGHTMOTORDIR;                                              //Set output 5 LOW to REVERSE the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD |= RIGHTMOTORDIR;                                            //Set output 5 HIGH for a FORWARD direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTD |=  RIGHTMOTORSTEP;       //Set output 4 HIGH to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= ~RIGHTMOTORSTEP;       //Set output 4 LOW because the pulse only has to last for 20us
}

void initialize_radio() {
  radio.begin();                                                            //Start the SPI nRF24L01 radio
  radio.openReadingPipe(1,radio_address);                                   // Open a reading pipe on the radio
  radio.startListening();
}

void get_radio_data(){
  // receive data from the radio (i.e. from the remote controller)
  if(radio.available()){
    radio.read(&received_byte,sizeof(received_byte));
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  if(receive_counter <= 25)receive_counter ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else received_byte = 0x00;                                                //After 100 milliseconds the received byte is deleted
}

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

void initialize_gyro() {
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}

void calibrate_gyro() {
  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(LED_PIN, !digitalRead(LED_PIN));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 4 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
}

bool battery_voltage_is_low() {
  int battery_voltage;
  //Load the battery voltage to the battery_voltage variable.
  //850 is the voltage compensation for the diode.
  //Resistor voltage divider => (3.3k + 2.15k)/2.15k = 2.5349
  //12.674V equals ~5V @ Analog 0.
  //12.472V equals 4.92V @ Analog 0.
  //12472 / 1023 =
  //12.674V equals 1023 analogRead(0).
  //12674 / 1023 = 12.192.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) * 12.191) + 850;
  if(battery_voltage < 10500 && battery_voltage > 8000){   //If batteryvoltage is below 10.5V and higher than 8.0V
  	return true;
  }
  return false;
}

float get_accelerometer_angle() {
  float accelerometer_data_raw;
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;
  // note: 57.296 is degrees/radian
  return asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer
}

void get_gyro_angle(float angle_acc) {
  float gyro_yaw_data_raw;
  float gyro_pitch_data_raw;
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  // 1/(250 cycles/sec)(131 LSB/deg/sec)] = 0.0000305 deg/cycle/LSB
  angle_gyro += gyro_pitch_data_raw * 0.0000305;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;            //Correct the drift of the gyro angle with the accelerometer angle
}




























