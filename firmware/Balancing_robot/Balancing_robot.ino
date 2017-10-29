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

#define DEGREEPERRADIAN  57.296

///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
byte radio_address[] = "1robt";
///////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////
// User Config for the gyro
int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = -600;                            //Enter the accelerometer calibration value
///////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////
//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)
///////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declaring global variables
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  initialize_serial();
  initialize_I2C();  // the I2C bus is used to communicate with the gyro
  wake_gyro();
  calibrate_gyro();
  initialize_timer();  //timer will be used to send pulse trains to the stepper motors
  
  // initialize pins used for stepper motor control
  pinMode(2, OUTPUT);                                                       //Configure digital port 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital port 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital port 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital port 5 as output
  
   // initializing radio at the end of "setup", as it uses the same pin (D13) as the led
  initialize_radio();
 
  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time
  Serial.println("initialized . . .");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  //check_battery_voltage();  // WARNING- may conflict with the SPI SCK function!

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
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  retrieve_gyro_acceleration();												//acc data is limited to +/-8200
  angle_acc = asin((float)accelerometer_data_raw/8200.0)* DEGREEPERRADIAN;  //Calculate the current angle in degrees [-90,+90] according to the accelerometer
  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  Serial.print("raw data is ");
  Serial.println(accelerometer_data_raw);
  Serial.print("Angle is ");
  Serial.println(angle_acc);
  retrieve_gyro_pitch_and_yaw();
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  // 0.000031 == (1/131 deg/sec/LSB) * 0.004 sec/loop = 0.00003053
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angle (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10) {
    pid_error_temp += pid_output * 0.015 ;    
  }
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

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1) {   //If the robot tips over or the start variable is zero or the battery is empty
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

  // Turn Left
  if(received_byte & BOT_LEFT) {                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
    Serial.println("Turning Left");
  }
  // Turn Right
  if(received_byte & BOT_RIGHT) {                                           //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
    Serial.println("Turning Right");
  }
  // Move Forward
  if(received_byte & BOT_FORWARD) {                                         //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    Serial.println("Go Forward");
  }
  // Move Backward
  if(received_byte & BOT_REVERSE) {                                         //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
    Serial.println("Go Backward");
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

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void initialize_radio() {
  radio.begin();                                                            //Start the SPI nRF24L01 radio
  radio.openReadingPipe(1,radio_address);                                   // Open a reading pipe on the radio
  radio.startListening();
}
  
void initialize_I2C() {
  Wire.begin();                                                             //Start the I2C bus as master
  //TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
  delay(20);                                                                //Short delay
}

void wake_gyro() {
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

void calibrate_gyro() {
  pinMode(13, OUTPUT);                                                      //Configure digital port 6 as output
  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
}

void check_battery_voltage(){
  //Load the battery voltage to the battery_voltage variable.
  //85 is the voltage compensation for the diode.
  //Resistor voltage divider => (3.3k + 2.2k)/2.2k = 2.5
  //12.5V equals ~5V @ Analog 0.
  //12.5V equals 1023 analogRead(0).
  //1250 / 1023 = 1.222.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) * 1.222) + 85;
  
  if(battery_voltage < 1050 && battery_voltage > 800){                      //If batteryvoltage is below 10.5V and higher than 8.0V
    digitalWrite(13, HIGH);                                                 //Turn on the led if battery voltage is to low
    low_bat = 1;                                                            //Set the low_bat variable to 1
  }
}

void retrieve_gyro_acceleration(){
  // This function retrieves the "Z" acceleration from gyro registers 0x3F through 0x40
  // Note that the orientation of the gyro when the bot is vertical places the gyros "z" axis horizontal to the floor/ground
  // Having initialized the gyro to +/-4g mode, the returned values will range from +/- 8192
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = (Wire.read()<<8)|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;
}

void retrieve_gyro_pitch_and_yaw(){
  // This function retrieves the gyroscope measurements for the "x" and "y" axes
  // seems like the "x" axis corresponds to the yaw and the "y" axis corresponds to the pitch
  // Note that the orientation of the gyro when the bot is vertical places the gyros "y" axis vertical wrt the floor/ground
  // Note that the orientation of the gyro when the bot is vertical places the gyros "x" axis out the left side of the bot
  // Having initialized the gyro to +/- 250deg/sec mode, the returned values will range from +/- 131
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 0x43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = (Wire.read()<<8)|Wire.read();                           //Combine the two bytes to make one integer
  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  gyro_pitch_data_raw = (Wire.read()<<8)|Wire.read();                         //Combine the two bytes to make one integer
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
}

void initialize_serial(){
  Serial.begin(9600);                                               //Start the serial port at 9600 kbps
  printf_begin();
}
























