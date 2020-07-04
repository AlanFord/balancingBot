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

#include <Wire.h>   //Include the Wire.h library so we can communicate with the gyro over I2C

#include "RF24.h"   //from the RF24 Arduino library

///////////////////////////////////////////////////////////////////////////////////////
// Define a data structure used to pass data to the  direction_control() function
///////////////////////////////////////////////////////////////////////////////////////
struct control_params {
  float pid_output_left;
  float pid_output_right;
  float pid_setpoint;
};


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
// Radio signal data for bot control
// These are the data values that can be received from the remote control
///////////////////////////////////////////////////////////////////////////////////////
const byte TURN_LEFT = B00000001;
const byte TURN_RIGHT = B00000010;
const byte MOVE_FORWARD = B00000100;
const byte MOVE_REVERSE = B00001000;
const byte STABILIZE = B00001100;


///////////////////////////////////////////////////////////////////////////////////////
// pin definitions for stepper motor controls
// These are used as masks to set bits in output registers
///////////////////////////////////////////////////////////////////////////////////////
const uint8_t LEFTMOTORDIR     = (0b00000001 << LEFTMOTORDIR_PIN);
const uint8_t LEFTMOTORSTEP    = (0b00000001 << LEFTMOTORSTEP_PIN);
const uint8_t RIGHTMOTORDIR    = (0b00000001 << RIGHTMOTORDIR_PIN);
const uint8_t RIGHTMOTORSTEP   = (0b00000001 << RIGHTMOTORSTEP_PIN);

///////////////////////////////////////////////////////////////////////////////////////
// a string used to communicate with the intended remote control receiver
///////////////////////////////////////////////////////////////////////////////////////
const byte radio_address[] = "1robt";

////////////////////////////////////////////////////////
///  \brief MPU-6050 I2C address (0x68 or 0x69).
////////////////////////////////////////////////////////
const int gyro_address = 0x68;

////////////////////////////////////////////////////////
///  \brief accelerometer calibration bias value that is
///  dependent on the physical mounting of the gyro.
///  This should be updated for each physical robot
////////////////////////////////////////////////////////
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



////////////////////////////////////////////////////////
///  \brief Turning velocity in units of  (30).
////////////////////////////////////////////////////////
const float turning_speed = 30;

////////////////////////////////////////////////////////
///  \brief Target maximum velocity in units of  (150).
////////////////////////////////////////////////////////
const float max_target_speed = 150;


////////////////////////////////////////////////////////
///  Global Variables
////////////////////////////////////////////////////////

////////////////////////////////////////////////////////
//  used by the primary timing routine
//  Initialized in setup()
//  Used in loop()
////////////////////////////////////////////////////////
unsigned long loop_timer;


/////////////////////////////////////////////////////////
// Calibration parameters set in setup() and used in loop()
////////////////////////////////////////////////////////
long gyro_yaw_calibration_value;
long gyro_pitch_calibration_value;

///////////////////////////////////////////////////////////////////////////////////////
// Variables for motor pulse calculations
// Values are determined in calculate_motor_pulse_interval()
// and used in the ISR() interrupt service routine
// These are marked volatile because their
// values can be changed by something beyond the control of the code section in which they appear
///////////////////////////////////////////////////////////////////////////////////////
volatile int throttle_left_motor;
volatile int throttle_right_motor;

///////////////////////////////////////////////////////////////////////////////////////
// User Config For the Radio
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus two additional pins
///////////////////////////////////////////////////////////////////////////////////////
RF24 radio(RADIO_CHIP_ENABLE_PIN, RADIO_CHIP_SELECT_PIN);




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
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
  // because gyro() will use it!
  calibrate_gyro();

  // Note: the radio uses the SPI bus
  initialize_radio();

  // Timer is used to by interrupt service to control the motors
  initialize_timer();
  //loop_timer = micros() + 4000;   //Set the loop_timer variable to the next end loop time
  loop_timer = micros();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  bool reset_pid;
  float angle_acc;
  byte received_byte = 0;
  static byte low_bat_flag = 0;
  static byte start_flag = 0;
  static float pid_output;
  static control_params control_parameters;
  static float angle_gyro;
  static float self_balance_pid_setpoint;
  unsigned long current_time;
  if ( battery_voltage_is_low() ) {
    digitalWrite(LED_PIN, HIGH);       //Turn on the led if battery voltage is to low
    low_bat_flag = 1;                  //Set the low_bat_flag variable to 1
  }

  angle_acc = get_accelerometer_angle();

  if (start_flag == 0 && angle_acc > -0.5 && angle_acc < 0.5) { //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                    //Load the accelerometer angle in the angle_gyro variable
    start_flag = 1;                                            //Set the start variable to start the PID controller
  }

  get_gyro_angle(angle_acc, angle_gyro);

  if (angle_gyro > 30 || angle_gyro < -30 || start_flag == 0 || low_bat_flag == 1) {  //If the robot tips over or the start variable is zero or the battery is empty
    reset_pid = true;
    start_flag = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }
  else {
    reset_pid = false;
  }
  pid_output = pid_calculator(reset_pid, pid_output, control_parameters.pid_setpoint, angle_gyro, self_balance_pid_setpoint);

  received_byte = get_radio_data();
  direction_control(pid_output, received_byte, control_parameters);

  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (control_parameters.pid_setpoint == 0) {                                                  //If the setpoint is zero degrees
    if (pid_output < 0)self_balance_pid_setpoint += 0.0015;                 //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if (pid_output > 0)self_balance_pid_setpoint -= 0.0015;                 //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  calculate_motor_pulse_interval(control_parameters.pid_output_left, control_parameters.pid_output_right);


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  //while (loop_timer > micros());
  //loop_timer += 4000;

  do {
    current_time = micros();
  } while (current_time - loop_timer < 4000);
  loop_timer = current_time;
}
