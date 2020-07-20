# Balancing Bot
## Table of Contents
- [Introduction](#introduction)
- [The Remote Control](#the-remote-control)
	- [Wiring the Nunchuck](#wiring-the-nunchuck)
	- [Wiring the Transceiver](#wiring-the-transceiver)
	- [Putting It All Together](#putting-it-all-together)
	- [Powering It Up](#powering-it-up)
	- [Programming the Arduino](#programming-the-arduino)
- [The Bot](#the-bot)
	- [Hardware](#hardware)
		- [The Frame](#the-frame)
		- [The Motors and Wheels](#the-motors-and-wheels)
		- [Motor Control](#motor-control)
		- [Balancing](#balancing)
		- [Power](#power)
		- [Communications](#communications)
		- [The Blinky Lights](#the-blinky-lights)
		- [The Brains Behind it All](#the-brains-behind-it-all)
	- [Software](#software)
		- [Timing Overview](#timing-overview)
		- [Battery Voltage Monitoring](#battery-voltage-monitoring)
		- [Attitude Monitoring](#attitude-monitoring)
		- [Wireless Communications](#wireless-communications)
		- [Serial Communications](#serial-communications)
		- [Motor Control Software](#motor-control-software)
		- [The Feedback Calculations](#the-feedback-calculations)
- [Operating the Balancing Bot](#operating-the-balancing-bot)
	- [Calibrating the Accelerometer](#calibrating-the-accelerometer)
	- [Starting up the Robot](#starting-up-the-robot)
	- [If Something Goes Wrong](#if-something-goes-wrong)




## Introduction
The Balancing Bot is a battery-powered two-wheeled balancing robot that can move under remote control.  The robot is built primarily from off-the-shelf parts.  The frame is plywood, easily made with hand tools or a small CNC router.  The robot’s electronics are based on an Arduino microcontroller and a combination gyro/accelerometer.  Communications with the remote control are performed over a 2.4 GHz wireless transceiver module.  The remote control is comprised of a repurposed Wii-compatible nunchuck controller, an Arduino microcontroller, and a 2.4 GHz wireless transceiver module.

This is a fork of the Joop Brokking's 
[balancing robot](http://www.brokking.net/yabr_main.html).
Many parts have been replaced with easier-to-obtain (for me) breakout boards from sparkfun.com, pololu.com, and amazon.com.

## The Remote Control
The primary component of the remote control is the Wii-compatible nunchuck.  The nunchuck joystick is used to maneuver the robot in right, left, forward, and backward movements.  The nunchuck is designed to communicate via a proprietary I2C connection.  Fortunately, it’s easy to communicate with the nunchuck using and Arduino Uno.  The Arduino programming environment provides a library (Wire) that implements the I2C communications protocol.

Getting the maneuvering commands from the remote control to the robot is performed using a Nordic Semiconductor nRF24L01+ 2.4 GHz wireless transceiver.  The communications protocol is proprietary, but the transceiver is available on a Sparkfun breakout board and communication is handled via the SPI and RF24 libraries.

Wiring the nunchuck to the Arduino can be plug-and-play if you use one of the various Wii nunchuck 
[breakout adapters](https://www.adafruit.com/product/345).  
However, the nunchuck can also be wired directly to the Arduino by removing the nunchuck connector.  This will be covered next.

### Wiring the Nunchuck
Removing the nunchuck connector reveals five wires, only four of which are required here - VCC (+3.3V), GND, SDA (data), and SCL (clock).  Unfortunately, the color of the various wires may change depending on the nunchuck supplier.  However, as a fallback the various wires can be identified by looking straight into the connector.  The wire layout would be as follows:

![nunchuck-connector]

Go ahead a cut off the connector and attach the wires to the following pins on an Arduino Uno – 

| Wii  | Arduino Pin |
| ---- | ------------|
|SDA   | A4          |
|SCL   | A5          |
|+3.3V | 3V3         |
|GND   | GND         |

Note that the Wii nunchuck is a +3.3V device and the Arduino is a +5V device.  This is accommodated because an I2C connection is *open-drain*, i.e. the data lines (SDA and SCL) are nominally either grounded or floating.  The Wii nunchuck has internal pull-up resistors that will pull each data line up to +3.3V when either the Nunchuck or the Arduino is not pulling the line to ground during signaling.

However, the Arduino also contains internal pull-up resistors that are enabled by default and will attempt to pull the signal lines up to +5V.  While this appears not to damage the nunchuck, it would be more appropriate to disable the Arduino internal pull-up resistors and rely only on the pull-ups internal to the nunchuck.

### Wiring the Transceiver
The transceiver that I used was part of a Sparkfun breakout board that used an SPI interface.  The SPI interface requires more data lines than an I2C or serial connection.  The following are the required connections:

| Transceiver | Arduino Pin |
| ----------- | ----------- |
|CE           | D9          |
|CSN          | D10         |
|MOSI         | D11         |
|MISO         | D12         |
|SCK          | D13         |
|VCC          | 5V          |
|GND          | GND         |
|IRQ          | not used!   |

### Putting It All Together
While all of those wires could be just attached to the Arduino Uno headers, it didn’t seem to make for stable connections.  In addition, it would be best if the wireless transceiver stood up vertically above the Arduino Uno to provide better radio reception.  My solution was to use an Arduino prototyping shield.  A female 8-pin header was soldered to the prototyping shield and a male right-angle 8-pin header was soldered to the transceiver board.  Plugging the transceiver into the female header on the prototyping shield allows the transceiver to stand upright above the Arduino Uno.  The nunchuck wires can be soldered directly to the appropriate locations on the prototyping shield.  I mounted the Arduino Uno on a 
[plastic mounting plate](https://www.adafruit.com/product/275)
 that provided an area to the side for securing the nunchuck cable with a cable tie and an adhesive-backed mounting base.

### Powering It Up
The remote control can be powered by a power supply with a barrel jack supplying between +9V and +12V.  Alternatively, it can be powered from a laptop via a USB connector.

To make the remote control more portable, it is possible to power it via a USB cable attached to a cell-phone battery pack.  Care must be taken to choose a batter pack that does not shut off when the current flow drops to about 20 mA (typical of the Arduino Uno).  If you have a battery pack that does tend to shut off, adding an LED to the Arduino that uses an additional 15 mA may keep the battery pack from shutting off.

### Programming the Arduino
The Arduino Uno program reads the state of the nunchuck joystick and transmits the state to the robot.  The Uno program is *Balancing_robot_remote.ino*.  When first powered up, the program initializes the microcontroller by making calls to the routines `initialize_serial()`, `initialize_radio()`, `initialize I2C()`, and `wake_nunchuck()`.  `initialize_serial()` inizializes a 9600 baud serial connection that can be optionally be used to monitor the state of the Uno.  `initialize_radio()` and `initialize I2C()` are self explanatory.  `wake_nunchuck()` initializes the I2C data communication with the nunchuck.  Of special interest are the lines in `initialize_I2C()` that disable the Arduino internal pull-up resistors.  This configuration permits safely connecting the +3.3V nunchuck with the +5V Arduino.  The relevant lines in `initialize_I2C()` are:

```cpp
Wire.begin();			//Start the I2C bus as master
digitalWrite(SCL, LOW);		//disable the pull-up on the SCL line
digitalWrite(SDA, LOW); 	//disable the pull-up on the SDA line
```

SCL and SDA are Arduino-defined constants referring to the appropriate pins.

Initializing the radio requires using a “pipe” address, or name. We need to ensure that the remote and the robot are on the same pipe address.  The pipe address is a five-character string – in this case “1robt”.

Communicating with the nunchuck consists of sending 0x00 to the nunchuck followed by a request for six bytes of data.  The first byte of data received represents the left/right position of the joystick, with the values varying from 0 to 255 (full left to full right).  The second byte of data received represents the front/back position of the joystick, with values varying from 0 to 255 (full reverse to full forward).  Ignoring a dead zone of 80 to 170 (to avoid jitter and noise), the nunchuck data is converted to robot commands of BOT_LEFT, BOT_RIGHT, BOT_FORWARD, and BOT_REVERSE.  The commands are represented by the following byte encoding:

```
BOT_LEFT            B00000001
BOT_RIGHT           B00000010
BOT_FORWARD         B00000100
BOT_REVERSE         B00001000
BOT_STABLE          B00001100
```

The values can be combined by ORing the bytes together.  For example, to move forward and turn right simultaneously the values of BOT_FORWARD and BOT_RIGHT would be ORed together to form a command byte of B00000110.

The command byte is then broadcast via the transceiver using the library call radio.write(). After a delay of 40 milliseconds the process is repeated.

## The Bot
It takes a lot to keep the robot upright and doing it’s job.  The robot runs on two wheels, powered by one stepper motor for each wheel.  Controlling the direction of wheels independently allows the robot to roll forward, roll backward, turn right, turn left, or a combination of these.  In addition to the overall motion of the robot, the wheels are also used to balance the robot.  If the accelerometer indicates that the robot is tipping forward or backward, the wheels are driven to move the wheels in the direction of the tilt, effectively moving the robot body into an upright positon.  The ability to do this without having the top of the robot flailing around requires that a weight be added to the top of the robot, simulating an “inverted pendulum”.  In this case the weight is implemented by mounting a large lithium-polymer (LiPo) battery on the top of the robot frame.  Beyond helping balance the robot, the size of the LiPo batter also provides a substantial power supply for the robot’s electronics and motors.

The robot also needs computing power, but an Arduino Uno (the microcontroller used in the remote control) is too large to use on the robot.  Instead, an Arduino Pro Mini is used.  The Pro Mini is significantly smaller and comes in 3.3 V and 5 V versions.  Only the 5V version should be used, as 5V is need to interface with some of the electronics.  This will be covered in more detail later.

### Hardware

#### The Frame
Wood.  Just wood.  Well, maybe a little fancier than usual.  The frame is built out of  “baltic birch” plywood.  This plywood has more plies (layers) than your average plywood, making it stronger, flatter, etc.  The various pieces of the frame are held together with wire brads from a brad nailer.  Hand nailing will work also.

#### The Motors and Wheels
The robot moves because of stepper motors attached to wheels.  The stepper motors used in this project are NEMA 14-size bipolar motors from STEPPERONLINE (part number 14HS13-0804S).  The motors had the following characteristics:

```
18 N-cm (25.5 oz.-in) holding torque
1.8 degree stepper angle (200 steps/rev)
5.4V, 0.8A
6.8 ohms
5 mm shaft diameter
24 mm shaft length
```

The motors are mounted directly to the wooden robot body using the four threaded mounting holes found on the face of all NEMA stepper motor bodies.

Wheels can be found from many sources or may be repurposed from old toys.  The wheels used in this project were 60mm x 8mm with silicone tires.  The wheels were mounted to the stepper motor shafts using Pololu aluminum mounting hubs sized for a 5mm stepper shaft.

#### Motor Control
The stepper motors are controlled by stepper motor drivers, one per motor.  The stepper motor drivers receive motor positioning signals from a microcontroller and convert them into motor coil currents to control the motors.  This design uses two of the Pololu A4988 Stepper Motor Driver.  This driver can operate from 8 V to 35 V and can deliver up to 1 A per phase without a heat sink.  Microstepping down to 1/16-step is available.  VDD can be in the range of 3 V – 5.5 V.  Current limiting (the motors in use can only take 800 mA) is performed by adjusting an on-board trimmer potentiometer.  After adjusting the potentiometer, the current limit is

```
Current limit = VREF / (8 x 0.068 ohms)
```

where VREF is the voltage on the trimmer dial (measure by lightly touching – pressing down can change the trimmer pot adjustment!) or at the neighboring via.  For example measuring 0.5 V with a reference voltage of 5 V would mean a current limit of 0.919 A.  As the motors can take a maximum of 800 mA, the measured voltage should never be greater than 0.435 V, assuming a reference voltage of 5 V.  The original design indicates a current limit of 150 mA was to be used.  The modified design used here needs about 500 mA to power the motors such that they will hold position.

The following is a general depiction of how the motor driver should be wired.

![A4988]

Additionally, the \~ENABLE pin should be raised to the logic power supply voltage to disable the driver.  MS1, MS2, and MS3 
can be used to control microstepping as follows:

| MS1  | MS2  | MS3  | Microstep Resolution |
| ---- | ---- | ---- | -------------------- |
| Low  | Low  | Low  | Full step            |
| High | Low  | Low  | Half step            |
| Low  | High | Low  | 1/4 step             |
| High | High | Low  | 1/8 step             |
| High | High | High | 1/16 step            |

The motor drivers used in the robot are wired as follows:

![A4988-wiring]

Only MS2 is High, so the microstepping is set to ¼ step.  The \~ENABLE line is not connected; hence the stepper driver is always enabled.  The STEP and DIR pins are connected to the following Arduino Pro Mini pins:

|            | Arduino Pins | Arduino Pins |
|            | for          | for          |
| Driver Pin | Left Motor   | Right Motor  |
| ---------- | ------------ | ------------ |
| STEP       |	D4          | D2           |
| DIR        |	D5          | D3           |

#### Balancing
Balancing of the robot is performed by monitoring the gyro/accelerometer.  The gyro/accelerometer is implemented using a MPU-6050 six-axis sensor on a GY-521 breakout board.  The GY-521 communicates over I2C and can be powered with between 3.3V and 5V.

Care should be taken when mounting the GY-521 to the robot.  The accelerometer X-axis should be the vertical axis with the Y-axis pointing to the left as you face the front of the robot.  Mounting in other orientations will require software changes.

Wiring up the GY-521 is straight forward – 

| GY-521 | Arduino Pro Mini Pin |
| ------ | -------------------- |
| SDA    | A4                   |
| SCL    | A5                   |
| VCC    | 5V                   |
| GND    | GND                  |

#### Power
Power ultimately comes from a 3S LiPo battery, used not only for power but also to add weight to the top of the robot for balancing.  Power control and polarity are provided by a toggle switch and a diode.  The battery voltage is regulated down to 5V using a D24V10F5, 5V, 1A step-down voltage regulator available from pololu.com.  5V is then supplied to all of the on-board electronics.  The +11.75V output of the diode is supplied to the motor drivers to power the stepper motors.  The following summarizes the power supply:

![power-supply]

#### Communications
The robot uses the same nRF24L01+ 2.4 GHz wireless transceiver used by the remote control.  The following are the required connections:

| Transceiver | Arduino Pin |
| ----------- | ----------- |
| CE          | D09         |
| CSN         | D10         |
| MOSI        | D11         |
| MISO        | D12         |
| SCK         | D13         |
| VCC         | 5V          |
| GND         | GND         |
| IRQ         | not used!   | 

#### The Blinky Lights
One status LED is used on the robot.  A red LED is connected is series with a 330 ohm resistor between the Arduino D17 pin and ground.  Of course, I forgot to add this LED originally so my robot has it bodge-wired into place.  

The LED is first used when the robot is initially powered on.  The robot should be laying on it’s back when the power switch is toggled.  The LED will begin blinking as the accelerometer is calibrated.  Once the blinking stops, slowly rotate the robot to vertical and it will stabilize.

The other use of the LED is to warn when the battery voltage is too low.  The LED will turn on continuously when the battery voltage drops below +10.5V.  At this point the motors will also be de-energized and the robot will stop.

#### The Brains Behind it All
Integrating the robot parts is the job of the Arduino Pro Mini.  The Arduino Pro Mini comes in both a 3.3V and 5V version, but the 5V version should be used here so the voltage is consistent with the supply voltage used by the transceiver and the motor drivers.

### Software
The Arduino provides several functions, namely attitude monitoring, motor control, communications with the remote, and battery voltage monitoring.  These will be described in the following sections.  As an introduction though, it should be clear that the Arduino separates work into two categories – setup and loop.  Work performed in the setup occurs only once after power-up.  Work performed in the loop is repeated until the robot is powered down.  Some of the robot functions are performed in the setup and some are controlled by the loop.

#### Timing Overview
The various processes that make up the control software work on a number of timing intervals.

- The remote control has a primary timing loop that is executed once every **40 milliseconds**.  This is the
frequency at which the handheld controls are polled for updates and new data are transmitted to the robot.
- Data received from the remote control by the robot is considered valid for only **100 milliseconds** if no other data is received.  After that time it is discarded.  This avoids the issue of using old, invalid commands when the radio link is interrupted.
- The robot has a primary timing loop that is executed once every **4 milliseconds**.  This timing loop 
controlls the frequency at which the gyro is checked, incoming remote control commands are logged, 
and balancing control calculations are updated, and motor speeds are determined.
- The time interval for the primary timing loop is also used to calculate the change in robot angle during the last loop interval (**4 milliseconds**).  The gyro only determines the *rate* of change in the angle.  The time interval is used to determine the change in the angle.
- The interrupt routine that drives the motors executes every **20 microseconds**.  The stepper motor drive 
period is comprised of one interrupt period executing a motor step followed by several non-stepping interrupt periods.

#### Battery Voltage Monitoring
The battery voltage is being monitored on pin A0, an analog input.  The nominal voltage for a 3S LiPo battery is +11.1 V, a maximum voltage of +12.6 V, and the minimum operating voltage is 9.9 V.  There is a diode in the battery circuit to prevent hooking the battery up with the wrong polarity.  The voltage drop across the diode is approximately 0.85 V.  Hence the remaining maximum voltage is +11.75 V and the minimum voltage would be +9.05 V.  Unfortunately the Arduino can only measure voltages between 0 V and 5 V.  Two resistors can be used to create a voltage divider to reduce the voltage to no greater than +5V.  The maximum voltage needs to be divided by at least (+11.75 V)/(+5 V) = 2.35.  Using readily available resistor sizes, a voltage divider can be made from 33K ohm and 21.5K ohm resistors, for a divider of (33K + 21.5K)/21.5 = 2.535.  Now the maximum voltage would be +11.75 V/2.535 = +4.64 V.

When reading the A0 pin, the Arduino generates an integer value of 0 for 0V and 1023 for 5V, or 0.00489 V per division.  Thus the integer generated from reading the pin would be

```
A0 = (battery voltage – 0.85V)/2.535/(0.00489V/division), or
A0 = (battery voltage * 80.67) – 68.57
```

Now, reading A0 with values between 0 and 1023, it’s hard to tell what the voltage really is.  So flipping the equation around and multiplying by 1000 (to get millivolts), we get

```
voltage reading = (A0+68.57)/80.67*1000 = A0*12.4 + 850
```

If the voltage reading drops below 10500 (+10.5V) the LED is turned on and the motors are de-energized.  The batter check is performed in the main loop of the Arduino software, executed on a 4 millisecond interval.

#### Attitude Monitoring
So why include both gyro and an accelerometer?  The gyro can determine the rate at which the robot is rotating (i.e. falling).  How is the gyro used to determine the angle of the robot?  The gyro’s value for rotation (degree/sec) is multiplied by the time since the last check to determine how much the robot’s angle has changed over that time.  But changed from what?  The gyro can’t determine what angle the robot started at.  This is where the accelerometer comes in.  When the robot is moved to an upright position, the accelerometer can tell from the gravitational pull how close the robot is to vertical.  Once the robot is close to vertical the angle calculated from the accelerometer is used to initialize the gyro’s ongoing, updated angle calculations.  Voila!

The gyro/accelerometer (henceforth referred to as the gyro) is initialized in the Arduino’s setup() function.  The gyro operating mode is set to 250 degree/sec full scale for the gyro and +/- 4g full scale for the accelerometer.  Both the gyro and accelerometer functions return a signed 16-bit integer.  The accelerometer has a sensitivity of 8192 LSB/g.  The gyro has a sensitivity of 131 LSB/degree/sec.

The gyro yaw and gyro pitch values are read from the gyro and then the program is paused for \~4 milliseconds.  This is repeated for a total of 500 times and the resulting yaw and pitch values are averaged over the 500 values.  The resulting average yaw and average pitch are the gyro calibration values.

In the main loop the gyro angle is initialized to equal the accelerometer angle only once, at the beginning of operation when the robot is standing vertical.  The gyro angle is then updated once every \~4 milliseconds (the loop time for the main program loop).

#### Wireless Communications
As with the remote control, communications with the wireless transceiver is done with the SPI bus.  As with the remote control, the radio is initialized with a pipe address of “1robt”.  Interestingly, while the radio can transmit on only one pipe, it can receive on six pipes simultaneously.  Thus when initializing the radio we must also specify a pipe number, in this case “1”.    Wireless communications occur in the main loop of the Arduino software, executed on a 4 millisecond interval.

#### Serial Communications
Serial communications can be enabled at compile-time by defining the macro SERIALCOMM.  Serial communications are then initiated at 9600 baud (slow enough not to impede the other work being done).  Serial communications occur over the USB connection.

#### Motor Control Software
Motor control is not done in either of the normal Arduino functions setup() or loop().  Instead, motor control is performed in an interrupt handler.  The Arduino has built-in timers that can cause the microcontroller to perform operations on a repeating fixed interval.  One of the ways this can be done is to associate a special program (i.e. a function) with the timer.  Such a program/function is referred to as an interrupt handler.  The robot uses Timer 2 to perform motor control on an interval of every 20 microseconds.  The motor control is performed by a function called ISR(TIMER2_COMPA_vect).  Timer 2 is initialized in the standard Arduino setup() function.

Moving the stepper motor by one step (or by a ¼ step in this configuration) a pulse is generated on the appropriate input pins for the stepper controllers.  This is done by pulsing Arduino Pro Mini pin D2 for the right motor and pin D4 for the left motor.
Some caution is needed when controlling the direction of the stepper motors.  The stepper motors and drivers are configured symmetrically with respect to the Arduino Pro Mini.  However, the left motor is rotated 180 degrees when mounted on the robot.  Thus, the right motor will go forward when Arduino pin D3 is high and reverse when pin D3 is low.  The left motor is reversed, going forward when Arduino pin D5 is low and reversing when pin D5 is high.

Manipulating the Arduino Port D pins for motor stepping and direction is made a little easier by the constants defined in the Arduino program:

```cpp
const uint8_t LEFTMOTORDIR     = (0b00000001 << 5);
const uint8_t LEFTMOTORSTEP    = (0b00000001 << 4);
const uint8_t RIGHTMOTORDIR    = (0b00000001 << 3);
const uint8_t RIGHTMOTORSTEP   = (0b00000001 << 2);
```

#### The Feedback Calculations
The feedback calculations are designed to continuously update the motor positioning required to accomplish several (sometimes mutually exclusive) goals simultaneously.  The goals are:
 - Keep the bot vertical and stable when not moving.  This is accomplished by adjusting the variable `self_balance_pid_setpoint`

 - Incorporate necessary changes when the remote control requests a turn.  This accomplished by setting the variables `pid_output_left` and `pid_output_right` to be +/- `turning_speed`.  The sign is adjusted to make the wheels turn in opposite directions, spinning the robot in place.  The `turning_speed` variable is a predefined constant (20).

 - Incorporate changes when the robot is to move forward or backward.  To move forward, the robot is leaned forward by adjusting pid_setpoint in 0.005 increments each time the main loop is executed (4 millisec) until the maximum speed `max_target_speed` (150) is reached.

  - If the robot is more than 30 degrees from vertical (i.e. it fell over), shut the motors down and reset the feedback calculations.

  - When the robot is not at the required angle from vertical, calculate a new.

As was mentioned in the Motor Control section, the interrupt handler is executed every 20 microseconds.  The motor step pulse is activated in one interrupt and deactivated in the next.  Thus, the minimum pulse period (the full cycle) is 40 microseconds and the step frequency is 25,000 steps/sec.  The motors are nominally 200 steps/rev and the motor controllers are set for ¼ microstepping, so the maximum speed of the robot is 

```
(25,000microsteps/sec)(1step/4microsteps)(1rev/200steps) = 31.25 revolutions/sec
 ```

The wheels/tires have a diameter of 60 mm, resulting in a maximum linear speed of

```
31.25 rev/sec * (pi * 60 mm) = 5890.5 mm/sec = 19.3 ft/sec.
```

`throttle_left_motor` and `throttle_right_motor` are generated by the feedback calculations to control the motor.  They contain the number of 20 microsecond intervals between the pulses used to move the stepper motor.  A counter for each motor counts up to the throttle value from 0.  On 0 the stepper pulse is created and on 1 the stepper pulse is ended, giving one 20 microsecond pulse followed by a (throttle-1) * 20 microsecond delay.  A negative value of throttle_left_motor or throttle_right_motor will cause the motor to reverse.

In summary, it’s important to remember that the value generated by the feedback calculations is the *delay* between stepper motor steps.

## Operating the Balancing Bot

### Calibrating the Accelerometer
This really only has to be done once, so I’m putting it in a separate section of the documentation.

### Starting up the Robot
It’s pretty simple; just follow these steps:

 1.	Power up the remote, either using a wall wart or a battery pack
 2.	Lay the robot on it’s back on a flat surface.
 3.	Flip the toggle switch to power-up the robot.
 4.	Wait for the gyro/accelerometer to calibrate (the red LED will stop blinking)
 5.	Gently rotate the robot to a vertical position
 6.	Turn it loose and start driving it around!

### If Something Goes Wrong 
If the robot falls down because it ran into something, just pull it back upright and it should stabilize itself.

If, during operation, the red LED turns on, the motors will de-energize because the LiPo battery voltage has dropped too low.  Flip the toggle switch to turn off the robot, swap in a new battery, and begin again!


<!-- Images -->
[nunchuck-connector]:   ./docs/connector.jpeg      "Nunchuck connector wiring"
[A4988]:                ./docs/basic.png           "A4988 diagram"
[A4988-wiring]:         ./docs/a4988.jpeg          "A4988 wiring diagram"
[power-supply]:         ./docs/power-supply.jpeg   "Power supply diagram"
