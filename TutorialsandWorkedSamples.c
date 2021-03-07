
 
/////////////////////////////////////////// MOTOR CODE ////////////////////////////////////////////////


// Codes (Modified from logan’s based code)//
#include <Servo.h> // include the library of servo motor control
// define the control pin of each motor
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// three machine states
enum STATE
{
  INITIALISING,
  RUNNING,
  STOPPED
};
// create servo objects for each motor
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;

int speed_val = 100;
int speed_change;

void setup()
{
  Serial.begin(9600); // start serial communication
}

void loop()
{
  // put your main code here, to run repeatedly:
  static STATE machine_state = INITIALISING; // start from the sate INITIALIING
  switch (machine_state)
  {
  case INITIALISING:
    machine_state = initialising();
    break;
  case RUNNING:
    machine_state = running();
    break;
  case STOPPED:
    machine_state = stopped();
    break;
  };
}

STATE initialising()
{
  enable_motors();                // enable motors
  Serial.println("INITIALISING"); // print the current stage
  return RUNNING;                 // return to RUNING STATE DIRECTLY
}

STATE running()
{
  read_serial_command(); // read command from serial communication

  speed_change_smooth(); //function to speed up and slow down smoothly

  return RUNNING; // return to RUNNING STATE again, it will run the RUNNING
} // STATE REPEATLY

STATE stopped()
{
  disable_motors(); // disable the motors
}

void speed_change_smooth() // change speed, called in RUNING STATE
{
  speed_val += speed_change; // speed value add on speed change
  if (speed_val > 1000)      // make sure speed change less than 1000
    speed_val = 1000;
  speed_change = 0; //make speed change equals 0 after updating the speed value
}

void read_serial_command() // this function is called in RUNING STATE
{
  if (Serial.available())
  {
    char val = Serial.read(); // get the input from serial communication
    Serial.print("Speed:");   //   print the current speed
    Serial.print(speed_val);
    Serial.print(" ms ");

    switch (val)
    {
    case 'w':
    case 'W':
      forward(); // call function forward if the command is “w” or “W”
      Serial.println("Forward");
      break;
    case 's':
    case 'S':
      reverse(); // call function reverse if the command is “s” or “S”
      Serial.println("Backward");
      break;
    case 'q':
    case 'Q':
      strafe_left(); // call function strafe_left if the command is “q” or “Q”
      Serial.println("Strafe Left");
      break;
    case 'e':
    case 'E':
      strafe_right(); // call function strafe_right if the command is “e” or “E”
      Serial.println("Strafe Rightt");
      break;
    case 'a':
    case 'A':
      ccw(); // call function ccw if the command is “a” or “A”
      Serial.println("ccw");
      break;
    case 'd':
    case 'D': // cw(); // call function cw if the command is “d” or “D”

        Serial.println("cw");
        break;
        case '-':
    case '_':
      speed_change = -100; // make the speed change equals 100
      Serial.println("-100");
      break;
    case '=':
    case '+':
      speed_change = 100; // make the speed change equals 100
      Serial.println("+");
      break;
    case 'p':
    case 'P':
      //default:
      stop();
      Serial.println("stop");
      break;
    }
  }
}

void disable_motors()
{ // function disable all motors, called in  STOPPED STATE
  left_front_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_front_motor.detach();

  pinMode(left_front, INPUT); // set pinMode for next step
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{ //enable all motors, was called in INITIALZING SATE
  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_front_motor.attach(right_front);
}

void stop()
{ // stop motors
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}
void forward()
{ // moving forward
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse()
{ // reverse
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}
void strafe_left()
{ // straight left
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}
void strafe_right()
{ //straight right
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}
void cw()
{ //clockwise
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}
void ccw()
{ //anticlockwise
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}



//////////////////////////////////// INFRARED SENSOR CODE /////////////////////////////////

int irsensor = A0; //sensor is attached on pinA0
byte serialRead = 0;               //for control serial communication
int signalADC = 0;                 // the read out signal in 0-1023 corresponding to 0-5v

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); // start serial communication
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (Serial.available()) // Check for input from terminal
  {
    serialRead = Serial.read(); // Read input
    if (serialRead == 49)       // Check for flag to execute, 49 is ascii for 1, stop serial printing
    {
      Serial.end(); // end the serial communication to display sensor data on monitor
    }
  }

  signalADC = analogRead(irsensor);               // the read out is a signal from 0-1023 corresponding to 0-5v
  int distance1 = 17948 * pow(signalADC, -1.22);  // calculate the distance using the datasheet graph
  int distancec = 46161 * pow(signalADC, -1.302); // calculate the distance using the calibrated graph
  Serial.print("distance1  ");                    // print the results out using serial print
  Serial.print(distance1);
  Serial.println("cm ");
  Serial.print("distancec  ");
  Serial.println(distancec);
  Serial.println("cm");
}



//////////////////////////////////////// GYRO CODE ////////////////////////////////////////

int sensorPin = A2;             // Define the pin that gyro is connected
int T = 100;                    // T is the time of one loop
int sensorValue = 0;            // read out value of sensor
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;      // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity  less than
                                // this value will not be ignored
float gyroRate = 0;             // read out value of sensor in voltage
float currentAngle = 0;         // current angle calculated by angular velocity integral on
byte serialRead = 0;            // for serial print control

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  // this section is initialize the sensor, find the the value of voltage when gyro is zero
  int i;
  float sum = 0;
  pinMode(sensorPin, INPUT);

  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (i = 0; i < 100; i++) //  read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    sensorValue = analogRead(sensorPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100; // average the sum as the zero drifting
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (Serial.available()) // Check for input from terminal
  {
    serialRead = Serial.read(); // Read input
    if (serialRead == 49)       // Check for flag to execute, 49 is asci for 1
    {
      Serial.end(); // end the serial communication to display the sensor data on monitor
    }
  }

  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(sensorPin) * gyroSupplyVoltage) / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage/1023*5);

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }

  // keep the angle between 0-360
  if (currentAngle < 0)
  {
    currentAngle += 360;
  }
  else if (currentAngle > 359)
  {
    currentAngle -= 360;
  }

  Serial.print(angularVelocity);
  Serial.print(" ");
  Serial.println(currentAngle);

  // control the time per loop
  delay(T);
}


////////////////////////////////////// PHOTOTRANSISTOR CODE /////////////////////////////////////////
int sensorValue = A1;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:

  float volts = analogRead(sensorValue) * 5.0 / 1024.0;
  Serial.print("analogy");
  Serial.print(analogRead(sensorValue));
  Serial.print("    ");
  Serial.print("A1 = ");
  Serial.println(volts);
  delay(100);
}



// Robot’s Base code
/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>             //Need for Servo pulse output
#include <FaBo9Axis_MPU9250.h> //refer to Installing9250Lib.png to install library

//#define NO_READ_MPU //Uncomment of MPU is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

    //State machine states
    enum STATE {
      INITIALISING,
      RUNNING,
      STOPPED
    };

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor; // create servo object to control Vex Motor Controller 29
Servo right_font_motor; // create servo object to control Vex Motor Controller 29

#ifndef NO_READ_NPU
FaBo9Axis fabo_9axis;
bool axis_OK;
#endif

int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  //If no MPU is attached then .begin will hang forever
#ifndef NO_READ_MPU
  if (fabo_9axis.begin())
  {
    SerialCom->println("configured FaBo 9Axis I2C Brick");
    axis_OK = true;
  }
  else
  {
    SerialCom->println("FaBo 9Axis device error");
    axis_OK = false;
  }
#endif

  delay(1000); //settling time but no really needed
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state)
  {
  case INITIALISING:
    machine_state = initialising();
    break;
  case RUNNING: //Lipo Battery Volage OK
    machine_state = running();
    break;
  case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
    machine_state = stopped();
    break;
  };
}

STATE initialising()
{
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running()
{
  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500)
  { //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_MPU
    if (axis_OK)
      MPU9250_reading();
#endif

#ifndef NO_HC - SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK())
      return STOPPED;
#endif
  }

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped()
{
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;

  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500)
  { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
    SerialCom->println("Please Re-charge Lipo");
#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK())
    {
      SerialCom->print("Lipo OK Counter:");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10)
      { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    }
    else
      counter_lipo_voltage_ok = 0;
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160)
  {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    SerialCom->print(" : Raw Lipo:");
    SerialCom->println(raw_lipo);
    Low_voltage_counter = 0;
    return true;
  }
  else
  {
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

#ifndef NO_HC - SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST)
  {
    SerialCom->println("HC-SR04: Out of range");
  }
  else
  {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_MPU
void MPU9250_reading()
{
  float ax, ay, az;

  fabo_9axis.readAccelXYZ(&ax, &ay, &az);
  SerialCom->print("MPU ax:");
  SerialCom->println(ax);
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available())
  {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val)
    {
    case 'w': //Move Forward
    case 'W':
      forward();
      SerialCom->println("Forward");
      break;
    case 's': //Move Backwards
    case 'S':
      reverse();
      SerialCom->println("Backwards");
      break;
    case 'q': //Turn Left
    case 'Q':
      strafe_left();
      SerialCom->println("Strafe Left");
      break;
    case 'e': //Turn Right
    case 'E':
      strafe_right();
      SerialCom->println("Strafe Right");
      break;
    case 'a': //Turn Right
    case 'A':
      ccw();
      SerialCom->println("ccw");
      break;
    case 'd': //Turn Right
    case 'D':
      cw();
      SerialCom->println("cw");
      break;
    case '-': //Turn Right
    case '_':
      speed_change = -100;
      SerialCom->println("-100");
      break;
    case '=':
    case '+':
      speed_change = 100;
      SerialCom->println("+");
      break;
    default:
      stop();
      SerialCom->println("stop");
      break;
    }
  }
}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
