#include <Servo.h>
#include <math.h>

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

HardwareSerial *SerialCom;

//Default sensor pins
const byte left_front_sensor = A7;
const byte left_back_sensor = A4;
const byte back_sensor = A11;
const byte front_sensor = A15;

int speed_val;
void setup(void){
  stop();
  SerialCom = &Serial;
  SerialCom ->begin(115200);
  SerialCom ->println("Starting");
  enable_motors();
  orientation();
  stop();
}
void loop(void){}

float left_back_dist (void){
  //Derived formula
  int ADCval = analogRead(left_back_sensor);
  float dist = 1/(0.0005*ADCval - 0.0078);   //11.724/ADCval *(5/1023.0) - 0.24;
  return dist; //in cm
}

float left_front_dist (void){
  //Derived formula
  int ADCval = analogRead(left_front_sensor);
  float dist = 1/(0.0005*ADCval - 0.0101);   //11.724/ADCval *(5/1023.0) - 0.24;
  return dist; //in cm
}

float front_dist (void){
  //Derived formula
  int ADCval = analogRead(front_sensor);
  float dist = 1/(0.0002*ADCval - 0.0068);  //20.9/ADCval *(5/1023.0);
  return dist; //in cm
}

float back_dist (void){
  //Derived formula
  int ADCval = analogRead(back_sensor);
  float dist = 1/(0.0002*ADCval - 0.0051);   //20.9/ADCval *(5/1023.0);
  return dist; //in cm
}

//INITIALISING
void orientation (void) {
  float separationDist = 19;
  float leftFrontDist, leftBackDist, dist; 
  float error = 6.2;
  float angle = 3.14/2;

  int ccwTurn, strafeRight, frontControl, rearControl, tinit;
  int ccwGain = 1500;
  int strafeGain = 50;
  int t = 0;
  
  while(((abs(angle) > 0.0349) || (abs(error) > 0.2)) && (t < 2000)){
    leftFrontDist = left_front_dist();
    leftBackDist = left_back_dist();
    //distance from the wall
    dist = (leftFrontDist + leftBackDist)/2;
    //approximate sin theta to theta
    angle = (leftFrontDist - leftBackDist)/separationDist;

    if ((angle < 3.14/8) && (angle > -3.14/8)) {
      //Reference angle is 0, control signal for turning CCW
      ccwTurn = angle * ccwGain;
      //6.2 = 15 - 8.8 (dist from centre of robot to sensor)
      error = 6.2 - dist;
      //Control signal for strafing right
      strafeRight = error * strafeGain ;
      strafeRight = (strafeRight > 500) ? 500 : strafeRight;
    } else{
      ccwTurn = 500;
      strafeRight = 0;
    }

    //Control signal for front motors
    frontControl = (-ccwTurn + strafeRight > 500) ? 500 : -ccwTurn + strafeRight;
    frontControl = (-ccwTurn + strafeRight < -500) ? -500 : frontControl;

    //Control signal for rear motors
    rearControl = (-ccwTurn - strafeRight > 500) ? 500 : -ccwTurn - strafeRight;
    rearControl = (-ccwTurn - strafeRight < -500) ? -500 : rearControl;
    
    left_font_motor.writeMicroseconds(1500 + frontControl);
    left_rear_motor.writeMicroseconds(1500 + rearControl);
    right_rear_motor.writeMicroseconds(1500 + rearControl);
    right_font_motor.writeMicroseconds(1500 + frontControl);
    
    SerialCom -> print("angle = ");
    SerialCom -> println(angle);
    SerialCom -> print(", error = ");
    SerialCom -> println(error);
    SerialCom -> println(t);
    
    //If the control signals are low for too long, start the timer to quit so that the motors don't burn out
    if ((abs(strafeRight) < 50) && (abs(ccwTurn) < 50)){
      t = t + millis() - tinit;
    } else {
      t = 0;
    }
  }
}

void set_gyro(void){
  
}

//RUNNING
void forward(void){
  
}


void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void cw(){
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void left(){
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}
