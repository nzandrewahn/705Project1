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

//byte long1 = 1;
//byte short1 = 1;
//byte long2 = 1;
//byte short2 = 1;
//byte front, back, left, right, left_front, left_rear, right_rear, right_front;
int speed_val;
void setup(void){
  stop();
  SerialCom = &Serial;
  SerialCom ->begin(115200);
  SerialCom ->println("Starting");
  enable_motors();
  orientation2();
  stop();
}
void loop(void){}

float left_back_dist (void){
  //Replace with derived formula
  int ADCval = analogRead(left_back_sensor);
  float dist = 1/(0.0005*ADCval - 0.0078);   //11.724/ADCval *(5/1023.0) - 0.24;
  return dist; //in cm
}

float left_front_dist (void){
  //Replace with derived formula
  int ADCval = analogRead(left_front_sensor);
  float dist = 1/(0.0005*ADCval - 0.0101);   //11.724/ADCval *(5/1023.0) - 0.24;
  return dist; //in cm
}

float front_dist (void){
  //Replace with derived formula
  int ADCval = analogRead(front_sensor);
  float dist = 1/(0.0002*ADCval - 0.0068);  //20.9/ADCval *(5/1023.0);
  return dist; //in cm
}


float back_dist (void){
  //Replace with derived formula
  int ADCval = analogRead(back_sensor);
  float dist = 1/(0.0002*ADCval - 0.0051);   //20.9/ADCval *(5/1023.0);
  return dist; //in cm
}


//INITIALISING
//void orientation (){
//  // Check readings, one will be max range, lowest voltage as voltage is inverse to distance
//  float diff = 10;
//  float gain = 10000;
//  float left_gain = 100;
////  int long1ADC = analogRead(long1);
////  int long2ADC = analogRead(long2);
////  int short1ADC = analogRead(short1);
////  int short2ADC = analogRead(short2);
//
////  minLong =  long1ADC < long2ADC ? long1 : long2;
////  minShort = short1ADC < short2ADC ? short1 : short2;
//
//  //Set the orientation
//  //Adjust thresholds
///*  if ((short1ADC < 100) || (long1ADC < 100)){
//    front = long1;
//    back = long2;
//    left = short2;
//    right = short1;
//    left_front = 46;
//    left_rear = 47;
//    right_rear = 50;
//    right_front = 51;
//  } else {
//    front = long2;
//    back = long1;
//    left = short1;
//    right = short2;
//    left_front = 50;
//    left_rear = 51;
//    right_rear = 46;
//    right_front = 47;
//  }*/
//  
//  float priorBackDist = back_dist();
//  float priorLeftDist = left_dist();
//  float leftDist;
//  speed_val = 200;
//
//  while((diff > 0.5*2) || (diff < -0.5*2)){
//    cw();
//    delay(100);
//    float backDist = back_dist();
//    leftDist = left_dist();
//
//    diff = priorBackDist + priorLeftDist - backDist - leftDist;
//    speed_val = gain * diff;
//    priorBackDist = backDist;
//    priorLeftDist = leftDist;
//    SerialCom ->println(diff);
//  }
//  SerialCom ->println("Moving left");
//
//  while((leftDist > 15.2) || (leftDist < 14.8)){
//    left();
//    delay(10);
//    leftDist = left_dist();
//    SerialCom ->println(leftDist);
//    speed_val = left_gain * (leftDist - 15);
//  }
//  stop();
//}

void orientation2 (void) {
  float separationDist = 19;
  float leftFrontDist, leftBackDist, dist; 
  float error = 6.2;
  float angle = 3.14/2;

  int ccwturn, strafeRight;
  int ccwgain = 1500;
  int strafeGain = 50;
  int t = 0;
  while(((angle > 0.0349) && (error > 0.5)) || (t < 2000)){
    
    leftFrontDist = left_front_dist();
    leftBackDist = left_back_dist();
    dist = (leftFrontDist + leftBackDist)/2;
    //approximate sin theta to theta
    angle = (leftFrontDist - leftBackDist)/separationDist;

    if ((angle < 3.14/8) && (angle > -3.14/8)) {
      //Reference angle is 0
      ccwturn = angle * ccwgain;
      //6.2 = 15 - 8.8 (dist from centre of robot to sensor)
      error = 6.2 - dist;
      strafeRight = error * strafeGain ;
      strafeRight = (strafeRight > 500) ? 500 : strafeRight;
    } else{
      ccwturn = 500;
      strafeRight = 0;
    }
    
    left_font_motor.writeMicroseconds(1500 - ccwturn + strafeRight);
    left_rear_motor.writeMicroseconds(1500 - ccwturn - strafeRight);
    right_rear_motor.writeMicroseconds(1500 - ccwturn - strafeRight);
    right_font_motor.writeMicroseconds(1500 - ccwturn + strafeRight);
    SerialCom -> print("angle = ");
    SerialCom -> println(angle);
    SerialCom -> print(", error = ");
    SerialCom -> println(error);
    if((abs(ccwturn) > 20) ||(abs(strafeRight) > 20)){
      t = millis() - t;
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
