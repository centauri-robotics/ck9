/***
   DC ROS Serial Communication Code
   Author: Jash Mota
   Company: Centauri Robotics
***/
#include <ros.h>
#include <std_msgs/Float32.h>
#include "Arduino.h"

ros::NodeHandle nh;
// Left encoder
int Left_Encoder_PinA = 2;
int Left_Encoder_PinB = 9;
volatile long Left_Encoder_Ticks = 0;
//Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;

//Right Encoder
int Right_Encoder_PinA = 3;
int Right_Encoder_PinB = 10;
volatile long Right_Encoder_Ticks = 0;
//Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;

// Left Motor
int enA = 6; //3
int dir1 = 5;

// Right Motor
int enB = 11; //9
int dir2 = 8;

//Initialising with defaults to keep the motors stopped
bool leftdir = 1, rightdir = 1;
int leftanalog = 255, rightanalog = 255;

//float lwheelv = 0;//left wheel velocity command
//float rwheelv = 0;//right wheel velocity from twist_to_motors

//Returns an Analog value for the given speed from twist to motors script
int speedConv(float x, float in_min, float in_max, float out_min, float out_max)
{
  int analogval = (int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min));
  if (analogval < 0)
    analogval = 0;
  else if (analogval > 255)
    analogval = 255;
  return (analogval);
}

//Sets Direction and Speed value for left motor
void setleft(float val)
{
  if (val < 0.0)
  {
    leftdir = 0;
    val = val * (-1);
    leftanalog = speedConv(val, 0.0, 0.5, 0.0, 255.0);
  }
  else
  {
    leftdir = 1;
    leftanalog = speedConv(val, 0.0, 0.5, 255.0, 0.0);
  }
}

//Sers Direction and Speed value for left motor
void setright(float val)
{
  if (val < 0.0)
  {
    rightdir = 0;
    val = val * (-1);
    rightanalog = speedConv(val, 0.0, 0.5, 0.0, 255.0);
  }
  else
  {
    rightdir = 1;
    rightanalog = speedConv(val, 0.0, 0.5, 255.0, 0.0);
  }
}

//Runs the left motor with above direction and speed values
void leftmotor(const std_msgs::Float32 data) // 1 for clcckwise 2 for anti clockwise
{
  float lwheelv = 0.0;
  lwheelv = data.data;
  setleft(lwheelv);
  if (leftdir)
  {
    digitalWrite(dir1, HIGH);
  }
  else if (!leftdir)
  {
    digitalWrite(dir1, LOW);
  }
  else
  {
    //Serial.println(F("Wrong direction set for MA!!"));
  }
  analogWrite(enA, leftanalog);
}

//Runs the right motor with above direction and speed values
void rightmotor(const std_msgs::Float32 data) // 1 for clockwise 2 for anti clockwise
{
  float rwheelv = 0.0;
  rwheelv = data.data;
  setright(rwheelv);
  if (rightdir)
  {
    digitalWrite(dir2, HIGH);
  }
  else if (!rightdir)
  {
    digitalWrite(dir2, LOW);
  }
  else
  {
    //Serial.println(F("Wrong direction set for MB!!"));
  }
  analogWrite(enB, rightanalog);
}

std_msgs::Float32 a;
std_msgs::Float32 b;
//Publish steps of left and right stepper motors
ros::Publisher pub1("lwheel", &a);
ros::Publisher pub2("rwheel", &b);

//Publish steps of left and right stepper motors
//Subscribe to left and right wheel velocity topics
ros::Subscriber<std_msgs::Float32> subl("left_wheel_speed", &leftmotor);
ros::Subscriber<std_msgs::Float32> subr("right_wheel_speed", &rightmotor);

void setup()
{
  //Initialise node and subscribe to necessary topics
  // nh.getHardware()->setBaud(38400);
  nh.initNode();
  nh.subscribe(subl);
  nh.subscribe(subr);
  nh.advertise(pub1);
  nh.advertise(pub2);
  Serial.begin(57600);
  //  Serial.begin(38400);
  SetupEncoders();
  //Define motor pins as output pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT); // sets pin A pullup
  pinMode(Left_Encoder_PinB, INPUT); // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT); // sets pin A pullup
  pinMode(Right_Encoder_PinB, INPUT); // sets pin B pullup

  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING);
}

void loop()
{
  a.data = Left_Encoder_Ticks;
  b.data = Right_Encoder_Ticks;

  pub1.publish(&a);
  pub2.publish(&b);

  nh.spinOnce();
  //  delayMicroseconds(10);
  delay(10);
}

void do_Left_Encoder()
{
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB); // read the input pin
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
}
void do_Right_Encoder()
{
  RightEncoderBSet = digitalRead(Right_Encoder_PinB); // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? +1 : -1;
}
