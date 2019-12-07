#include <Servo.h>

const int servoPin1 = 2;
const int servoPin2 = 3;
const int motor1[4] = { 4, 5, 6, 7 };
const int motor2[4] = { 8, 9, 10, 11 };

const int LIFT_DOWN = 70;
const int LIFT_UP = 140;
const int HANDLE_OPEN = 90;
const int HANDLE_CLOSE = 180;



const int MOTOR_STATE_STOPPED = 0;
const int MOTOR_STATE_FORWARD = 1;
const int MOTOR_STATE_BACKWARD = -1;

int rMotorState = MOTOR_STATE_STOPPED;
int lMotorState = MOTOR_STATE_STOPPED;

const String API_STEP_RIGHT = "r";
const String API_STEP_LEFT = "l";
const String API_MOVE = "m";
const String API_STOP = "s";
const String API_CATCH = "c";
const String API_DROP = "d";
const String API_CAN_CATCH_OBJECT = "o";



void MotorStep(int ms)
{
  if (rMotorState == MOTOR_STATE_STOPPED)
  {
    digitalWrite(motor2[1], false);
    digitalWrite(motor2[3], false);
    digitalWrite(motor2[0], false);
    digitalWrite(motor2[2], false);
  }
  else if (rMotorState == MOTOR_STATE_FORWARD)
  {
    digitalWrite(motor2[1], false);
    digitalWrite(motor2[3], false);
    digitalWrite(motor2[0], true);
    digitalWrite(motor2[2], true);
  }
  else if (rMotorState == MOTOR_STATE_BACKWARD)
  {
    digitalWrite(motor2[1], true);
    digitalWrite(motor2[3], true);
    digitalWrite(motor2[0], false);
    digitalWrite(motor2[2], false);
  }

  if (lMotorState == MOTOR_STATE_STOPPED)
  {
    digitalWrite(motor1[1], false);
    digitalWrite(motor1[3], false);
    digitalWrite(motor1[0], false);
    digitalWrite(motor1[2], false);
  }
  else if (lMotorState == MOTOR_STATE_FORWARD)
  {
    digitalWrite(motor1[1], true);
    digitalWrite(motor1[3], true);
    digitalWrite(motor1[0], false);
    digitalWrite(motor1[2], false);
  }
  else if (lMotorState == MOTOR_STATE_BACKWARD)
  {
    digitalWrite(motor1[1], false);
    digitalWrite(motor1[3], false);
    digitalWrite(motor1[0], true);
    digitalWrite(motor1[2], true);
  }

  delay(ms);
}


int xxxx = 0;

//Lift
Servo Servo1;


//catch
Servo Servo2;
void setup() {
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);

  Serial.begin(115200);
//  pinMode(servoPin1, OUTPUT);
//  pinMode(servoPin2, OUTPUT);

  for (int i = 0; i < 4; ++i)
  {
    pinMode(motor1[i], OUTPUT);
    pinMode(motor2[i], OUTPUT);
  }
}

void loop() 
{
  MotorStep(1);
  
  String msg = "z";
  char data;
  if (Serial.available())
  {
    msg = "";
    while (true)
    {
      data = Serial.read();
      if (data == '\n')
        break;
      msg += data;
    }
    if (msg.startsWith("q"))
    {
      delay(5000);
    }
    if (msg.startsWith(API_STEP_RIGHT))
    {
      int oldrMotorState = rMotorState;
      int oldlMotorState = lMotorState;


      rMotorState = MOTOR_STATE_BACKWARD;
      lMotorState = MOTOR_STATE_FORWARD;
      MotorStep(250);

      rMotorState = oldrMotorState;
      lMotorState = oldlMotorState;
      MotorStep(250);
    }
    else if (msg.startsWith(API_STEP_LEFT))
    {
      int oldrMotorState = rMotorState;
      int oldlMotorState = lMotorState;


      rMotorState = MOTOR_STATE_FORWARD;
      lMotorState = MOTOR_STATE_BACKWARD;
      MotorStep(250);

      rMotorState = oldrMotorState;
      lMotorState = oldlMotorState;
      MotorStep(250);
    }
    else if (msg.startsWith(API_MOVE))
    {
      rMotorState = MOTOR_STATE_FORWARD;
      lMotorState = MOTOR_STATE_FORWARD;
      MotorStep(250);
      rMotorState = MOTOR_STATE_STOPPED;
      lMotorState = MOTOR_STATE_STOPPED;
      MotorStep(1);
    }
    else if (msg.startsWith(API_STOP))
    {
      rMotorState = MOTOR_STATE_STOPPED;
      lMotorState = MOTOR_STATE_STOPPED;
      MotorStep(250);
    }
    else if (msg.startsWith(API_CATCH))
    {
      float xx = atof(msg.substring(API_CATCH.length()).c_str());
      Servo1.write(LIFT_DOWN + (LIFT_UP - LIFT_DOWN) * xx); 
      Servo2.write(HANDLE_CLOSE);
    }
    else if (msg.startsWith(API_DROP))
    {
      Servo2.write(HANDLE_OPEN);
    }
    else if (msg.startsWith(API_CAN_CATCH_OBJECT))
    {
      float xx = atof(msg.substring(API_CAN_CATCH_OBJECT.length()).c_str());
      Servo1.write(LIFT_DOWN + (LIFT_UP - LIFT_DOWN) * xx); 
      xxxx++;
      if (xxxx > 10)
      {
        Serial.println("t");
      }
      else
      {
        Serial.println("f" + String(xxxx));
      }
    }
  }

  return;
  
  moveRight(true);
  moveLeft(true);
  delay(3000);
  stopRight();
  stopLeft();
  delay(3000);
  moveRight(true);
  moveLeft(false);
  delay(3000);
  stopRight();
  stopLeft();
  delay(3000);
  moveRight(false);
  moveLeft(true);
  delay(3000);
  stopRight();
  stopLeft();
  
  delay(9999999);
}


void stopRight()
{
  digitalWrite(motor2[1], false);
  digitalWrite(motor2[3], false);
  digitalWrite(motor2[0], false);
  digitalWrite(motor2[2], false);
}
void moveRight(bool forward)
{
  digitalWrite(motor2[1], !forward);
  digitalWrite(motor2[3], !forward);
  digitalWrite(motor2[0], forward);
  digitalWrite(motor2[2], forward);
}


void stopLeft()
{
  digitalWrite(motor1[1], false);
  digitalWrite(motor1[3], false);
  digitalWrite(motor1[0], false);
  digitalWrite(motor1[2], false);
}
void moveLeft(bool forward)
{
  digitalWrite(motor1[1], forward);
  digitalWrite(motor1[3], forward);
  digitalWrite(motor1[0], !forward);
  digitalWrite(motor1[2], !forward);
}


