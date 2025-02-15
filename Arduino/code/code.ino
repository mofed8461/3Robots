#include <Servo.h>

const int servoPin1 = 2;
const int servoPin2 = 3;
const int motor1[4] = { 4, 5, 6, 7 };
const int motor2[4] = { 8, 9, 10, 11 };

const int MOTOR_STATE_STOPPED = 0;
const int MOTOR_STATE_FORWARD = 1;
const int MOTOR_STATE_BACKWARD = -1;


const int rMotorState = MOTOR_STATE_STOPPED;
const int lMotorState = MOTOR_STATE_STOPPED;

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



//lift
const int LIFT_DOWN = 70;
const int LIFT_UP = 140;
Servo Servo1;

//catch
const int HANDLE_OPEN = 90;
const int HANDLE_CLOSE = 180;
Servo Servo2;
void setup() {
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);

  Serial.begin(9600);
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
  char data;
  if (Serial.available())
  {
    data = Serial.read();
    Serial.println(data);
  }
  
  Servo1.write(LIFT_UP); 
  Servo2.write(HANDLE_OPEN); 
  delay(1000); 
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
