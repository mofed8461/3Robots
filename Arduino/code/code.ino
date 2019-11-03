const int servoPin1 = 2;
const int servoPin2 = 3;
const int motor1[4] = { 4, 5, 6, 7 };
const int motor2[4] = { 8, 9, 10, 11 };


void setup() {
  pinMode(servoPin1, OUTPUT);
  pinMode(servoPin2, OUTPUT);

  for (int i = 0; i < 4; ++i)
  {
    pinMode(motor1[i], OUTPUT);
    pinMode(motor2[i], OUTPUT);
  }
}

void loop() {
  
}
