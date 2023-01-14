#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;


// increase kp’s value and see what happens
float kp = 38;//45  
float ki = 0.001;//0.01
float kd = 110;//320



int p = 0;
int i = 0;
int d = 0;

float error = 0;
float lastError = 0;

const int maxSpeed = 210;
const int minSpeed = -255;

const int baseSpeed = 210;

int motorSpeed;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};
void setup() {

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  // calibrate the sensor. For maximum grade the line follower should do the movement itself, without human interaction.
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
    // do motor movement here, with millis() as to not ruin calibration)
    //CalibrateMovement()
    if ( 0 <= i && i < 15   )  leftFunc();
    if ( 15 <= i && i < 30   )  stopFunc();
    if ( 30 <= i && i  < 45   )  rightFunc();
    if ( 45 <= i && i < 60   )  stopFunc();
    if ( 60 <= i && i < 75   )  leftFunc();
    if ( 75 <= i && i < 90   )  stopFunc();
    if ( 90 <= i && i < 105  )  rightFunc();
    if ( 105 <= i && i < 130  )  stopFunc();
    if ( 130 <= i && i < 145   )  leftFunc();
    if ( 145 <= i && i < 160  )  stopFunc();
    if ( 160 <= i && i < 175   )  rightFunc();
    if ( 175 <= i && i < 190  )  stopFunc();

    if ( i >= 190  )  {
      stopFunc();
    }
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
   
}
float speed = 255;
void rightFunc() {
  setMotorSpeed(speed,0);
}
void leftFunc() {
  setMotorSpeed(-speed+15,0);
}
void stopFunc() {
  setMotorSpeed(0,0);
}
void CalculatePIDValues() {
  lastError = error;
  error = map(qtr.readLineBlack(sensorValues), 0, 5000, -70, 70);

  p = error;
  i = i + error;
  d = error - lastError;
}

void loop() {
  CalculatePIDValues();
  pidControl(kp,ki,kd);
  CalculateMotorSpeed();
  setMotorSpeed(m1Speed, m2Speed);

  
//  DEBUGGING
//  Serial.print("Error: ");
//  Serial.println(error);
//  Serial.print("M1 speed: ");
//  Serial.println(m1Speed);
//
//  Serial.print("M2 speed: ");
//  Serial.println(m2Speed);
//
//  delay(250);
}

void CalculateMotorSpeed() {
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < 0) {
    m1Speed += motorSpeed;
  }
  else if (error > 0) {
    m2Speed -= motorSpeed;
  }
  // make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed properly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making a steep turn
  m1Speed = constrain(m1Speed, 0, maxSpeed);
  m2Speed = constrain(m2Speed, 0, maxSpeed);
}
float lastMove = 0 ;
float moveDelay = 300;
bool movingLeft = true;

float startMove = 0;
int state = 1;
void CalibrateMovement(){
  if(millis() - lastMove > moveDelay ) {
      
      lastMove = millis();
      if(state == 1) {
        setMotorSpeed(speed,-speed);
        state = 2;
      }
      else if(state == 2) {
        setMotorSpeed(-speed,speed);
        state = 3;
      }
      else if(state == 3) {
        setMotorSpeed(-speed,speed);
        state = 4;
      }
      else if(state == 4) {
        setMotorSpeed(speed,-speed);
        state = 1;
      }

  }
  
}


// calculate PID value based on error, kp, kd, ki, p, i and d.
void pidControl(float kp, float ki, float kd) {
  motorSpeed = kp * p + ki * i + kd * d; // = error in this case
}


// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse 
   motor1Speed = -motor1Speed;
   motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  }
  else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  }
  else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
