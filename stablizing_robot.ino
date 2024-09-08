/*
Final presentation arduino code
Jay 31
Nov 26, 2023
Stablizing/balancing robot
Program to self balance a two wheeled robot by driving two 
motors based on input from MPU6050 gyroscope/accelerometer.
Also will avoid obstacles using ultra sonic distance sensor.
Original Code by midhun_s from:
https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define leftMotorPWMPin 6
#define leftMotorDirPin 7
#define rightMotorPWMPin 5
#define rightMotorDirPin 4

#define Kp 12
#define Ki 1
#define Kd 0
#define sampleTime 0.005
#define targetAngle 1

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0, dError;
volatile byte count = 0;

void setMotors(int leftMotorSpeed, int rightMotorSpeed){
  if(leftMotorSpeed >= 0){
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else{
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }

  if(rightMotorSpeed >= 0){
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else{
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

void init_PID(){
  // initialize Timer1
  cli(); // disable global interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  //turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable time compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  // set the motor control and PWN pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  // set the status LED to output mode
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(-2551);
  mpu.setZAccelOffset(1099);
  mpu.setXGyroOffset(-22);

  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

  // set motor power after constraining it
  setMotors(-motorPower, motorPower);

  Serial.print("Current angle: "); Serial.print(currentAngle);
  Serial.print("\terror: "); Serial.print(error);
  Serial.print("\terror sum: "); Serial.print(errorSum);
  Serial.print("\tMotor Power: "); Serial.println(motorPower);
}

// ISR called every 5 milliseconds
ISR(TIMER1_COMPA_vect) {

  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);

  // calculate error and sum of errors
  error = targetAngle - currentAngle;
  errorSum += (Ki*error);
  errorSum = constrain(errorSum, -300, 300);
  dError = (error - prevError);

  // calculate output from P, I and D values
  motorPower = Kp*error + errorSum*sampleTime + Kd*dError;
  motorPower = constrain(motorPower, -255, 255);

  prevError = error;
  prevAngle = currentAngle;

  // toggle the led on pin12 every second
  count++;
  if(count == 100){
    count = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
