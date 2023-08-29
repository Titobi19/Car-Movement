#define outputA;
#include<Servo.h>
Servo motor;
#include <MPU6050_tockn.h>
#include <Wire.h>
#include<math.h>
MPU6050 mpu6050(Wire);
int p = 0;
int in1 = 5;
int in2 = 6;
int in3 = 7;
int in4 = 8;
int ena = A1;
int enb = A2;
int trig = 12;
int echo = 11;
volatile int count = 0;
float wheelDiameter = 6.6;
float numberOfSlots = 20;
float distancePerTick = PI * wheelDiameter / numberOfSlots;
float distance;
float theta = 0.0000;
float angle_offset = 0.0000;

void setup() {
  // put your setup code here, to run once:
  motor.attach(9);
  motor.write(180);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting Wheel Encoder");
  attachInterrupt(digitalPinToInterrupt (2), readEncoder, CHANGE);
  for (int i = 0; i < 1000; i++) {
    mpu6050.update();

    angle_offset += mpu6050.getAngleZ();
  }
  angle_offset = angle_offset / 1000.00;
  Serial.println("");
  Serial.print("offset angle is: ");
  Serial.println(angle_offset);

}

void loop() {
  // put your main code here, to run repeatedly:
  goFoward();
  delay(4000);
  turnRight();
  delay(2000);
  getAngle();
  if (theta = 180) {
    goFoward();
  }
  digitalWrite(trig, HIGH);
  delay(2);
  digitalWrite(trig, LOW);
  int period = pulseIn(echo, HIGH);
  int distance = (period / 2) / 29.1;
  Serial.print(" Distance = ");
  Serial.println(distance);
  distance = count * distancePerTick;
  Serial.print(distance);
  for (p = 0; p > 180; p++) {
    motor.write(p);
    delay(10);
  }
  for (p = 180 ; p > 0; p--) {
    motor.write(p);
    delay(10);
  }

}
void readEncoder() {
  count++;
}
void turnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, 150);
}
void turnLeft() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(ena, 150);
}
void getAngle() {
  mpu6050.update();
  float theta = 90 + mpu6050.getAngleZ() - angle_offset;
  theta = radians(theta);
  float aa = int(100 * theta) % 628;
  theta = aa / 100;
  theta = degrees(theta);
  return theta;
}
void goFoward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
