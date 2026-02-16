#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

const int room = 1120;
const int trig = 0;
const int echo = 1;

void setup() {
    Enes100.begin("Cryan me a River", WATER, 123, room, 8, 9);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
}

int sign(float number) {
    if (number >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

void setMotorPow(bool side, int dir, int power) {
  int pinEN;
  int pinain;
  int pinbin;
  if (side == 1) {
    pinain = 2;
    pinbin = 7;
    pinEN = 10;
  }
  else {
    pinain = 12;
    pinbin = 13;
    pinEN = 11;
  }

  int inA = (1 + sign(dir)) / 2;
  int inB = (1 - sign(dir)) / 2;
  digitalWrite(pinain, inA);
  digitalWrite(pinbin, inB);
  analogWrite(pinEN, power);
}

void stopMotors() {
  setMotorPow(0, 1, 0);
  setMotorPow(1, 1, 0);
}

void directionCtrl(float desAngle) {
  int PWR;
  while (PWR != 2) {
    float turnDist = desAngle - Enes100.getTheta();
    PWR = (2.8 + 60 * (abs(turnDist)));
    setMotorPow(0, turnDist, PWR);
    setMotorPow(1, -turnDist, PWR);
  }
  stopMotors();
}

void moveToX(float x) {
  float Xdist = x - Enes100.getX();
  if (Xdist > 0){
    directionCtrl(0);
  }
  else {
    directionCtrl(PI);
  }
  while (abs(Xdist) > 0.015) {
    Xdist = x - Enes100.getX();
    int PWR = abs(50 * Xdist) + 10;
    setMotorPow(1, 1, PWR);
    setMotorPow(0, 1, PWR);
  }
  stopMotors();
}

void moveToY(float y) {
  float Ydist = y - Enes100.getY();
  if (Ydist > 0) {
    directionCtrl((PI / 2));
  }
  else {
    directionCtrl(-(PI / 2));
  }
  while (abs(Ydist) > 0.015) {
    Ydist = y - Enes100.getY();
    int PWR = abs(50 * Ydist) + 10;
    setMotorPow(1, 1, PWR);
    setMotorPow(0, 1, PWR);
  }
  stopMotors();
}

float pulse() {
  int dur;
  float pulsDis;
  Enes100.println("Pulsing:");
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  dur = pulseIn(echo, HIGH);
  pulsDis = (dur*.0343)/2;
  Enes100.print("(Dist. Measured = ");
  Enes100.print(pulsDis);
  Enes100.println(") ");
  return pulsDis;
}

void obstCh(float xCol) {
  Enes100.println("Sucessfully reached viewing column!");
  moveToX(xCol);
  while (1 == 1) {
    moveToY(1.5);
    directionCtrl(0);
    if (pulse() < 30) {
      return;
    }
    Enes100.print("detection / unkn at pos 1, continuing down... ");
    moveToY(1.0);
    directionCtrl(0);
    if (pulse() < 30) {
      return;
    }
    Enes100.print("again, at pos 2... ");
    moveToY(0.5);
    directionCtrl(0);
    if (pulse() < 30) {
      return;
    }
    Enes100.println("detection / unkn at pos 3, looping");
  }
}

float apprDis = 0.13;

void loop() {
  Enes100.println("Started!");
  delay(5000);
  if (Enes100.getY() < 1) {
    moveToY(1.5 - apprDis);
  }
  else {
    moveToY(0.5 + apprDis);
  }

  delay(10000);
  obstCh(1.2);
  Enes100.println("Passing obstacles at column 1");
  obstCh(2.0);
  Enes100.println("Passing obstacles at column 2");
  moveToX(2.9);
  moveToY(1.5);
  moveToX(3.75);
  Enes100.println("Limbo Passed!");
  Enes100.println("Ending action.");
  return;
}
