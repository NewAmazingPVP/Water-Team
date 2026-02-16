#include <Enes100.h>

#include <Servo.h>

#include <NewPing.h>

#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

Servo ArmServo;

int pos = 0;
int inital_pos = 0;

const int TrigPin = 0;
const int EchoPin = 1;
const int MAX_DISTANCE = 50;

#define SONAR_NUM     1
#define PING_INTERVAL 33

NewPing sonar(TrigPin, EchoPin, MAX_DISTANCE);

#define s2 5
#define s3 6
#define out 7

bool Pollutants = false;

int  Red=0, Blue=0, Green=0;

int sign(float number) {
    if (number >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

void setup() {
    Serial.begin(9600);

    Enes100.begin("CRyan Me A River", WATER, 12, 1116, 8, 9);

    Enes100.println("Connected...");
    Tank.begin();

    ArmServo.attach(9);

    pinMode(s0,OUTPUT);
    pinMode(s1,OUTPUT);
    pinMode(s2,OUTPUT);
    pinMode(s3,OUTPUT);
    pinMode(out,INPUT);

    Serial.begin(9600);

    digitalWrite(s0,HIGH);
    digitalWrite(s1,HIGH);
}

void loop() {
    float x, y, t; bool v;

    x = Enes100.getX();
    y = Enes100.getY();
    t = Enes100.getTheta();
    v = Enes100.isVisible();

    delay(50);
    unsigned int uS = sonar.ping();

    GetColors();

    if (Red<Blue && Red<=Green && Red<78)  {
        Serial.println("Red");
    }

    else {
        Serial.println("Unknown");
    }
    if (Red<Blue && Red<=Green && Red<78){
        Serial.println("Pollutants Detected");
    }
    delay(2000);

    for (pos = 0; pos = 180; pos += 1) {
      ArmServo.write(pos);
      delay (15000);
      inital_pos = 0;
      ArmServo.write(inital_pos);
      delay (10000);

    }

}

    if (v)
    {
        Enes100.print(x);
        Enes100.print(",");
        Enes100.print(y);
        Enes100.print(",");
        Enes100.println(t);
    }
    else {
        Enes100.println("Not visible");
    }

    float Yval = Enes100.getY();
    if (Yval > 1) {
        directionCtrl(-(PI / 2));
    }
    else {
        directionCtrl(PI / 2);
    }
    Enes100.println("Setup Done!");
    moveToX(1.0);
    moveToY(1.0);
    Enes100.println("Navigation Done!");
    delay(10000);

    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);

    Enes100.mission(DEPTH, 30);
    delay(1000);

    int Sensor_WaterLV=analogRead(A0);

    int Water_Level_ml = map(Sensor_WaterLV, 0, 150, 0, 40);

    Serial.println(Water_Level_ml);
    delay(30);

    void directionCtrl(float desAngle) {
        while (1 != 0) {
            float turnDist = desAngle - (Enes100.getTheta());
            int PWR = (sign(turnDist)) * (2.8 + 60 * (abs(turnDist)));
            int aPWR = abs(PWR);
            Tank.setLeftMotorPWM(-PWR);
            Tank.setRightMotorPWM(PWR);
            Enes100.print("to go: ");
            Enes100.print(turnDist);
            Enes100.print(" current motor power: ");
            Enes100.println(aPWR);
            if (aPWR == 2) {
                Tank.setLeftMotorPWM(0);
                Tank.setRightMotorPWM(0);
                return;
            }
        }

    return;
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
        Tank.setLeftMotorPWM(PWR);
        Tank.setRightMotorPWM(PWR);
    }
    Tank.turnOffMotors();
    return;
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
        Tank.setLeftMotorPWM(PWR);
        Tank.setRightMotorPWM(PWR);
    }
    Tank.turnOffMotors();
    return;
}

}

void directionCtrl(float desAngle) {
    while (1 != 0) {
        float turnDist = desAngle - (Enes100.getTheta());
        int PWR = (sign(turnDist)) * (2.8 + 60 * (abs(turnDist)));
        int aPWR = abs(PWR);
        Tank.setLeftMotorPWM(-PWR);
        Tank.setRightMotorPWM(PWR);
        Enes100.print("to go: ");
        Enes100.print(turnDist);
        Enes100.print(" current motor power: ");
        Enes100.println(aPWR);
        if (aPWR == 2) {
            Tank.setLeftMotorPWM(0);
            Tank.setRightMotorPWM(0);
            return;
        }
    }
    return;
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
        Tank.setLeftMotorPWM(PWR);
        Tank.setRightMotorPWM(PWR);
    }
    Tank.turnOffMotors();
    return;
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
        Tank.setLeftMotorPWM(PWR);
        Tank.setRightMotorPWM(PWR);
    }
    Tank.turnOffMotors();
    return;
}

void GetColors()
{
  digitalWrite(s2,  LOW);
  digitalWrite(s3, LOW);
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);
  digitalWrite(s3, HIGH);
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW  : HIGH);
  delay(20);
  digitalWrite(s2, HIGH);
  Green = pulseIn(out,  digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);
}
