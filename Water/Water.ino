#include <Servo.h>

#include "Enes100.h"

void setup() {
    // Initialize Enes100 Library
    // Team Name, Mission Type, Marker ID, Room Number, Wifi Module TX Pin, Wifi Module RX Pin
    Enes100.begin("C'Ryan Me A River", WATER, 12, 1116, 3, 2);
    // At this point we know we are connected.
    Enes100.println("Connected...");
}

void loop() {
    float x, y, t; bool v; // Declare variables to hold the data
    //Enes100.getX will make sure you get the latest data available to you about your OTV's location.
    //The first time getX is called, X, Y, theta and visibility are queried and cached.
    //Subsequent calls return from the cache, so there is no performance gain to saving the function response to a variable.

    x = Enes100.getX();  // Your X coordinate! 0-4, in meters, -1 if no aruco is not visibility (but you should use Enes100.isVisible to check that instead)
    y = Enes100.getY();  // Your Y coordinate! 0-2, in meters, also -1 if your aruco is not visible.
    t = Enes100.getTheta();  //Your theta! -pi to +pi, in radians, -1 if your aruco is not visible.
    v = Enes100.isVisible(); // Is your aruco visible? True or False.

    if (v) // If the ArUco marker is visible
    {
        Enes100.print(x); // print out the location
        Enes100.print(",");
        Enes100.print(y);
        Enes100.print(",");
        Enes100.println(t);
    }
    else { // otherwise
        Enes100.println("Not visible"); // print not visible
    }

    // Transmit the state of the pool
    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
    // Transmit the depth of the pool in mm (20, 30, or 40)
    Enes100.mission(DEPTH, 30);
    delay(1000);
}

//Note: working code below

#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"



void setup() {
    Enes100.begin("C'ryan me a River", WATER, 12, 1116, 8, 9);
    Tank.begin();
}

int sign(float number) {
    if (number >= 0) {
        return 1;
    }
    else {
        return -1;
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

void loop() {
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
}

