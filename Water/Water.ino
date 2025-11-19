#include <Enes100.h>

#include <Servo.h>

#include <NewPing.h>

#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

//Servo:
Servo ArmServo;

int pos = 0; //will change based on what degree we want
int inital_pos = 0;

//Ultrasonic:
const int TrigPin = 0; //note: only need one of these, as it sends out signal from ultrasonic
const int EchoPin = 1;               // Arduino pin tied to echo pin on the ultrasonic sensor.
const int MAX_DISTANCE = 50; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define SONAR_NUM     1 // Number of sensors.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

NewPing sonar(TrigPin, EchoPin, MAX_DISTANCE); // NewPing setup of pins .

// color sensor:
//#define s0 4        //Module pins  wiring
//#define s1 5
#define s2 5
#define s3 6
#define out 7

bool Pollutants = false;

int  Red=0, Blue=0, Green=0;  //RGB values 

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
    // Initialize Enes100 Library
    // Team Name, Mission Type, Marker ID, Room Number, Wifi Module TX Pin, Wifi Module RX Pin
    Enes100.begin("CRyan Me A River", WATER, 12, 1116, 8, 9);
    // At this point we know we are connected.
    Enes100.println("Connected...");
    Tank.begin();

    //Servo:
    ArmServo.attach(9);

    //color sensor:
    pinMode(s0,OUTPUT);    //pin modes
    pinMode(s1,OUTPUT);
    pinMode(s2,OUTPUT);
    pinMode(s3,OUTPUT);
    pinMode(out,INPUT);

    Serial.begin(9600);   //intialize the serial monitor  baud rate
   
    digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels  means the output frequency scalling is at 100% (recommended)
    digitalWrite(s1,HIGH);  //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%
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




    //Ultrasonic:
    delay(50);       // Wait 50ms between pings ( 20 pings/sec). 29ms should be the shortest delay between pings.
    unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
    //only need if we have serial.begin
    //Serial.print("Ping: ");
    //Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    //Serial.println("cm");


    //color sensor:
      
    GetColors();                                     //Execute the GetColors function  to get the value of each RGB color
                                                    //Depending  of the RGB values given by the sensor we can define the color and displays it on  the monitor

    if (Red<Blue && Red<=Green && Red<78)  {    //if  Red value is the lowest one and smaller thant 23 it's likely Red
        Serial.println("Red");
    }

    else {
        Serial.println("Unknown");                  //if the color is not recognized, you can add as many as you want
    }
    if (Red<Blue && Red<=Green && Red<78){ //original: 23 ; 50 max? : 255   SET: 78
        Serial.println("Pollutants Detected");
    }
    delay(2000);                                   //2s delay you can modify if you  want

  //Note: for the values, man need to increace the detection values for things like red (ex: Red<23 up to Red<60) in order to have more range of detection (at costs of accuracy). 
  //Note: the higher the frequency --> the lower the data we get printed. This means that if we get low data, then the lowest color detecting data value menas that that ci=olor is likely the one in front of the color sensor. 

  //Servo:
   //if ([need a function when reach watertank]){
    for (pos = 0; pos = 180; pos += 1) { //position 0 is assumed to be straight up. pos 90 is 90 degrees (pi/2) and should be right above the tank (may need to change). move at a incrament of 1 degree
      ArmServo.write(pos);
      delay (15000);
      inital_pos = 0;
      ArmServo.write(inital_pos);
      delay (10000);
      //can adjust based on the water tank, and if collection of water is marked as complete. (happens after water depth and pollutant detection)
    }
  //}
}

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


    // Transmit the state of the pool
    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
    // Transmit the depth of the pool in mm (20, 30, or 40)
    Enes100.mission(DEPTH, 30);
    delay(1000);

    int Sensor_WaterLV=analogRead(A0); // Incoming analog signal read and appointed sensor

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

//Note: working code below






//int sign(float number) {
    //if (number >= 0) {
    //    return 1;
    //}
    //else {
    //    return -1;
    //}
//}

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

//color sensor how to get color:
void GetColors()  
{    
  digitalWrite(s2,  LOW);                                           //S2/S3 levels define which set  of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH  is for green 
  digitalWrite(s3, LOW);                                           
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);       //here we wait  until "out" go LOW, we start measuring the duration and stops when "out" is  HIGH again, if you have trouble with this expression check the bottom of the code
  delay(20);  
  digitalWrite(s3, HIGH);                                         //Here  we select the other color (set of photodiodes) and measure the other colors value  using the same techinque
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW  : HIGH);
  delay(20);  
  digitalWrite(s2, HIGH);  
  Green = pulseIn(out,  digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);  
}

//Note: the higher the frequency --> the lower the data we get printed


//void loop() {
    //float Yval = Enes100.getY();
    //if (Yval > 1) {
    //    directionCtrl(-(PI / 2));
    //}
    //else {
    //    directionCtrl(PI / 2);
    //}
    //Enes100.println("Setup Done!");
    //moveToX(1.0);
    //moveToY(1.0);
    //Enes100.println("Navigation Done!");
    //delay(10000);
//}
