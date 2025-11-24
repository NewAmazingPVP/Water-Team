#include <Arduino.h>
#include "Enes100.h"

#define TEAM_NAME   "C'Ryan Me A River"
#define MISSION     WATER
#define MARKER_ID   459
#define ROOM_NUMBER 1116

#define WIFI_TX 8
#define WIFI_RX 9

#define ENA 10
#define IN1 2
#define IN2 7
#define ENB 11
#define IN3 12
#define IN4 13

#define BASE_PWM        95
#define K_TURN          120.0f
#define K_STRAIGHT      90.0f

// #define DISTANCE_MODE_TIME
#define DISTANCE_MODE_VISION

#define TURN_MODE_VISION
// #define TURN_MODE_TIME

static const float METERS_PER_SEC = 0.5f;
static const uint16_t TURN_TIME_90_MS = 5750;
static const int TURN_PWM = 120;

static float nA(float a) {
  while (a > PI)a -= 2 * PI;
  while (a < -PI)a += 2 * PI;
  return a;
}

static void mL(int p) {
  int s = abs(p);
  analogWrite(ENA, s);
  if (p >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}
static void mR(int p) {
  int s = abs(p);
  analogWrite(ENB, s);
  if (p >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}
static void setM(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  mL(L);
  mR(R);
}
static void brake() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

static bool waitVis(uint32_t t = 6000) {
  uint32_t s = millis();
  while (millis() - s < t) {
    if (Enes100.isVisible())return true;
    delay(30);
  } return false;
}

static void turnTo(float tgt, uint32_t t = 3500) {
  uint32_t s = millis();
  while (millis() - s < t) {
    float e = nA(tgt - Enes100.getTheta());
    int pwm = (int)constrain(K_TURN * e, -180, 180);
    if (fabs(e) < 0.03f)break;
    setM(-pwm, pwm);
    delay(10);
  }
  brake(); delay(100);
}
static void turnBy(float d) {
  turnTo(nA(Enes100.getTheta() + d));
}

static void straightHold_heading(float sec, int base = BASE_PWM) {
  float th0 = Enes100.getTheta();
  uint32_t end = millis() + (uint32_t)(sec * 1000);
  while ((int32_t)(end - millis()) > 0) {
    float e = nA(th0 - Enes100.getTheta());
    int c = (int)constrain(K_STRAIGHT * e, -90, 90);
    setM(base - c, base + c);
    delay(10);
  }
  brake();
}

static void goForward4m_time() {
  float sec = 4.0f / METERS_PER_SEC;
  straightHold_heading(sec, BASE_PWM);
}

static void goForward4m_vision() {
  if (!waitVis()) {
    goForward4m_time();
    return;
  }
  float x0 = Enes100.getX(), y0 = Enes100.getY(), th0 = Enes100.getTheta();
  uint32_t t0 = millis(), timeout = 20000;
  while (millis() - t0 < timeout) {
    float x = Enes100.getX(), y = Enes100.getY(), th = Enes100.getTheta();
    float s = (x - x0) * cos(th0) + (y - y0) * sin(th0);
    if (s >= 4.0f) break;
    float e = nA(th0 - th);
    int c = (int)constrain(K_STRAIGHT * e, -90, 90);
    setM(BASE_PWM - c, BASE_PWM + c);
    delay(10);
    if (!Enes100.isVisible()) { //vision  loss tolerance
      uint32_t lost = millis();
      while (!Enes100.isVisible() && millis() - lost < 800) {
        setM(BASE_PWM, BASE_PWM);
        delay(8);
      }
      if (!Enes100.isVisible()) {
        brake();
        goForward4m_time();
        return;
      }
    }
  }
  brake();
}

static void turn90_vision() {
  turnBy(PI / 2.0f);
  brake(); delay(1000);
}
static void turn90_time() {
  setM(-TURN_PWM, TURN_PWM);
  delay(TURN_TIME_90_MS);
  brake(); delay(1000);
}

static bool ran = false;
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  Enes100.begin(TEAM_NAME, MISSION, MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
}
void loop() {
  if (ran) return;

  #ifdef DISTANCE_MODE_VISION
    goForward4m_vision();
  #else
    goForward4m_time();
  #endif
  
  for (int i = 0; i < 3; i++) {
    #ifdef TURN_MODE_VISION
        turn90_vision();
    #else
        turn90_time();
    #endif
  }

  ran = true;
}
