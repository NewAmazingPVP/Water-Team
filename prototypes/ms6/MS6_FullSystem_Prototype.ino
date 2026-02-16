#include <math.h>
#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include "Enes100.h"

const bool DEBUG = true;

void dbg(const char* s) {
  if (DEBUG) Enes100.print(s);
}
void dbgln(const char* s) {
  if (DEBUG) Enes100.println(s);
}
void dbgFloat(float v) {
  if (DEBUG) Enes100.print(v);
}

#define TEAM_NAME   "C'Ryan Me A River"
#define MISSION     WATER
#define MARKER_ID   123
#define ROOM_NUMBER 1120

#define WIFI_TX_PIN 8
#define WIFI_RX_PIN 9

#define ENA 10
#define IN1 2
#define IN2 7
#define ENB 11
#define IN3 12
#define IN4 13

#define SERVO_PIN 3

#define US_TRIG   0
#define US_ECHO   1
#define US_MAX_CM 200

#define DEPTH_AIN A0

#define TCS_S0  A1
#define TCS_S1  A2
#define TCS_S2  4
#define TCS_S3  5
#define TCS_OUT 6

const float ARENA_X  = 4.0f;
const float ARENA_Y  = 2.0f;

const float A_X = 0.33f;
const float A_Y = 1.50f;
const float B_X = 0.33f;
const float B_Y = 0.50f;

const float OBST_COL_X1 = 1.50f;
const float OBST_COL_X2 = 2.30f;

const int   LIMBO_SIDE_IS_TOP   = 1;
const float LIMBO_X             = 3.70f;
const float LIMBO_Y             = (LIMBO_SIDE_IS_TOP ? 1.70f : 0.50f);
const float LIMBO_APPR_DIST_M   = 0.40f;
const float LIMBO_PASS_DELTA_X  = 0.10f;

const int   BASE_PWM            = 95;
const float K_TURN              = 120.0f;
const float K_STRAIGHT          = 90.0f;
const int   MAX_PWM             = 180;
const unsigned long VISION_WAIT_MS = 6000;

const float ULTRA_OBST_STOP_M   = 0.28f;
const float LANE_SHIFT_M        = 0.50f;
const float EDGE_GUARD_Y_M      = 0.12f;

const float PI_F                = 3.14159265f;
const float HEADING_SPIN_THRESH = 0.6f;
const float DIST_SLOW_RADIUS    = 0.4f;

const unsigned long SENSE_WINDOW_MS         = 5000UL;
const unsigned long SENSE_INTERVAL_MS       = 40UL;
const float         DEPTH_STABLE_STD_MM_MAX = 2.0f;
const int           COLOR_CONFIRM_COUNT     = 5;

const int SERVO_STOW_DEG    = 20;
const int SERVO_MEASURE_DEG = 100;
const int SERVO_MOVE_MS     = 450;

const unsigned long TCS_RED_MAX     = 78;
const int           TCS_MIN_SAMPLES = 25;

bool  ran = false;

Servo  arm;
NewPing sonar(US_TRIG, US_ECHO, US_MAX_CM);

static float nA(float a){
  while (a >  PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
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

static void setM(int L, int R){
  if (L > 255)  L = 255;
  if (L < -255) L = -255;
  if (R > 255)  R = 255;
  if (R < -255) R = -255;
  mL(L);
  mR(R);
}

static void brake(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

static float ultraM(){
  unsigned int uS = sonar.ping();
  unsigned int cm = sonar.convert_cm(uS);
  if (cm == 0) {
    float farM = 5.0f;
    dbg("ultraM = ");
    dbgFloat(farM);
    dbgln("");
    return farM;
  }
  float m = cm / 100.0f;
  dbg("ultraM = ");
  dbgFloat(m);
  dbgln(" m");
  return m;
}

static bool waitVis(unsigned long t = VISION_WAIT_MS){
  dbg("waitVis: up to ms=");
  dbgFloat((float)t);
  dbgln("");
  unsigned long start = millis();
  while (millis() - start < t){
    if (Enes100.isVisible()){
      dbg("waitVis: visible after ms=");
      dbgFloat((float)(millis() - start));
      dbgln("");
      return true;
    }
    delay(30);
  }
  dbgln("waitVis: TIMEOUT no vision");
  return false;
}

static void turnTo(float tgt, unsigned long t = 3500){
  dbg("turnTo: tgt=");
  dbgFloat(tgt);
  dbgln("");
  unsigned long start = millis();
  while (millis() - start < t){
    float th = Enes100.getTheta();
    float e  = nA(tgt - th);

    dbg(" turnTo: th=");
    dbgFloat(th);
    dbg(" e=");
    dbgFloat(e);
    dbgln("");

    if (fabs(e) < 0.03f){
      dbgln(" turnTo: within tolerance");
      break;
    }

    float pwmF = K_TURN * e;
    int   pwm  = (int)pwmF;
    if (pwm >  MAX_PWM) pwm =  MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    setM(-pwm, pwm);
    delay(10);
  }
  brake();
  delay(100);
}

static void turnBy(float d){
  float tgt = nA(Enes100.getTheta() + d);
  dbg("turnBy: d=");
  dbgFloat(d);
  dbg(" tgt=");
  dbgFloat(tgt);
  dbgln("");
  turnTo(tgt);
}

static bool driveToward(float tx, float ty, float stopDistM){
  dbg("driveToward: target=(");
  dbgFloat(tx);
  dbg(" ");
  dbgFloat(ty);
  dbg(") stopDist=");
  dbgFloat(stopDistM);
  dbgln("");

  if (!waitVis()){
    dbgln("driveToward: FAIL (no initial vision)");
    return false;
  }

  unsigned long start   = millis();
  unsigned long lastLog = 0;

  while (true){
    if (!Enes100.isVisible()){
      brake();
      dbgln("driveToward: lost vision waiting...");
      if (!waitVis()){
        dbgln("driveToward: FAIL (vision never returned)");
        return false;
      }
    }

    float x  = Enes100.getX();
    float y  = Enes100.getY();
    float th = Enes100.getTheta();

    float dx   = tx - x;
    float dy   = ty - y;
    float dist = (float)sqrt(dx*dx + dy*dy);

    if (dist <= stopDistM){
      dbg("driveToward: reached dist=");
      dbgFloat(dist);
      dbgln("");
      brake();
      return true;
    }

    float th_des = (float)atan2(dy, dx);
    float e      = nA(th_des - th);
    float ae     = (float)fabs(e);

    if (ae > HEADING_SPIN_THRESH){
      float pwmF = K_TURN * e;
      int   pwm  = (int)pwmF;
      if (pwm >  MAX_PWM) pwm =  MAX_PWM;
      if (pwm < -MAX_PWM) pwm = -MAX_PWM;
      setM(-pwm, pwm);
    } else {

      float scale = 1.0f;
      if (dist < DIST_SLOW_RADIUS){
        scale = dist / DIST_SLOW_RADIUS;
        if (scale < 0.4f) scale = 0.4f;
      }
      int base = (int)(BASE_PWM * scale);

      float steerF = K_STRAIGHT * e;
      if (steerF >  60.0f) steerF =  60.0f;
      if (steerF < -60.0f) steerF = -60.0f;
      int steer = (int)steerF;

      setM(base - steer, base + steer);
    }

    unsigned long now = millis();
    if (now - lastLog > 200){
      lastLog = now;
      dbg("driveToward: x=");
      dbgFloat(x);
      dbg(" y=");
      dbgFloat(y);
      dbg(" th=");
      dbgFloat(th);
      dbg(" dist=");
      dbgFloat(dist);
      dbg(" e=");
      dbgFloat(e);
      dbgln("");
    }

    if (millis() - start > 30000){
      dbgln("driveToward: TIMEOUT");
      brake();
      return false;
    }

    delay(10);
  }
}

static void orientTo(float tx, float ty){
  if (!waitVis()){
    dbgln("orientTo: NO VISION");
    return;
  }
  float x  = Enes100.getX();
  float y  = Enes100.getY();
  float th_des = (float)atan2(ty - y, tx - x);

  dbg("orientTo: from (");
  dbgFloat(x);
  dbg(" ");
  dbgFloat(y);
  dbg(") to(");
  dbgFloat(tx);
  dbg(" ");
  dbgFloat(ty);
  dbg(") th_des=");
  dbgFloat(th_des);
  dbgln("");

  turnTo(th_des);
}

static void slideLane(float dir){
  dbg("slideLane: dir=");
  dbgFloat(dir);
  dbgln("");

  if (!waitVis()){
    dbgln("slideLane: NO VISION");
    return;
  }

  float y = Enes100.getY();
  if ((dir > 0.0f && y > ARENA_Y - EDGE_GUARD_Y_M) ||
      (dir < 0.0f && y < EDGE_GUARD_Y_M)){
    dbgln("slideLane: edge guard skipping slide");
    return;
  }

  float x = Enes100.getX();
  float targetX = x;
  float targetY = y + dir * LANE_SHIFT_M;

  dbg("slideLane: target=(");
  dbgFloat(targetX);
  dbg(" ");
  dbgFloat(targetY);
  dbgln(")");

  driveToward(targetX, targetY, 0.02f);
}

static unsigned long readColorRaw(byte s2, byte s3){
  digitalWrite(TCS_S2, s2);
  digitalWrite(TCS_S3, s3);
  delayMicroseconds(100);
  unsigned long val = pulseIn(TCS_OUT,
                              digitalRead(TCS_OUT) == HIGH ? LOW : HIGH,
                              25000);
  return val;
}

static void tcsBegin(){
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, HIGH);
  dbgln("TCS: initialized");
}

static bool detectPollutants_5s(){
  dbgln("Color: detectPollutants_5s START");
  int votes = 0;
  int samples = 0;
  unsigned long endT = millis() + SENSE_WINDOW_MS;
  while ((long)(endT - millis()) > 0){
    unsigned long R = readColorRaw(LOW,  LOW);
    unsigned long B = readColorRaw(LOW,  HIGH);
    unsigned long G = readColorRaw(HIGH, HIGH);
    if (R > 0 && B > 0 && G > 0){
      if (R < B && R <= G && R < TCS_RED_MAX) votes++;
      samples++;
    }
    delay(SENSE_INTERVAL_MS);
  }
  dbg("Color: samples=");
  dbgFloat((float)samples);
  dbg(" votes=");
  dbgFloat((float)votes);
  dbgln("");

  bool polluted = (samples >= TCS_MIN_SAMPLES) && (votes >= COLOR_CONFIRM_COUNT);
  dbg("Color: polluted=");
  dbgln(polluted ? "true" : "false");
  return polluted;
}

static int stableDepthMM_5s(){
  dbgln("Depth: stableDepthMM_5s START");
  const int N = (int)(SENSE_WINDOW_MS / SENSE_INTERVAL_MS);
  int v[N];
  int k = 0;

  while (k < N){
    int raw = analogRead(DEPTH_AIN);
    int mm  = map(raw, 0, 150, 0, 40);
    v[k++] = mm;
    delay(SENSE_INTERVAL_MS);
  }

  float sum = 0.0f;
  for (int i = 0; i < N; i++) sum += v[i];
  float mean = sum / N;

  float var = 0.0f;
  for (int i = 0; i < N; i++){
    float d = v[i] - mean;
    var += d * d;
  }
  float sd = sqrtf(var / N);

  dbg("Depth: mean=");
  dbgFloat(mean);
  dbg(" sd=");
  dbgFloat(sd);
  dbgln("");

  if (sd > DEPTH_STABLE_STD_MM_MAX){
    dbgln("Depth: UNSTABLE, returning -1");
    return -1;
  }

  int mm = (int)roundf(mean);
  int best = 20;
  int bestd = abs(mm - 20);
  int cand[3] = {20, 30, 40};
  for (int i = 0; i < 3; i++){
    int d = abs(mm - cand[i]);
    if (d < bestd){
      best  = cand[i];
      bestd = d;
    }
  }
  dbg("Depth: chosen=");
  dbgFloat((float)best);
  dbgln("");
  return best;
}

static void sendTelemetry(bool polluted, int depthMM){
  dbg("Telemetry: polluted=");
  dbgln(polluted ? "true" : "false");
  dbg(" depth=");
  dbgFloat((float)depthMM);
  dbgln("");

  Enes100.mission(WATER_TYPE, (polluted ? FRESH_POLLUTED : FRESH_UNPOLLUTED));
  if (depthMM > 0) Enes100.mission(DEPTH, depthMM);
}

void setup(){

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  arm.attach(SERVO_PIN);
  arm.write(SERVO_STOW_DEG);

  tcsBegin();

  Enes100.begin(TEAM_NAME, MISSION, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);

  dbgln("=== Real Robot Full Nav + Mission Test: START ===");
}

void loop(){
  if (ran) return;

  float mx = A_X;
  float my = A_Y;

  if (waitVis()){
    float y0 = Enes100.getY();
    dbg("Start Y=");
    dbgFloat(y0);
    dbgln("");

    if (fabs(y0 - A_Y) < fabs(y0 - B_Y)){
      mx = B_X; my = B_Y;
      dbgln("Heuristic: choosing mission site B");
    } else {
      mx = A_X; my = A_Y;
      dbgln("Heuristic: choosing mission site A");
    }
  } else {
    dbgln("No start vision: default mission A");
  }

  dbgln("STATE_ORIENT_TO_MISSION");
  orientTo(mx, my);

  dbgln("STATE_DRIVE_TO_MISSION");
  driveToward(mx, my, 0.12f);

  dbgln("STATE_MEASURE_WATER");
  arm.write(SERVO_MEASURE_DEG);
  delay(SERVO_MOVE_MS);

  bool polluted = false;
  int  depthMM  = -1;

  polluted = detectPollutants_5s();

  depthMM = stableDepthMM_5s();

  arm.write(SERVO_STOW_DEG);
  delay(SERVO_MOVE_MS);

  if (depthMM > 0) sendTelemetry(polluted, depthMM);
  else             sendTelemetry(polluted, 30);

  dbgln("STATE_NAV_OBSTACLES");
  if (waitVis()) {

    const float goalX       = OBST_COL_X2 + 0.30f;
    const float laneY       = LIMBO_Y;
    const float SWIPE_MIN_Y = 0.25f;
    const float SWIPE_MAX_Y = 1.75f;

    dbgln("Obstacles init align forward");
    turnTo(0.0f);

    unsigned long t0 = millis();
    while (Enes100.getX() < goalX && millis() - t0 < 25000UL) {

      if (!Enes100.isVisible()) {
        brake();
        dbgln("Obstacles lost vision wait");
        if (!waitVis()) {
          dbgln("Obstacles vision not recovered abort obstacle nav");
          break;
        }

        dbgln("Obstacles re align forward after vision return");
        turnTo(0.0f);
      }

      float front = ultraM();
      if (front < ULTRA_OBST_STOP_M) {

        dbgln("Obstacles front blocked sidestep");

        float y = Enes100.getY();

        float upY   = y + LANE_SHIFT_M;
        float downY = y - LANE_SHIFT_M;

        float dir;

        if (upY > SWIPE_MAX_Y && downY >= SWIPE_MIN_Y) {
          dir = -1.0f;
          dbgln("Sidestep choose DOWN due to upper limit");
        }

        else if (downY < SWIPE_MIN_Y && upY <= SWIPE_MAX_Y) {
          dir = +1.0f;
          dbgln("Sidestep choose UP due to lower limit");
        }

        else {

          dir = (y < laneY ? +1.0f : -1.0f);
          dbgln("Sidestep choose based on laneY");
        }

        float sideHeading = (dir > 0.0f) ? (PI_F * 0.5f) : -(PI_F * 0.5f);
        dbgln("Sidestep turn 90");
        turnTo(sideHeading);

        float yStart     = Enes100.getY();
        unsigned long ts = millis();
        dbgln("Sidestep move sideways");
        while (fabs(Enes100.getY() - yStart) < (LANE_SHIFT_M * 0.9f) &&
               millis() - ts < 4000UL &&
               Enes100.isVisible()) {
          setM(BASE_PWM, BASE_PWM);
          delay(20);
        }
        brake();

        dbgln("Sidestep re align forward");
        turnTo(0.0f);

        continue;
      }

      float x       = Enes100.getX();
      float remaining = goalX - x;
      if (remaining <= 0.05f) {
        dbgln("Obstacles reached clear region");
        break;
      }

      float th = Enes100.getTheta();
      float e  = nA(0.0f - th);

      float steerF = K_STRAIGHT * e;
      if (steerF >  60.0f) steerF =  60.0f;
      if (steerF < -60.0f) steerF = -60.0f;
      int steer = (int)steerF;

      setM(BASE_PWM - steer, BASE_PWM + steer);

      dbg("Obstacles step x=");
      dbgFloat(x);
      dbg(" rem=");
      dbgFloat(remaining);
      dbg(" e=");
      dbgFloat(e);
      dbgln("");

      delay(30);
    }

    brake();
  } else {
    dbgln("STATE_NAV_OBSTACLES skipped no vision");
  }

  dbgln("STATE_APPROACH_LIMBO");
  orientTo(LIMBO_X, LIMBO_Y);
  driveToward(LIMBO_X, LIMBO_Y, LIMBO_APPR_DIST_M);
  brake();

  dbgln("STATE_PASS_LIMBO: extra 2s timed push");
  unsigned long tPush = millis();
  while (millis() - tPush < 2000UL) {
    setM(BASE_PWM, BASE_PWM);
    delay(10);
  }
  brake();

  dbgln("STATE_FINISH");
  ran = true;
}
