#include <math.h>      // must be before Enes100
#include <Arduino.h>
#include "Enes100.h"
#include <Servo.h>


/********************  SIMPLE DEBUG HELPERS  ********************/
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

/********************  TEAM / WIFI (SET THESE FOR YOUR ROBOT)  ********************/
#define TEAM_NAME   "Ryan Me A River"   // <-- change if needed
#define MISSION     WATER               // or DATA, etc.
#define MARKER_ID   123                   // <-- set to your real ArUco ID
#define ROOM_NUMBER 1116                // <-- your room number

#define WIFI_TX_PIN 8
#define WIFI_RX_PIN 9

/********************  MOTOR PINS (FROM WORKING REAL CODE)  ********************/
#define ENA 10
#define IN1 2
#define IN2 7
#define ENB 11
#define IN3 12
#define IN4 13

// --- Water measurement hardware ---
#define SERVO_PIN   3

#define DEPTH_AIN   A0   // analog depth sensor

// TCS34725-style color sensor pins
#define TCS_S0      A1
#define TCS_S1      A2
#define TCS_S2      4
#define TCS_S3      5
#define TCS_OUT     6

// Servo positions for arm
#define SERVO_STOW_DEG     20
#define SERVO_MEASURE_DEG  100
#define SERVO_MOVE_MS      450  // wait time after move

// Color sensing / thresholds
#define SENSE_WINDOW_MS         5000
#define SENSE_INTERVAL_MS       40
#define DEPTH_STABLE_STD_MM_MAX 2
#define COLOR_CONFIRM_COUNT     5
#define TCS_RED_MAX             78
#define TCS_MIN_SAMPLES         25


/********************  CONSTANTS (MATCH PHYSICAL NAV)  ********************/
// Frame / arena
const float ARENA_X  = 4.0;
const float ARENA_Y  = 2.0;

// Mission A/B
const float A_X = 0.55;
const float A_Y = 1.50;
const float B_X = 0.55;
const float B_Y = 0.50;

// Obstacles (column region)
const float OBST_COL_X1 = 1.50;
const float OBST_COL_X2 = 2.30;

// Limbo
const int   LIMBO_SIDE_IS_TOP   = 1;
const float LIMBO_X             = 3.70;
const float LIMBO_Y             = (LIMBO_SIDE_IS_TOP ? 1.50 : 0.50);
const float LIMBO_APPR_DIST_M   = 0.40;
const float LIMBO_PASS_DELTA_X  = 0.50;

// Vision / motion
const int   BASE_PWM            = 95;
const float K_TURN              = 120.0f;
const float K_STRAIGHT          = 90.0f;
const int   MAX_PWM             = 180;
const unsigned long VISION_WAIT_MS = 6000;

// Obstacle / lane shifting
const float ULTRA_OBST_STOP_M   = 0.28f;
const float LANE_SHIFT_M        = 0.35f;
const float EDGE_GUARD_Y_M      = 0.12f;

// Heading control thresholds
const float PI_F                = 3.14159265f;
const float HEADING_SPIN_THRESH = 0.6f;   // rad ~34 deg
const float DIST_SLOW_RADIUS    = 0.4f;   // start slowing inside 40 cm

/********************  GLOBALS  ********************/
bool ran = false;    // single-run in real robot too
Servo arm;

/********************  UTILS  ********************/
static float nA(float a){
  while (a >  PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

/********************  LOW-LEVEL MOTOR CONTROL (REAL ROBOT)  ********************/
// From your working code, just wrapped so the rest of the nav code stays identical.

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

// This replaces the old Tank.setLeftMotorPWM / setRightMotorPWM
static void setM(int L, int R){
  if (L > 255)  L = 255;
  if (L < -255) L = -255;
  if (R > 255)  R = 255;
  if (R < -255) R = -255;
  mL(L);
  mR(R);
}

// This replaces Tank.turnOffMotors()
static void brake(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/********************  COLOR & DEPTH HELPERS  ********************/

static unsigned long readColorRaw(uint8_t s2, uint8_t s3){
  digitalWrite(TCS_S2, s2);
  digitalWrite(TCS_S3, s3);
  delayMicroseconds(100);
  // Measure pulse width; invert edge based on current pin state
  unsigned long val = pulseIn(
      TCS_OUT,
      (digitalRead(TCS_OUT) == HIGH) ? LOW : HIGH,
      25000
  );
  return val;
}

static void tcsBegin(){
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  // 100% frequency scaling
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, HIGH);
}

static bool detectPollutants_5s(){
  int votes   = 0;
  int samples = 0;

  unsigned long end = millis() + SENSE_WINDOW_MS;
  while ((long)(end - millis()) > 0){
    unsigned long R = readColorRaw(LOW,  LOW);
    unsigned long B = readColorRaw(LOW,  HIGH);
    unsigned long G = readColorRaw(HIGH, HIGH);

    if (R > 0 && B > 0 && G > 0){
      // "Red-ish and dark" = polluted
      if (R < B && R <= G && R < TCS_RED_MAX){
        votes++;
      }
      samples++;
    }
    delay(SENSE_INTERVAL_MS);
  }

  bool polluted = (samples >= TCS_MIN_SAMPLES) &&
                  (votes   >= COLOR_CONFIRM_COUNT);

  dbgln("Color: detectPollutants_5s done");
  return polluted;
}

static int stableDepthMM_5s(){
  const int N = SENSE_WINDOW_MS / SENSE_INTERVAL_MS;
  int v[N];
  int k = 0;

  while (k < N){
    int raw = analogRead(DEPTH_AIN);
    // Rough mapping from ADC to mm depth; tune constants on real robot
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

  if (sd > DEPTH_STABLE_STD_MM_MAX){
    dbgln("Depth: unstable");
    return -1;   // too noisy / moving
  }

  // Snap mean to nearest of {20,30,40} mm
  int mm = (int)roundf(mean);
  int cand[3] = {20, 30, 40};
  int best    = cand[0];
  int bestd   = abs(mm - cand[0]);
  for (int i = 1; i < 3; i++){
    int d = abs(mm - cand[i]);
    if (d < bestd){
      best  = cand[i];
      bestd = d;
    }
  }
  return best;
}

/********************  TELEMETRY  ********************/

static void sendTelemetry(bool polluted, int depthMM){
  // WATER_TYPE / FRESH_POLLUTED / FRESH_UNPOLLUTED / DEPTH
  // come from the Enes100 library.
  Enes100.mission(WATER_TYPE,
                  polluted ? FRESH_POLLUTED : FRESH_UNPOLLUTED);
  if (depthMM > 0){
    Enes100.mission(DEPTH, depthMM);
  }
}


/********************  (OPTIONAL) ULTRASONIC STUB  ********************/
// In your sim code this used Tank.readDistanceSensor(1), and the only usage
// is inside the commented-out obstacle state. To keep the code compiling on
// real hardware, we just stub it to "very far".
//
// If you later wire a real ultrasonic, you can implement it here.
static float ultraM(){
  float d = 5.0f;   // 5m = "no obstacle"
  dbg("ultraM (stub) = ");
  dbgFloat(d);
  dbgln("");
  return d;
}

/********************  VISION HELPERS  ********************/
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

/********************  TURNING  ********************/
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

    if (fabs(e) < 0.03f){  // ~2 deg
      dbgln(" turnTo: within tolerance");
      break;
    }

    float pwmF = K_TURN * e;
    int   pwm  = (int)pwmF;
    if (pwm >  MAX_PWM) pwm =  MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    setM(-pwm, pwm);       // spin in place
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

/********************  DRIVE TOWARD POINT (VISION-BASED)  ********************/
// This is the “don’t orbit around target” version.
// NAV LOGIC IDENTICAL TO SIM VERSION.
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

    // --- Avoid circles: spin in place on large heading error ---
    if (ae > HEADING_SPIN_THRESH){
      float pwmF = K_TURN * e;
      int   pwm  = (int)pwmF;
      if (pwm >  MAX_PWM) pwm =  MAX_PWM;
      if (pwm < -MAX_PWM) pwm = -MAX_PWM;
      setM(-pwm, pwm);    // spin only
    } else {
      // Heading OK: drive forward with steering.
      float scale = 1.0f;
      if (dist < DIST_SLOW_RADIUS){
        scale = dist / DIST_SLOW_RADIUS;  // 0..1
        if (scale < 0.4f) scale = 0.4f;   // avoid stalling
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

/********************  ORIENT TO POINT  ********************/
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

/********************  LANE SLIDE (VISION + ULTRASONIC)  ********************/
static void slideLane(float dir){ // dir: +1 toward y+ (top), -1 toward y- (bottom)
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

/********************  SETUP / LOOP  ********************/
void setup(){
  // Motor pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Arm servo
  arm.attach(SERVO_PIN);
  arm.write(SERVO_STOW_DEG);   // start stowed

  // Color sensor init
  tcsBegin();

  // Enes100 / WiFi
  Enes100.begin(TEAM_NAME, WATER, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);

  dbgln("=== Real Robot Full Nav + Water Measure: START ===");
}


void loop(){
  if (ran) return;

  // Decide mission site A/B using same heuristic as your real code
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

  // --- STATE_ORIENT_TO_MISSION ---
  dbgln("STATE_ORIENT_TO_MISSION");
  orientTo(mx, my);

  // --- STATE_DRIVE_TO_MISSION (pure vision) ---
  dbgln("STATE_DRIVE_TO_MISSION");
  driveToward(mx, my, 0.12f);   // about 12 cm radius

  // --- STATE_MEASURE_WATER (SIM: SKIPPED) ---
  dbgln("STATE_MEASURE_WATER");
  arm.write(SERVO_MEASURE_DEG); delay(SERVO_MOVE_MS);
  bool polluted=false;
  int depthMM=-1;

  polluted = detectPollutants_5s();
  depthMM  = stableDepthMM_5s();
  arm.write(SERVO_STOW_DEG); delay(SERVO_MOVE_MS);

  // send telemetry (no random alternatives)
  if (depthMM>0) sendTelemetry(polluted, depthMM);
  else           sendTelemetry(polluted, 30); // conservative default if unstable


  // --- STATE_NAV_OBSTACLES --- (still commented, same as sim)
  /*
  dbgln("STATE_NAV_OBSTACLES");
  if (waitVis()){
    unsigned long t0 = millis();
    while (Enes100.getX() < (OBST_COL_X2 + 0.30f) && millis() - t0 < 20000){
      if (!Enes100.isVisible()){
        brake();
        dbgln("Obstacles: lost vision waiting...");
        if (!waitVis()){
          dbgln("Obstacles: vision not recovered breaking");
          break;
        }
      }
      float front = ultraM();
      if (front < ULTRA_OBST_STOP_M){
        dbgln("Obstacles: front blocked sliding lane");
        float y = Enes100.getY();
        if (y < ARENA_Y / 2.0f) slideLane(+1.0f);
        else                    slideLane(-1.0f);
      } else {
        // simple straight push in obstacle zone
        setM(BASE_PWM, BASE_PWM);
        delay(20);
      }
    }
    brake();
  } else {
    dbgln("STATE_NAV_OBSTACLES: skipped (no vision)");
  }
  */

  // --- STATE_APPROACH_LIMBO ---
  dbgln("STATE_APPROACH_LIMBO");
  orientTo(LIMBO_X, LIMBO_Y);
  driveToward(LIMBO_X, LIMBO_Y, LIMBO_APPR_DIST_M);
  brake();

  // --- STATE_PASS_LIMBO ---
  dbgln("STATE_PASS_LIMBO");
  if (waitVis()){
    float x  = Enes100.getX();
    float y  = Enes100.getY();
    float th_des = (float)atan2(LIMBO_Y - y, LIMBO_X - x);
    dbg("Limbo align: x=");
    dbgFloat(x);
    dbg(" y=");
    dbgFloat(y);
    dbg(" th_des=");
    dbgFloat(th_des);
    dbgln("");
    turnTo(th_des);
  } else {
    dbgln("Limbo: no vision for final align");
  }

  float targetX = LIMBO_X + LIMBO_PASS_DELTA_X;
  float targetY = LIMBO_Y;
  dbgln("STATE_PASS_LIMBO: drive past limbo");
  driveToward(targetX, targetY, 0.10f);
  brake();

  dbgln("STATE_FINISH");
  ran = true;
}
