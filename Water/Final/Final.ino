/********************  INCLUDES  ********************/
#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include "Enes100.h"

/********************  DEBUG TOGGLES  ********************/
#define DEBUG         0   // master toggle for all debug logs
#define DEBUG_SERIAL  0   // when DEBUG=1: 0 = only Enes100, 1 = Enes100 + Serial

#if DEBUG
  #if DEBUG_SERIAL
    #define DBG_BEGIN()        do { Serial.begin(9600); delay(200); } while(0)
    #define DBG_PRINT(x)       do { Serial.print(x); Enes100.print(x); } while(0)
    #define DBG_PRINTLN(x)     do { Serial.println(x); Enes100.println(x); } while(0)
  #else
    #define DBG_BEGIN()        do {} while(0)
    #define DBG_PRINT(x)       do { Enes100.print(x); } while(0)
    #define DBG_PRINTLN(x)     do { Enes100.println(x); } while(0)
  #endif
#else
  #define DBG_BEGIN()        do {} while(0)
  #define DBG_PRINT(x)       do {} while(0)
  #define DBG_PRINTLN(x)     do {} while(0)
#endif

/********************  CONSTANTS  (EDITABLE)  ********************/
// --- Frame / arena ---
#define ARENA_X                 4.0f
#define ARENA_Y                 2.0f
#define LINE_WIDTH_M            0.0285f
#define START_HEADING_RANDOM    1

// >>> Replace with your section’s exact A/B centers if posted <<<
#define A_X                     0.55f
#define A_Y                     1.50f
#define B_X                     0.55f
#define B_Y                     0.50f
#define MISSION_PLACE_TOL_M     0.05f

#define OBST_COL_X1             1.50f
#define OBST_COL_X2             2.30f
#define OBST_BLOCK_SIZE_M       0.127f

#define LIMBO_ROD_DIAM_M        0.025f
#define LIMBO_GROUND_CLEAR_M    0.184f
#define LIMBO_SIDE_IS_TOP       1
#define LIMBO_X                 3.70f
#define LIMBO_Y                 (LIMBO_SIDE_IS_TOP ? 1.50f : 0.50f)
#define LIMBO_APPR_DIST_M       0.40f
#define LIMBO_PASS_DELTA_X      0.50f   // how far past limbo center to drive

#define WATER_DEPTHS_MM_0       20
#define WATER_DEPTHS_MM_1       30
#define WATER_DEPTHS_MM_2       40

#define POOL_STANDOFF_TARGET_M  0.15f
#define POOL_STANDOFF_TOL_M     0.03f

// --- Vision / motion ---
#define VISION_WAIT_MS          6000
#define BASE_PWM                95
#define K_TURN                  120.0f
#define K_STRAIGHT              90.0f
#define MAX_PWM                 180

// --- Obstacles / lanes ---
#define ULTRA_OBST_WARN_M       0.40f
#define ULTRA_OBST_STOP_M       0.28f
#define LANE_SHIFT_M            0.35f
#define EDGE_GUARD_Y_M          0.12f

// --- Limbo ---
#define LIMBO_ALIGN_TOL_RAD     0.10f
#define LIMBO_SLOW_PWM          75

// --- Sensing windows ---
#define SENSE_WINDOW_MS         5000
#define SENSE_INTERVAL_MS       40
#define DEPTH_STABLE_STD_MM_MAX 2
#define COLOR_CONFIRM_COUNT     5

// --- Pins ---
#define WIFI_TX   8
#define WIFI_RX   9

#define ENA 10
#define IN1 2
#define IN2 7
#define ENB 11
#define IN3 12
#define IN4 13

#define SERVO_PIN 3
// WARNING: pins 0/1 are also Hardware Serial RX/TX on many Arduinos.
// If DEBUG_SERIAL=1 and Serial is used, this can conflict with ultrasonic wiring.
#define US_TRIG   0
#define US_ECHO   1
#define US_MAX_CM 200
#define DEPTH_AIN A0

// S0 and S1 are unused in the pinout, they are connected to VCC
#define TCS_S0  A1
#define TCS_S1  A2
#define TCS_S2  4
#define TCS_S3  5
#define TCS_OUT 6

#define SERVO_STOW_DEG          20
#define SERVO_MEASURE_DEG       100
#define SERVO_MOVE_MS           450

#define TCS_RED_MAX             78
#define TCS_MIN_SAMPLES         25

#define INVERT_LEFT             0
#define INVERT_RIGHT            0

// --- Team / mission ---
#define TEAM_NAME   "C'Ryan Me A River"
#define MISSION     WATER
#define MARKER_ID   123
#define ROOM_NUMBER 1116

/********************  GLOBALS  ********************/
Servo arm;
NewPing sonar(US_TRIG, US_ECHO, US_MAX_CM);
bool ran = false;   // single demonstration run

/********************  UTILS  ********************/
static float nA(float a){
  while(a > PI)   a -= 2*PI;
  while(a < -PI)  a += 2*PI;
  return a;
}

static void mL(int p){
  int s = constrain(abs(p),0,255);
  analogWrite(ENA, s);
  if ((p>=0) ^ INVERT_LEFT)  { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); }
  else                       { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); }
}

static void mR(int p){
  int s = constrain(abs(p),0,255);
  analogWrite(ENB, s);
  if ((p>=0) ^ INVERT_RIGHT) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); }
  else                       { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); }
}

static void setM(int L,int R){
  mL(L); mR(R);
}

static void brake(){
  analogWrite(ENA,0); analogWrite(ENB,0);
  DBG_PRINTLN("Motors: BRAKE");
}

static bool waitVis(uint32_t t=VISION_WAIT_MS){
  DBG_PRINT("waitVis: waiting up to ms=");
  DBG_PRINTLN(t);
  uint32_t s = millis();
  while (millis()-s < t){
    if (Enes100.isVisible()){
      DBG_PRINT("waitVis: visible after ms=");
      DBG_PRINTLN(millis()-s);
      return true;
    }
    delay(30);
  }
  DBG_PRINTLN("waitVis: TIMEOUT, not visible");
  return false;
}

/********************  VISION-ASSISTED MOTION  ********************/
static void turnTo(float tgt, uint32_t t=3500){
  DBG_PRINT("turnTo: tgt=");
  DBG_PRINTLN(tgt);
  uint32_t s = millis();
  while (millis()-s < t){
    float th = Enes100.getTheta();
    float e  = nA(tgt - th);
    DBG_PRINT(" turnTo loop: th=");
    DBG_PRINT(th);
    DBG_PRINT(" e=");
    DBG_PRINTLN(e);

    if (fabs(e) < 0.03f) {
      DBG_PRINTLN(" turnTo: within tolerance, stopping");
      break;
    }
    int pwm = (int)constrain(K_TURN * e, -MAX_PWM, MAX_PWM);
    DBG_PRINT(" turnTo: pwm=");
    DBG_PRINTLN(pwm);
    setM(-pwm, pwm);
    delay(10);
  }
  brake(); delay(100);
}

static void turnBy(float d){
  float tgt = nA(Enes100.getTheta()+d);
  DBG_PRINT("turnBy: delta=");
  DBG_PRINT(d);
  DBG_PRINT(" tgt=");
  DBG_PRINTLN(tgt);
  turnTo(tgt);
}

static bool driveToward(float tx, float ty, float stopDistM){
  DBG_PRINT("driveToward: target=(");
  DBG_PRINT(tx); DBG_PRINT(",");
  DBG_PRINT(ty); DBG_PRINT(") stopDist=");
  DBG_PRINTLN(stopDistM);

  // Purely vision-based: if we can't see, we just wait; no timed driving.
  if (!waitVis()) {
    DBG_PRINTLN("driveToward: FAIL, never saw marker");
    return false;
  }

  uint32_t t0 = millis();
  uint32_t lastLog = 0;

  while (true){
    if (!Enes100.isVisible()){
      brake();
      DBG_PRINTLN("driveToward: lost vision, braking and waiting");
      if (!waitVis()) {
        DBG_PRINTLN("driveToward: FAIL, vision lost and not recovered");
        return false; // stay stopped until vision returns, or give up
      }
    }

    float x  = Enes100.getX();
    float y  = Enes100.getY();
    float th = Enes100.getTheta();

    float dx   = tx - x;
    float dy   = ty - y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist <= stopDistM){
      DBG_PRINT("driveToward: reached target, dist=");
      DBG_PRINTLN(dist);
      brake();
      return true;
    }

    float th_des = atan2f(dy, dx);
    float e      = nA(th_des - th);
    int steer    = (int)constrain(K_STRAIGHT * e, -90, 90);
    setM(BASE_PWM - steer, BASE_PWM + steer);

    uint32_t now = millis();
    if (now - lastLog > 200){
      lastLog = now;
      DBG_PRINT("driveToward: x=");
      DBG_PRINT(x);
      DBG_PRINT(" y=");
      DBG_PRINT(y);
      DBG_PRINT(" th=");
      DBG_PRINT(th);
      DBG_PRINT(" dist=");
      DBG_PRINT(dist);
      DBG_PRINT(" e=");
      DBG_PRINT(e);
      DBG_PRINT(" steer=");
      DBG_PRINTLN(steer);
    }

    delay(10);
    if (millis() - t0 > 30000){ // timeout safeguard
      DBG_PRINTLN("driveToward: TIMEOUT");
      brake();
      return false;
    }
  }
}

/********************  COLOR & DEPTH  ********************/
static unsigned long readColorRaw(byte s2, byte s3){
  digitalWrite(TCS_S2, s2);
  digitalWrite(TCS_S3, s3);
  delayMicroseconds(100);
  unsigned long val = pulseIn(TCS_OUT, digitalRead(TCS_OUT)==HIGH ? LOW : HIGH, 25000);
  return val;
}

static void tcsBegin(){
  pinMode(TCS_S0,OUTPUT); pinMode(TCS_S1,OUTPUT);
  pinMode(TCS_S2,OUTPUT); pinMode(TCS_S3,OUTPUT);
  pinMode(TCS_OUT,INPUT);
  digitalWrite(TCS_S0,HIGH); digitalWrite(TCS_S1,HIGH); // 100% freq scaling
  DBG_PRINTLN("TCS: initialized");
}

static bool detectPollutants_5s(){
  DBG_PRINTLN("Color: detectPollutants_5s START");
  int votes=0, samples=0;
  uint32_t end = millis()+SENSE_WINDOW_MS;
  while ((int32_t)(end - millis()) > 0){
    unsigned long R = readColorRaw(LOW,LOW);
    unsigned long B = readColorRaw(LOW,HIGH);
    unsigned long G = readColorRaw(HIGH,HIGH);
    if (R>0 && B>0 && G>0){
      if (R < B && R <= G && R < TCS_RED_MAX) votes++;
      samples++;
    }
    delay(SENSE_INTERVAL_MS);
  }
  DBG_PRINT("Color: samples=");
  DBG_PRINT(samples);
  DBG_PRINT(" votes=");
  DBG_PRINTLN(votes);
  bool polluted = (samples>=TCS_MIN_SAMPLES) && (votes>=COLOR_CONFIRM_COUNT);
  DBG_PRINT("Color: polluted=");
  DBG_PRINTLN(polluted ? "true" : "false");
  return polluted;
}

static int stableDepthMM_5s(){
  DBG_PRINTLN("Depth: stableDepthMM_5s START");
  const int N = SENSE_WINDOW_MS / SENSE_INTERVAL_MS;
  int v[N], k=0;
  while (k<N){
    int raw = analogRead(DEPTH_AIN);
    int mm  = map(raw, 0, 150, 0, 40); // tune once if needed
    v[k++] = mm;
    delay(SENSE_INTERVAL_MS);
  }
  float sum=0; for(int i=0;i<N;i++) sum+=v[i];
  float mean=sum/N;
  float var=0; for(int i=0;i<N;i++){ float d=v[i]-mean; var+=d*d; }
  float sd = sqrtf(var/N);

  DBG_PRINT("Depth: mean=");
  DBG_PRINT(mean);
  DBG_PRINT(" sd=");
  DBG_PRINTLN(sd);

  if (sd > DEPTH_STABLE_STD_MM_MAX){
    DBG_PRINTLN("Depth: UNSTABLE, returning -1");
    return -1; // unstable
  }
  int mm = (int)roundf(mean);
  int best=20, bestd=abs(mm-20);
  int cand[3]={20,30,40};
  for (int i=0;i<3;i++){
    int d=abs(mm-cand[i]);
    if (d<bestd){best=cand[i]; bestd=d;}
  }
  DBG_PRINT("Depth: chosen=");
  DBG_PRINTLN(best);
  return best;
}

/********************  ULTRASONIC HELPERS  ********************/
static float ultraM(){
  unsigned int uS = sonar.ping();   // microseconds
  unsigned int cm = sonar.convert_cm(uS);
  if (cm==0) return 5.0f;           // out of range → big number
  float m = cm / 100.0f;
  DBG_PRINT("Ultrasonic: ");
  DBG_PRINT(m);
  DBG_PRINTLN(" m");
  return m;
}

/********************  HIGH-LEVEL STATES  ********************/
enum State {
  STATE_ORIENT_TO_MISSION=0,
  STATE_DRIVE_TO_MISSION,
  STATE_FINE_STANDOFF,
  STATE_MEASURE_WATER,
  STATE_NAV_OBSTACLES,
  STATE_APPROACH_LIMBO,
  STATE_PASS_LIMBO,
  STATE_FINISH
};

static void sendTelemetry(bool polluted, int depthMM){
  DBG_PRINT("Telemetry: polluted=");
  DBG_PRINT(polluted ? "true" : "false");
  DBG_PRINT(" depth=");
  DBG_PRINTLN(depthMM);

  Enes100.mission(WATER_TYPE, (polluted ? FRESH_POLLUTED : FRESH_UNPOLLUTED));
  if (depthMM>0) Enes100.mission(DEPTH, depthMM);
}

/********************  SETUP / LOOP  ********************/
void setup(){
  DBG_BEGIN();
  DBG_PRINTLN("SETUP: starting");

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT); pinMode(ENB,OUTPUT);

  arm.attach(SERVO_PIN);
  arm.write(SERVO_STOW_DEG);

  tcsBegin();

  Enes100.begin(TEAM_NAME, MISSION, MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
  DBG_PRINTLN("SETUP: Enes100 connected");
}

static void orientTo(float tx, float ty){
  DBG_PRINT("orientTo: target=(");
  DBG_PRINT(tx); DBG_PRINT(",");
  DBG_PRINT(ty); DBG_PRINTLN(")");
  if (!waitVis()) {
    DBG_PRINTLN("orientTo: NO VISION, abort");
    return;
  }
  float x = Enes100.getX();
  float y = Enes100.getY();
  float th_des = atan2f(ty-y, tx-x);
  DBG_PRINT("orientTo: current=(");
  DBG_PRINT(x); DBG_PRINT(",");
  DBG_PRINT(y); DBG_PRINT(") th_des=");
  DBG_PRINTLN(th_des);
  turnTo(th_des);
}

static void slideLane(float dir){ // dir: +1 toward y+ (top), -1 toward y- (bottom)
  DBG_PRINT("slideLane: dir=");
  DBG_PRINTLN(dir);
  if (!waitVis()) {
    DBG_PRINTLN("slideLane: NO VISION, abort");
    return;
  }
  float y = Enes100.getY();
  if ((dir>0 && y > ARENA_Y-EDGE_GUARD_Y_M) || (dir<0 && y < EDGE_GUARD_Y_M)) {
    DBG_PRINTLN("slideLane: edge guard, refusing to slide");
    return;
  }

  float x = Enes100.getX();
  float targetX = x;               // stay roughly same x
  float targetY = y + dir*LANE_SHIFT_M;

  DBG_PRINT("slideLane: target=(");
  DBG_PRINT(targetX); DBG_PRINT(",");
  DBG_PRINT(targetY); DBG_PRINTLN(")");
  driveToward(targetX, targetY, 0.02f);
}

void loop(){
  if (ran) return;

  DBG_PRINTLN("=== STATE: START ===");

  // Decide mission site (A or B). If you know it beforehand, set (mx,my) directly.
  float mx=A_X, my=A_Y;
  if (waitVis()){
    float y0=Enes100.getY();
    DBG_PRINT("Start Y=");
    DBG_PRINTLN(y0);
    if (fabsf(y0 - A_Y) < fabsf(y0 - B_Y)) {
      mx = B_X; my = B_Y;
      DBG_PRINTLN("Heuristic: choosing mission site B");
    } else {
      mx = A_X; my = A_Y;
      DBG_PRINTLN("Heuristic: choosing mission site A");
    }
  } else {
    DBG_PRINTLN("Start: no vision, defaulting mission site A");
  }

  // --- STATE_ORIENT_TO_MISSION ---
  DBG_PRINTLN("STATE_ORIENT_TO_MISSION");
  orientTo(mx, my);

  // --- STATE_DRIVE_TO_MISSION ---
  DBG_PRINTLN("STATE_DRIVE_TO_MISSION");
  driveToward(mx, my, 0.12f);  // purely vision-based approach (~12 cm from mission)

  // --- STATE_FINE_STANDOFF (OPTIONAL ULTRASONIC) ---
  /*
  DBG_PRINTLN("STATE_FINE_STANDOFF (ultrasonic)");
  while (true){
    float d = ultraM();
    if (d <= POOL_STANDOFF_TARGET_M + POOL_STANDOFF_TOL_M) break;
    setM(70,70);
    delay(30);
    if (d < ULTRA_OBST_STOP_M) break;
  }
  brake();
  */

  // --- STATE_MEASURE_WATER ---
  DBG_PRINTLN("STATE_MEASURE_WATER");
  arm.write(SERVO_MEASURE_DEG); delay(SERVO_MOVE_MS);
  bool polluted=false;
  int depthMM=-1;

  polluted = detectPollutants_5s();
  depthMM  = stableDepthMM_5s();
  arm.write(SERVO_STOW_DEG); delay(SERVO_MOVE_MS);

  // send telemetry (no random alternatives)
  if (depthMM>0) sendTelemetry(polluted, depthMM);
  else           sendTelemetry(polluted, 30); // conservative default if unstable

  // --- STATE_NAV_OBSTACLES ---
  DBG_PRINTLN("STATE_NAV_OBSTACLES");
  if (waitVis()){
    uint32_t t0 = millis();
    while (Enes100.getX() < (OBST_COL_X2 + 0.30f) && millis()-t0 < 20000){
      if (!Enes100.isVisible()){
        brake();
        DBG_PRINTLN("Obstacles: lost vision, braking and waiting");
        if (!waitVis()) {
          DBG_PRINTLN("Obstacles: vision not recovered, breaking");
          break;
        }
      }
      float front = ultraM();
      if (front < ULTRA_OBST_STOP_M){
        DBG_PRINTLN("Obstacles: front blocked, sliding lane");
        float y = Enes100.getY();
        if (y < ARENA_Y/2.0f) slideLane(+1); else slideLane(-1);
      } else {
        // hold current heading (simplified)
        int c = 0;
        setM(BASE_PWM - c, BASE_PWM + c);
        delay(15);
      }
    }
    brake();
  } else {
    DBG_PRINTLN("STATE_NAV_OBSTACLES: skipped, no vision");
  }

  // --- STATE_APPROACH_LIMBO ---
  DBG_PRINTLN("STATE_APPROACH_LIMBO");
  orientTo(LIMBO_X, LIMBO_Y);
  driveToward(LIMBO_X, LIMBO_Y, LIMBO_APPR_DIST_M);
  brake();

  // --- STATE_PASS_LIMBO ---
  DBG_PRINTLN("STATE_PASS_LIMBO");
  if (waitVis()){
    float x = Enes100.getX();
    float y = Enes100.getY();
    float th_des = atan2f(LIMBO_Y - y, LIMBO_X - x);
    DBG_PRINT("Limbo align: x=");
    DBG_PRINT(x);
    DBG_PRINT(" y=");
    DBG_PRINT(y);
    DBG_PRINT(" th_des=");
    DBG_PRINTLN(th_des);
    turnTo(th_des);
  } else {
    DBG_PRINTLN("Limbo: no vision for final align");
  }
  // Drive to a point just past the limbo, all via vision
  float targetX = LIMBO_X + LIMBO_PASS_DELTA_X;
  float targetY = LIMBO_Y;
  DBG_PRINTLN("STATE_PASS_LIMBO: driving past rod");
  driveToward(targetX, targetY, 0.10f);
  brake();

  // --- STATE_FINISH ---
  DBG_PRINTLN("STATE_FINISH");
  ran = true;
}
