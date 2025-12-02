/********************  INCLUDES  ********************/
#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include "Enes100.h"

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

#define WATER_DEPTHS_MM_0       20
#define WATER_DEPTHS_MM_1       30
#define WATER_DEPTHS_MM_2       40

#define POOL_STANDOFF_TARGET_M  0.15f
#define POOL_STANDOFF_TOL_M     0.03f

// --- Vision / motion ---
#define VISION_WAIT_MS          6000
#define VISION_DROP_GRACE_MS    800
#define BASE_PWM                95
#define K_TURN                  120.0f
#define K_STRAIGHT              90.0f
#define MAX_PWM                 180
#define METERS_PER_SEC          0.50f
#define TURN_TIME_90_MS         5750
#define TURN_PWM                120

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

// --- Random guess (last resort) ---
#define ALLOW_RANDOM_GUESS      0

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
#define US_TRIG   0
#define US_ECHO   1
#define US_MAX_CM 200
#define DEPTH_AIN A0

//S0 and S1 are unused in the pinout, they are connected to VCC

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
static float nA(float a){ while(a>PI) a-=2*PI; while(a<-PI)a+=2*PI; return a; }

static void mL(int p){
  int s = constrain(abs(p),0,255);
  analogWrite(ENA, s);
  if ((p>=0) ^ INVERT_LEFT)  { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); }
  else                       { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);}
}
static void mR(int p){
  int s = constrain(abs(p),0,255);
  analogWrite(ENB, s);
  if ((p>=0) ^ INVERT_RIGHT) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); }
  else                       { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);}
}
static void setM(int L,int R){ mL(L); mR(R); }
static void brake(){ analogWrite(ENA,0); analogWrite(ENB,0); }

static bool waitVis(uint32_t t=VISION_WAIT_MS){
  uint32_t s = millis();
  while (millis()-s < t){ if (Enes100.isVisible()) return true; delay(30); }
  return false;
}

/********************  VISION-ASSISTED MOTION  ********************/
static void turnTo(float tgt, uint32_t t=3500){
  uint32_t s = millis();
  while (millis()-s < t){
    float e = nA(tgt - Enes100.getTheta());
    if (fabs(e) < 0.03f) break;
    int pwm = (int)constrain(K_TURN * e, -MAX_PWM, MAX_PWM);
    setM(-pwm, pwm);
    delay(10);
  }
  brake(); delay(100);
}
static void turnBy(float d){ turnTo(nA(Enes100.getTheta()+d)); }

static void holdStraight_time(float sec, int base=BASE_PWM){
  float th0 = Enes100.getTheta();
  uint32_t end = millis() + (uint32_t)(sec*1000);
  while ((int32_t)(end - millis()) > 0){
    float e = nA(th0 - Enes100.getTheta());
    int c = (int)constrain(K_STRAIGHT * e, -90, 90);
    setM(base-c, base+c);
    delay(10);
  }
  brake();
}

static bool driveToward(float tx, float ty, float stopDistM){
  if (!waitVis()) return false;
  uint32_t t0 = millis();
  while (true){
    float x=Enes100.getX(), y=Enes100.getY(), th=Enes100.getTheta();
    float dx = tx-x, dy = ty-y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist <= stopDistM) { brake(); return true; }
    float th_des = atan2f(dy, dx);
    float e = nA(th_des - th);
    int steer = (int)constrain(K_STRAIGHT * e, -90, 90);
    setM(BASE_PWM - steer, BASE_PWM + steer);

    // Graceful loss handling
    if (!Enes100.isVisible()){
      uint32_t lost=millis();
      while(!Enes100.isVisible() && millis()-lost < VISION_DROP_GRACE_MS){
        setM(BASE_PWM, BASE_PWM); delay(8);
      }
      if (!Enes100.isVisible()){
        brake(); return false;
      }
    }
    delay(10);
    if (millis()-t0 > 30000) { brake(); return false; } // timeout safeguard
  }
}

/********************  COLOR & DEPTH  ********************/
static unsigned long readColorRaw(byte s2, byte s3){
  digitalWrite(TCS_S2, s2);
  digitalWrite(TCS_S3, s3);
  delayMicroseconds(100);
  return pulseIn(TCS_OUT, digitalRead(TCS_OUT)==HIGH? LOW:HIGH, 25000);
}
static void tcsBegin(){
  pinMode(TCS_S0,OUTPUT); pinMode(TCS_S1,OUTPUT);
  pinMode(TCS_S2,OUTPUT); pinMode(TCS_S3,OUTPUT); pinMode(TCS_OUT,INPUT);
  digitalWrite(TCS_S0,HIGH); digitalWrite(TCS_S1,HIGH);
}
static bool detectPollutants_5s(){
  int votes=0, samples=0;
  uint32_t end=millis()+SENSE_WINDOW_MS;
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
  return (samples>=TCS_MIN_SAMPLES) && (votes>=COLOR_CONFIRM_COUNT);
}
static int stableDepthMM_5s(){
  // collect samples then compute median and std-dev
  const int N = SENSE_WINDOW_MS / SENSE_INTERVAL_MS;
  int v[N], k=0;
  while (k<N){
    int raw = analogRead(DEPTH_AIN);
    int mm  = map(raw, 0, 150, 0, 40); // tune once if needed
    v[k++] = mm;
    delay(SENSE_INTERVAL_MS);
  }
  // compute mean and stddev
  float sum=0; for(int i=0;i<N;i++) sum+=v[i];
  float mean=sum/N;
  float var=0; for(int i=0;i<N;i++){ float d=v[i]-mean; var+=d*d; }
  float sd = sqrtf(var/N);
  if (sd > DEPTH_STABLE_STD_MM_MAX){
    return -1; // unstable
  }
  // round to nearest of {20,30,40}
  int mm = (int)roundf(mean);
  int best=20, bestd=abs(mm-20);
  int cand[3]={20,30,40};
  for (int i=0;i<3;i++){
    int d=abs(mm-cand[i]); if (d<bestd){best=cand[i]; bestd=d;}
  }
  return best;
}

/********************  ULTRASONIC HELPERS  ********************/
static float ultraM(){
  unsigned int uS = sonar.ping();   // microseconds
  unsigned int cm = sonar.convert_cm(uS);
  if (cm==0) return 5.0f;           // out of range → big number
  return cm / 100.0f;
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
  // Library enums vary by release; these are the usual names:
  Enes100.mission(WATER_TYPE, (polluted ? FRESH_POLLUTED : FRESH_UNPOLLUTED));
  if (depthMM>0) Enes100.mission(DEPTH, depthMM);
}

/********************  SETUP / LOOP  ********************/
void setup(){
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT); pinMode(ENB,OUTPUT);

  arm.attach(SERVO_PIN);
  arm.write(SERVO_STOW_DEG);

  tcsBegin();

  Enes100.begin(TEAM_NAME, MISSION, MARKER_ID, ROOM_NUMBER, WIFI_TX, WIFI_RX);
}

static void orientTo(float tx, float ty){
  if (waitVis()){
    float x=Enes100.getX(), y=Enes100.getY();
    float th_des = atan2f(ty-y, tx-x);
    turnTo(th_des);
  }
}

static void slideLane(float dir){ // dir: +1 right (toward y+), -1 left (toward y-)
  // Guard edges
  float y = Enes100.getY();
  if ((dir>0 && y > ARENA_Y-EDGE_GUARD_Y_M) || (dir<0 && y < EDGE_GUARD_Y_M)) return;
  // 90-deg turn, move LANE_SHIFT_M, turn back
  turnBy(dir>0? +PI/2 : -PI/2);
  holdStraight_time(LANE_SHIFT_M / METERS_PER_SEC, BASE_PWM);
  turnBy(dir>0? -PI/2 : +PI/2);
}

void loop(){
  if (ran) return;

  // Decide mission site (A or B). If you know it beforehand, set (mx,my) directly.
  float mx=A_X, my=A_Y;
  // Heuristic: if y is closer to B_Y at start, assume mission at A; else B. (Not critical; we visually drive to the pool region anyway.)
  if (waitVis()){
    float y0=Enes100.getY();
    if (fabsf(y0 - A_Y) < fabsf(y0 - B_Y)) { mx = B_X; my = B_Y; } else { mx = A_X; my = A_Y; }
  }

  // --- STATE_ORIENT_TO_MISSION ---
  orientTo(mx, my);

  // --- STATE_DRIVE_TO_MISSION ---
  bool ok = driveToward(mx, my, 0.30f);  // stop ~30 cm away
  if (!ok){ // fallback: timed straight
    holdStraight_time(0.8f, BASE_PWM);
  }

  // --- STATE_FINE_STANDOFF ---
  // approach slowly with ultrasonic to standoff 0.15 m
  while (true){
    float d = ultraM();
    if (d <= POOL_STANDOFF_TARGET_M + POOL_STANDOFF_TOL_M) break;
    setM(70,70);
    delay(30);
    if (d < ULTRA_OBST_STOP_M) break;
  }
  brake();

  // --- STATE_MEASURE_WATER ---
  arm.write(SERVO_MEASURE_DEG); delay(SERVO_MOVE_MS);
  bool polluted=false;
  int depthMM=-1;

  // color
  polluted = detectPollutants_5s();
  // depth
  depthMM  = stableDepthMM_5s();
  arm.write(SERVO_STOW_DEG); delay(SERVO_MOVE_MS);

  // last resort (off by default)
  if (ALLOW_RANDOM_GUESS){
    if (depthMM<0){ int choices[3]={20,30,40}; depthMM = choices[random(0,3)]; }
    // 50-50 on pollutants
    if (!polluted){ polluted = (random(0,2)==1); }
  }

  // send telemetry
  if (depthMM>0) sendTelemetry(polluted, depthMM);
  else           sendTelemetry(polluted, 30); // conservative default if unstable

  // --- STATE_NAV_OBSTACLES ---
  // Drive toward far side, sliding lanes if ultrasonic sees a block
  if (!waitVis()){
    // timed traverse if no vision at all
    holdStraight_time( (OBST_COL_X2 - (mx)) / METERS_PER_SEC, BASE_PWM);
  } else {
    uint32_t t0 = millis();
    while (Enes100.getX() < (OBST_COL_X2 + 0.30f) && millis()-t0 < 20000){
      float front = ultraM();
      if (front < ULTRA_OBST_STOP_M){
        // try to slide toward open lane: bias toward arena center
        float y=Enes100.getY();
        if (y < ARENA_Y/2.0f) slideLane(+1); else slideLane(-1);
      } else {
        // straight with heading hold
        float th0 = Enes100.getTheta();
        float e = nA(th0 - Enes100.getTheta());
        int c = (int)constrain(K_STRAIGHT * e, -90, 90);
        setM(BASE_PWM - c, BASE_PWM + c);
        delay(15);
      }
    }
    brake();
  }

  // --- STATE_APPROACH_LIMBO ---
  orientTo(LIMBO_X, LIMBO_Y);
  // approach and slow down near limbo
  if (!driveToward(LIMBO_X, LIMBO_Y, LIMBO_APPR_DIST_M)){
    holdStraight_time( 0.8f, BASE_PWM );
  }
  brake();

  // --- STATE_PASS_LIMBO ---
  // final alignment
  if (waitVis()){
    float x=Enes100.getX(), y=Enes100.getY();
    float th_des = atan2f(LIMBO_Y - y, LIMBO_X - x);
    turnTo(th_des);
  }
  // go under limbo slowly
  setM(LIMBO_SLOW_PWM, LIMBO_SLOW_PWM);
  delay(1800); // tune to cross under; or replace with vision distance increment
  brake();

  // --- STATE_FINISH ---
  ran = true;
}
