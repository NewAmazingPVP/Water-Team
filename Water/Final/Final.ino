#include <math.h>      // must be before Enes100
#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include "Enes100.h"

/********************  TEAM / WIFI (SET THESE FOR YOUR ROBOT)  ********************/
#define TEAM_NAME   "C'Ryan Me A River"
#define MISSION     WATER
#define MARKER_ID   123
#define ROOM_NUMBER 1120

#define WIFI_TX_PIN 8
#define WIFI_RX_PIN 5   // was 9; move WiFi module TX wire from D9 -> D5

/********************  MOTOR PINS (FROM WORKING REAL CODE)  ********************/
#define ENA 6           // was 10; move motor driver ENA wire from D10 -> D6 (PWM)
#define IN1 2
#define IN2 7
#define ENB 11
#define IN3 12
#define IN4 13

/********************  SENSOR / ACTUATOR PINS  ********************/
#define SERVO_PIN 3

// Ultrasonic (NewPing)
#define US_TRIG   A1    // was D0; move ultrasonic TRIG wire from D0 -> A5
#define US_ECHO   A2    // was D1; move ultrasonic ECHO wire from D1 -> A6
#define US_MAX_CM 200

// Depth sensor (analog)
#define DEPTH_AIN A0

// TCS3200 color sensor
//#define TCS_S0  A1
//#define TCS_S1  A2
#define TCS_S2  4
#define TCS_S3  A3      // was D5; move TCS S3 wire from D5 -> A3
#define TCS_OUT A4      // was D6; move TCS OUT wire from D6 -> A4

#define PUMP_PIN       A5     // uses A5 as digital output (D19). Not PWM.
#define PUMP_ON_MS     2000   // how long to pump after measuring (tweak)

/********************  CONSTANTS (MATCH PHYSICAL NAV)  ********************/
// Frame / arena
const float ARENA_X  = 4.0f;
const float ARENA_Y  = 2.0f;

// Mission A/B
const float A_X = 0.33f;
const float A_Y = 1.50f;
const float B_X = 0.33f;
const float B_Y = 0.50f;

// Obstacles (column region)
const float OBST_COL_X1 = 1.50f;
const float OBST_COL_X2 = 2.30f;

// Limbo
const int   LIMBO_SIDE_IS_TOP   = 1;
const float LIMBO_X             = 3.70f;
const float LIMBO_Y             = (LIMBO_SIDE_IS_TOP ? 1.70f : 0.50f);
const float LIMBO_APPR_DIST_M   = 0.40f;
const float LIMBO_PASS_DELTA_X  = 0.10f;

// Vision / motion
const int   BASE_PWM            = 95;
const float K_TURN              = 120.0f;
const float K_STRAIGHT          = 90.0f;
const int   MAX_PWM             = 180;
const unsigned long VISION_WAIT_MS = 6000;

// Obstacle / lane shifting
const float ULTRA_OBST_STOP_M   = 0.28f;
const float LANE_SHIFT_M        = 0.50f;
const float EDGE_GUARD_Y_M      = 0.12f;

// Heading control thresholds
const float PI_F                = 3.14159265f;
const float HEADING_SPIN_THRESH = 0.6f;   // rad ~34 deg
const float DIST_SLOW_RADIUS    = 0.05f;   // start slowing inside 40 cm
const int   DEPTH_OFFSET_MM  = -10; 

// Sensing windows for water mission
const unsigned long SENSE_WINDOW_MS         = 5000UL; // 5 seconds
const unsigned long SENSE_INTERVAL_MS       = 40UL;
const float         DEPTH_STABLE_STD_MM_MAX = 2.0f;
const int           COLOR_CONFIRM_COUNT     = 5;

// Servo positions
const int SERVO_STOW_DEG    = 110; //initial angle
const int SERVO_MEASURE_DEG = 40;
const int SERVO_MOVE_MS     = 450;

// Color thresholds
const unsigned long TCS_RED_MAX     = 78;
const int           TCS_MIN_SAMPLES = 25;

// put with your other constants
//const float OBST_EXIT_X = 2.85f;  // just into the Open Zone (past 2.80 m)
// Edge-follow backup routing
const float WALL_CLEAR_Y     = 0.10f;        // 10 cm from wall
const float START_CURVE_X    = 1.10f;        // where we meet the edge smoothly
const float OBST_EXIT_X      = 2.85f;        // past obstacle field (see above)
const float BUMP_OUT_X       = 0.15f;        // initial right bump-out from mission site
const float LIMBO_BUFFER_X   = (LIMBO_X - 0.20f);  // stage before final limbo approach

const float LIMBO_PRE_X = (LIMBO_X - 0.25f);


/********************  GLOBALS  ********************/
bool  ran = false;    // single-run

Servo  arm;
NewPing sonar(US_TRIG, US_ECHO, US_MAX_CM);

/********************  UTILS  ********************/
static float nA(float a){
  while (a >  PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

/********************  LOW-LEVEL MOTOR CONTROL (REAL ROBOT)  ********************/
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

/********************  ULTRASONIC (REAL, USED BY NAV)  ********************/
static float ultraM(){


  //delay(50); //uncomment this if you want to guarantee delay

  unsigned int uS = sonar.ping();           // microseconds
  unsigned int cm = sonar.convert_cm(uS);
  if (cm == 0) {
    float farM = 5.0f;
    return farM;                            // out of range → big number
  }
  float m = cm / 100.0f;
  return m;
}

/********************  VISION HELPERS  ********************/
static bool waitVis(unsigned long t = VISION_WAIT_MS){
  unsigned long start = millis();
  while (millis() - start < t){
    if (Enes100.isVisible()){
      return true;
    }
    delay(30);
  }
  return false;
}

/********************  TURNING  (NAV CODE: UNCHANGED LOGIC) ********************/
static void turnTo(float tgt, unsigned long t = 3500){
  unsigned long start = millis();
  while (millis() - start < t){
    float th = Enes100.getTheta();
    float e  = nA(tgt - th);

    if (fabs(e) < 0.05f){  // TODO: TUNE THIS
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
  turnTo(tgt);
}

static void backupEdgeBypass() {
  if (!waitVis()) return;

  float x0 = Enes100.getX();
  float y0 = Enes100.getY();

  // pick nearest wall lane
  const float yEdgeTop    = ARENA_Y - WALL_CLEAR_Y;   // ~1.90
  const float yEdgeBottom = WALL_CLEAR_Y;             // ~0.10
  float yEdge = (y0 < ARENA_Y * 0.5f) ? yEdgeBottom : yEdgeTop;

  unsigned long tPush = millis();
  while (millis() - tPush < 1500UL) {
    setM(-BASE_PWM, -BASE_PWM);
    delay(10);
  }
  brake();

  // 0) small bump-out in +X so we don’t scrape at start
  driveToward(x0 + BUMP_OUT_X, y0, 0.05f);

  // 1) smooth curve to chosen edge at a forward X
  driveToward(START_CURVE_X, yEdge, 0.05f);

  // 2) run along that edge until we’re past the obstacle field
  driveToward(OBST_EXIT_X, yEdge, 0.05f);

  // 3) ALWAYS align Y to limbo center in the open zone
  driveToward(OBST_EXIT_X, LIMBO_Y, 0.06f);      // lateral slide to centerline

  // 4) stage just before the rod, still on limbo centerline
  driveToward(LIMBO_PRE_X, LIMBO_Y, 0.08f);

  // 5) face straight +X and go straight through
  turnTo(0.0f);
  driveToward(LIMBO_X + LIMBO_PASS_DELTA_X, LIMBO_Y, 0.10f);
}


/********************  DRIVE TOWARD POINT (VISION-BASED)  ********************/
// NAV LOGIC IDENTICAL TO YOUR WORKING VERSION
static bool driveToward(float tx, float ty, float stopDistM){
  if (!waitVis()){
    return false;
  }

  unsigned long start = millis();

  while (true){
    if (!Enes100.isVisible()){
      brake();
      if (!waitVis()){
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
      brake();
      return true;
    }

    float th_des = (float)atan2(dy, dx);
    float e      = nA(th_des - th);
    float ae     = (float)fabs(e);

    // --- Avoid circles: spin in place on large heading error ---
    if (ae > HEADING_SPIN_THRESH && dist > 0.60f){
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
        if (scale < 0.6f) scale = 0.6f;   // avoid stalling
      }
      int base = (int)(BASE_PWM * scale);
      if (base < 80) base = 80; //TODO: Test this
      
      float steerF = K_STRAIGHT * e;
      if (steerF >  60.0f) steerF =  60.0f;
      if (steerF < -60.0f) steerF = -60.0f;
      int steer = (int)steerF;

      setM(base - steer, base + steer);
    }

    if (millis() - start > 30000){
      brake();
      return false;
    }

    delay(10);
  }
}

/********************  ORIENT TO POINT  (NAV CODE) ********************/
static void orientTo(float tx, float ty){
  if (!waitVis()){
    return;
  }
  float x  = Enes100.getX();
  float y  = Enes100.getY();
  float th_des = (float)atan2(ty - y, tx - x);
  turnTo(th_des);
}

/********************  LANE SLIDE (VISION + ULTRASONIC)  ********************/
static void slideLane(float dir){ // dir: +1 toward y+ (top), -1 toward y- (bottom)
  if (!waitVis()){
    return;
  }

  float y = Enes100.getY();
  if ((dir > 0.0f && y > ARENA_Y - EDGE_GUARD_Y_M) ||
      (dir < 0.0f && y < EDGE_GUARD_Y_M)){
    return;
  }

  float x = Enes100.getX();
  float targetX = x;
  float targetY = y + dir * LANE_SHIFT_M;

  driveToward(targetX, targetY, 0.02f);
}

/********************  COLOR & DEPTH HELPERS (MISSION)  ********************/
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
  //pinMode(TCS_S0, OUTPUT);
  //pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  //digitalWrite(TCS_S0, HIGH);
  //digitalWrite(TCS_S1, HIGH); // 100% freq scaling
}

static bool detectPollutants_5s(){
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

  bool polluted = (samples >= TCS_MIN_SAMPLES) && (votes >= COLOR_CONFIRM_COUNT);
  return polluted;
}

static int stableDepthMM_5s(){
  const int N = (int)(SENSE_WINDOW_MS / SENSE_INTERVAL_MS);
  int v[N];
  int k = 0;

  while (k < N){
    int raw = analogRead(DEPTH_AIN);
    int mm  = map(raw, 0, 150, 0, 40); // tune once if needed
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
    return -1; // unstable
  }

  int mm = (int)roundf(mean) + DEPTH_OFFSET_MM;
  if (mm < 0) mm = 0;
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
  return best;
}

static inline void pumpInit() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW); // off
}

static inline void pumpOn()  { digitalWrite(PUMP_PIN, HIGH); }
static inline void pumpOff() { digitalWrite(PUMP_PIN, LOW);  }

// Blocks for ms (simple and reliable)
static void runPumpMs(unsigned long ms){
  pumpOn();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { delay(5); }
  pumpOff();
}

/********************  TELEMETRY  ********************/
static void sendTelemetry(bool polluted, int depthMM){
  Enes100.mission(WATER_TYPE, (polluted ? FRESH_POLLUTED : FRESH_UNPOLLUTED));
  if (depthMM > 0) Enes100.mission(DEPTH, depthMM);
  runPumpMs(PUMP_ON_MS);
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

  // Servo
  arm.attach(SERVO_PIN);
  arm.write(SERVO_STOW_DEG);

  // Color sensor
  tcsBegin();

  pumpInit();

  // Enes100 (vision / wifi)
  Enes100.begin(TEAM_NAME, MISSION, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);
}

void loop(){
  //runPumpMs(10000);
  
  if (ran) return;

  // Decide mission site A/B using same heuristic as your real code
  float mx = A_X;
  float my = A_Y;

  if (waitVis()){
    float y0 = Enes100.getY();

    if (fabs(y0 - A_Y) < fabs(y0 - B_Y)){
      mx = B_X; my = B_Y;
    } else {
      mx = A_X; my = A_Y;
    }
  } else {
    // No start vision: default mission A
  }

  // --- STATE_ORIENT_TO_MISSION ---
  orientTo(mx, my);

  // --- STATE_DRIVE_TO_MISSION (pure vision) ---
  driveToward(mx, my, 0.12f);   // about 12 cm radius

  // --- STATE_MEASURE_WATER ---
  arm.write(SERVO_MEASURE_DEG);
  delay(SERVO_MOVE_MS);

  bool polluted = false;
  int  depthMM  = -1;

  // 5s color measurement
  polluted = detectPollutants_5s();

  // 5s depth measurement
  depthMM = stableDepthMM_5s();

  arm.write(SERVO_STOW_DEG);
  delay(SERVO_MOVE_MS);

  // send telemetry (conservative 30mm if unstable)
  if (depthMM > 0) sendTelemetry(polluted, depthMM);
  else             sendTelemetry(polluted, 30);

  // // --- STATE_NAV_OBSTACLES ---
  // if (waitVis()) {
  //   // Target is a point just past the obstacle column, in the limbo lane
  //   const float goalX       = OBST_COL_X2 + 0.30f;
  //   const float laneY       = LIMBO_Y;
  //   const float SWIPE_MIN_Y = 0.25f;   // hard lower bound
  //   const float SWIPE_MAX_Y = 1.75f;   // hard upper bound

  //   // Face roughly toward that goal first (heading ~0 rad, +X)
  //   turnTo(0.0f);

  //   unsigned long t0 = millis();
  //   while (Enes100.getX() < goalX && millis() - t0 < 25000UL) {

  //     // If vision drops, stop and wait for it to return
  //     if (!Enes100.isVisible()) {
  //       brake();
  //       if (!waitVis()) {
  //         break;
  //       }
  //       // Re-align once we get vision back
  //       turnTo(0.0f);
  //     }

  //     // Check ultrasonic straight ahead
  //     float front = ultraM();
  //     if (front < ULTRA_OBST_STOP_M) {
  //       // ---- SIDESTEP SEQUENCE WITH Y-LIMITS ----

  //       float y = Enes100.getY();

  //       // Proposed step positions
  //       float upY   = y + LANE_SHIFT_M;   // toward top (dir = +1)
  //       float downY = y - LANE_SHIFT_M;   // toward bottom (dir = -1)

  //       float dir;

  //       // 1) If stepping up would exceed max Y, force step down
  //       if (upY > SWIPE_MAX_Y && downY >= SWIPE_MIN_Y) {
  //         dir = -1.0f;
  //       }
  //       // 2) If stepping down would go below min Y, force step up
  //       else if (downY < SWIPE_MIN_Y && upY <= SWIPE_MAX_Y) {
  //         dir = +1.0f;
  //       }
  //       // 3) If both directions are legal, use lane-based choice
  //       else {
  //         // If below limbo lane, move up; if above, move down
  //         dir = (y < laneY ? +1.0f : -1.0f);
  //       }

  //       // 1) Turn 90° sideways (left or right in world Y)
  //       float sideHeading = (dir > 0.0f) ? (PI_F * 0.5f) : -(PI_F * 0.5f);
  //       turnTo(sideHeading);

  //       // 2) Drive sideways until we've moved ~LANE_SHIFT_M in Y
  //       float yStart     = Enes100.getY();
  //       unsigned long ts = millis();
  //       while (fabs(Enes100.getY() - yStart) < (LANE_SHIFT_M * 0.9f) &&
  //              millis() - ts < 4000UL &&
  //              Enes100.isVisible()) {
  //         setM(BASE_PWM, BASE_PWM);
  //         delay(20);
  //       }
  //       brake();

  //       // 3) Re-orient toward heading 0 (straight +X)
  //       turnTo(0.0f);

  //       // Back to top of loop and re-check ultrasonic / progress
  //       continue;
  //     }

  //     // ---- CLEAR AHEAD: MOVE FORWARD WITH HEADING ~0 RAD ----
  //     float x         = Enes100.getX();
  //     float remaining = goalX - x;
  //     if (remaining <= 0.05f) {
  //       break;
  //     }

  //     float th = Enes100.getTheta();
  //     float e  = nA(0.0f - th);      // error from perfect +X heading

  //     float steerF = K_STRAIGHT * e;
  //     if (steerF >  60.0f) steerF =  60.0f;
  //     if (steerF < -60.0f) steerF = -60.0f;
  //     int steer = (int)steerF;

  //     setM(BASE_PWM - steer, BASE_PWM + steer);

  //     delay(30); //TODO: ULTRASONIC DELAY
  //   }

  //   brake();
  // } else {
  //   // STATE_NAV_OBSTACLES skipped no vision
  // }



/*********** STATE_NAV_OBSTACLES with BACKUP ***********/
//dbgln("STATE_NAV_OBSTACLES");
if (waitVis()) {
  const float goalX       = OBST_COL_X2 + 0.30f;
  const float laneY       = LIMBO_Y;
  const float SWIPE_MIN_Y = 0.25f;
  const float SWIPE_MAX_Y = 1.75f;

  // Quick self-test: if US returns "very far" repeatedly, assume dead.
  int usDead = 0;
  for (int i = 0; i < 6; i++) { if (ultraM() >= 4.5f) usDead++; delay(50); }
  if (true) { //usDead >= 5
  //  dbgln("Ultrasonic appears offline -> BACKUP EDGE BYPASS");
    backupEdgeBypass();
  } else {
    //dbgln("Obstacles init align forward");
    turnTo(0.0f);

    unsigned long t0 = millis();
    int stale = 0;       // consecutive "far" readings (sensor failing)
    bool fallBack = false;

    while (Enes100.getX() < goalX && millis() - t0 < 25000UL) {
      if (!Enes100.isVisible()) {
        brake();
        if (!waitVis()) { break; }
        turnTo(0.0f);
      }

      float front = ultraM();
      if (true) {               // looks like sensor not reading
        if (++stale > 10) { fallBack = true; break; }
      } else stale = 0;

      if (front < ULTRA_OBST_STOP_M) {
        // ---- sidestep with y-limits ----
        float y = Enes100.getY();
        float upY   = y + LANE_SHIFT_M;
        float downY = y - LANE_SHIFT_M;
        float dir;

        if (upY > SWIPE_MAX_Y && downY >= SWIPE_MIN_Y)      dir = -1.0f;
        else if (downY < SWIPE_MIN_Y && upY <= SWIPE_MAX_Y) dir = +1.0f;
        else                                                dir = (y < laneY ? +1.0f : -1.0f);

        float sideHeading = (dir > 0.0f) ? (PI_F * 0.5f) : -(PI_F * 0.5f);
        turnTo(sideHeading);

        float yStart = Enes100.getY();
        unsigned long ts = millis();
        while (fabs(Enes100.getY() - yStart) < (LANE_SHIFT_M * 0.9f) &&
               millis() - ts < 4000UL &&
               Enes100.isVisible()) {
          setM(BASE_PWM, BASE_PWM);
          delay(20);
        }
        brake();
        turnTo(0.0f);
        continue;
      }

      // forward with heading 0
      float x = Enes100.getX();
      float remaining = goalX - x;
      if (remaining <= 0.05f) break;

      float th = Enes100.getTheta();
      float e  = nA(0.0f - th);
      float steerF = K_STRAIGHT * e;
      if (steerF >  60.0f) steerF =  60.0f;
      if (steerF < -60.0f) steerF = -60.0f;
      int steer = (int)steerF;
      setM(BASE_PWM - steer, BASE_PWM + steer);
      delay(30);
    }
    brake();
    if (fallBack) {
   //   dbgln("Switching to BACKUP EDGE BYPASS mid-run");
      backupEdgeBypass();
    }
  }
} else {
 // dbgln("STATE_NAV_OBSTACLES skipped no vision");
}
/*********** end state ***********/




  // --- STATE_APPROACH_LIMBO ---
  orientTo(LIMBO_X, LIMBO_Y);
  driveToward(LIMBO_X, LIMBO_Y, LIMBO_APPR_DIST_M);
  brake();

  // // --- STATE_PASS_LIMBO ---
  // if (waitVis()){
  //   float x  = Enes100.getX();
  //   float y  = Enes100.getY();
  //   float th_des = (float)atan2(LIMBO_Y - y, LIMBO_X - x);
  //   turnTo(th_des);
  // } else {
  //   // no vision for final align
  // }
  //
  // float targetX = LIMBO_X + LIMBO_PASS_DELTA_X;
  // float targetY = LIMBO_Y;
  // driveToward(targetX, targetY, 0.10f);
  // brake();

  unsigned long tPush = millis();
  while (millis() - tPush < 2000UL) {
    setM(BASE_PWM, BASE_PWM);
    delay(10);
  }
  brake();

  ran = true;
}
