#include <Arduino.h>
#include "Enes100.h"

#define TEAM_NAME "C'Ryan Me A River"
#define MISSION_TYPE WATER
#define MARKER_ID 12
#define ROOM_NUMBER 1116
#define WIFI_TX 8
#define WIFI_RX 9

#define ENA 5
#define IN1 2
#define IN2 3
#define ENB 6
#define IN3 7
#define IN4 12

#define BASE_PWM 90
#define K_TURN 120.0f
#define K_STRAIGHT 80.0f

static float nA(float a){while(a>PI)a-=2*PI;while(a<-PI)a+=2*PI;return a;}

static void mL(int p){
  int s=abs(p);
  analogWrite(ENA,s);
  if(p>=0){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);}
  else{digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);}
}

static void mR(int p){
  int s=abs(p);
  analogWrite(ENB,s);
  if(p>=0){digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}
  else{digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}
}

static void setM(int L,int R){
  L=constrain(L,-255,255);
  R=constrain(R,-255,255);
  mL(L);
  mR(R);
}

static void brake(){
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

static bool waitVis(uint32_t t=5000){
  uint32_t s=millis();
  while(millis()-s<t){
    if(Enes100.isVisible())return true;
    delay(30);
  }
  return false;
}

static void turnTo(float tgt,uint32_t t=3500){
  uint32_t s=millis();
  while(millis()-s<t){
    float e=nA(tgt-Enes100.getTheta());
    int pwm=(int)constrain(K_TURN*e,-180,180);
    if(fabs(e)<0.03f)break;
    setM(-pwm,pwm);
    delay(10);
  }
  brake();
  delay(100);
}

static void turnBy(float d){
  turnTo(nA(Enes100.getTheta()+d));
}

static void straightHold(float sec,int base=BASE_PWM){
  float th0=Enes100.getTheta();
  uint32_t end=millis()+(uint32_t)(sec*1000);
  while((int32_t)(end-millis())>0){
    float e=nA(th0-Enes100.getTheta());
    int c=(int)constrain(K_STRAIGHT*e,-80,80);
    setM(base-c,base+c);
    delay(10);
  }
  brake();
}

static void demoA(){
  waitVis();
  float y0=Enes100.getY();
  straightHold(3.0f,BASE_PWM);
  float y1=Enes100.getY();
  Enes100.print("A lateral(m): ");
  Enes100.println(fabs(y1-y0));
}

static void demoB(){
  for(int i=0;i<3;i++){
    turnBy(PI/2.0f);
    brake();
    delay(1000);
  }
  Enes100.println("B done");
}

static void demoC(){
  if(!waitVis()){
    Enes100.println("C not visible");
    return;
  }
  float x=Enes100.getX(),y=Enes100.getY(),th=Enes100.getTheta();
  const char* lr=(x<2.0f)?"LEFT":"RIGHT";
  const char* tb=(y<1.0f)?"BOTTOM":"TOP";
  Enes100.print("x=");Enes100.print(x);
  Enes100.print(" y=");Enes100.print(y);
  Enes100.print(" th=");Enes100.print(th);
  Enes100.print(" zone ");Enes100.print(lr);
  Enes100.print("-");Enes100.println(tb);
}

void setup(){
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
  Serial.begin(115200);
  Enes100.begin(TEAM_NAME,MISSION_TYPE,MARKER_ID,ROOM_NUMBER,WIFI_TX,WIFI_RX);
  Enes100.println("ready a/b/c/r");
}

void loop(){
  if(Serial.available()){
    char c=Serial.read();
    if(c=='a')demoA();
    else if(c=='b')demoB();
    else if(c=='c')demoC();
    else if(c=='r'){demoA();demoB();demoC();}
  }
}
