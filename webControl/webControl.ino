/*
 * MEAM510 LAB4_1_4 control Motor with Web
 * Nov 2021
 * 
 * Haitao Zhu
 * University of Pennsylvania
 * copyright (c) 2021 All Right Reserved
 */

#include "MOTORJS.h"// contains string "body" html code
#include "html510.h"
#include <Adafruit_MotorShield.h>
#include "Adafruit_VL53L0X.h"
#include <HCSR04.h>

#define PIN1 14 //left PD
#define PIN2 27 //right PD
unsigned long last_time = 0;
int last_state = 0;

UltraSonicDistanceSensor distanceSensor(26, 25);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_DCMotor *RFMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RRMotor = AFMS.getMotor(2);
Adafruit_DCMotor *LFMotor = AFMS.getMotor(3);
Adafruit_DCMotor *LRMotor = AFMS.getMotor(4);

#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

void goForwBackw(int);// control forward, backward ,brake
void goLeftRight(int);// control left,right
void rotLeftRight(int);// control rotate
void angleRot(int);
bool isReceived(int);

HTML510Server h(80);
const char* ssid      = "Haitao_RACE";

unsigned long UdpTime = millis();

 /************/
 /* web handler */
void handleRoot(){
  h.sendhtml(body);
}
void handleFull(){
  goForwBackw(255);
  }
void handleWall(){
  unsigned long ms = millis();
  while ((millis()-ms)<10000){
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      uint16_t result;
      if (measure.RangeStatus != 4){result = measure.RangeMilliMeter;}
      else{result = 510;}
      if (result < 150){
        Serial.println("Turn");
        angleRot(-90);//if use battery -190; sensor error;
        delay(100);
      }
      else{
          goForwBackw(-100);
          Serial.println("go backward");
          delay(100);
        }
      
      goForwBackw(0);
  }
  goForwBackw(0);
  
}
void handleBeacon(){ 
  unsigned long ms = millis();
  while ((millis()-ms)<2000){
    bool lr = isReceived(PIN1);
    bool rr = isReceived(PIN2);
    while (!(lr&&rr)){
      if(!lr){
        rotLeftRight(50);
        delay(300);
        rotLeftRight(0);
        }
       else{
        rotLeftRight(-50);
        delay(300);
        rotLeftRight(0);
        }
        lr = isReceived(PIN1);
        rr = isReceived(PIN2);
      }
    
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    uint16_t result;
    if (measure.RangeStatus != 4){result = measure.RangeMilliMeter;}
    else{result = 510;}
    while (result>100){
      goForwBackw(-result/2);
      lr = isReceived(PIN1);
      rr = isReceived(PIN2);
      while (!(lr&&rr)){
        if(!lr){
          rotLeftRight(50);
          delay(100);
          rotLeftRight(0);
          }
        else{
          rotLeftRight(-50);
          delay(100);
          rotLeftRight(0);
          }
          lr = isReceived(PIN1);
          rr = isReceived(PIN2);
        }
      lox.rangingTest(&measure, false);
      if (measure.RangeStatus != 4){result = measure.RangeMilliMeter;}
      else{result = 510;}
      }
      goForwBackw(0);
  }
  goForwBackw(0);      
}
void handleStop(){
  goForwBackw(0);
  }
void handleReve(){//low speed reverse
  goForwBackw(-255);
  }
void handleSlef(){// low speed right turn
  goLeftRight(100);
  }
void handleSrig(){// low speed left turn
  goLeftRight(-255);
  }
void handleGlef(){// ground turn right
  rotLeftRight(255);
  }
void handleGrig(){// ground turn left
  rotLeftRight(-255);
  }

/*************/

void setup(){
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  while (!AFMS.begin()){
    //Serial.println("Could not find motor shield.Disaster!!!");
    delay(2000);
  }
  while (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  pinMode(PIN1,INPUT);
  pinMode(PIN2,INPUT);
  WiFi.softAP(ssid);
  WiFi.softAPConfig(IPAddress(192,168,1,2),IPAddress(192,168,1,2),IPAddress(255,255,255,0));

  h.begin();
  h.attachHandler("/",handleRoot);
  h.attachHandler("/hit_full_speed",handleFull);
  h.attachHandler("/hit_wall",handleWall);
  h.attachHandler("/hit_beacon",handleBeacon);
  h.attachHandler("/hit_STOP_BUTTOM",handleStop);
  h.attachHandler("/hit_reverse",handleReve);
  h.attachHandler("/slow_right",handleSrig);
  h.attachHandler("/slow_left",handleSlef);
  h.attachHandler("/ground_right",handleSrig);
  h.attachHandler("/ground_left",handleSlef);

}

void loop(){
  h.serve();
  delay(10);
}

void goForwBackw(int vel){
  //+: forward, -:backward
  int dir = BRAKE;
  if (vel > 0){
    dir = FORWARD;
  }
  else if(vel <0){
    vel = -vel;
    dir = BACKWARD;
    }
  vel = min(vel,255);
  if (vel!=0)  vel = max(vel,50);
  LFMotor->setSpeed(vel);
  LRMotor->setSpeed(vel);
  RRMotor->setSpeed(vel);
  RFMotor->setSpeed(vel);
  LFMotor->run(dir);
  LRMotor->run(dir);
  RFMotor->run(dir);
  RRMotor->run(dir);
}
void goLeftRight(int vel){
  // -: left, +:right
  int dir = BRAKE;
  if (vel > 0){
    dir = FORWARD;
  }
  else if(vel <0){
    vel = -vel;
    dir = BACKWARD;
  }
  vel = min(vel,255);
  if (vel!=0)  vel = max(vel,50);
  LFMotor->setSpeed(vel);
  LRMotor->setSpeed(vel);
  RRMotor->setSpeed(vel);
  RFMotor->setSpeed(vel);
  LFMotor->run(dir);
  LRMotor->run(3-dir);
  RFMotor->run(3-dir);
  RRMotor->run(dir); 
}
void rotLeftRight(int vel){
  // -left , +backward
  int dir = BRAKE;
  if (vel > 0){
    dir = FORWARD;
  }
  else if(vel <0){
    vel = -vel;
    dir = BACKWARD;
  }
  vel = min(vel,255);
  if (vel!=0)  vel = max(vel,50);
  LFMotor->setSpeed(vel);
  LRMotor->setSpeed(vel);
  RRMotor->setSpeed(vel);
  RFMotor->setSpeed(vel);
  LFMotor->run(dir);
  LRMotor->run(dir);
  RFMotor->run(3-dir);
  RRMotor->run(3-dir); 
}
void angleRot(int D_angle){
  mpu6050.update();
  int r_pos1 = -int(mpu6050.getAngleZ());
  int r_pos2;
  int delta = D_angle;
  while(abs(delta)>1){
    rotLeftRight(delta);
    delay(80);
    goForwBackw(0);
    mpu6050.update();
    r_pos2 = -int(mpu6050.getAngleZ());
    delta = delta-(r_pos2-r_pos1);
    r_pos1 = r_pos2;
    Serial.println(delta);
   }
  goForwBackw(0);
}

bool isReceived(int ch){
  unsigned long last_time = 0;
  int last_state = 0;
  unsigned long t = millis();
  while ((millis()-t) <300){
    unsigned long us = micros();
    if ( digitalRead(PIN1) != last_state){
      last_state =1 - last_state;
      //Serial.println(last_state);
      if (last_state == 1){
        int period = us-last_time;
            Serial.println(period);
        if (period <300){}
        else if (((period > 1000000/700-400)&&(period < 1000000/700+400)) || ((period > 1000000/23/2 - 3000) && (period < 10000000/23-3000))){
            return true;}
        else{}
        last_time = us;
        }
    }
  }
  return false;
}
