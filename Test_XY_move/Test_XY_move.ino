/*
 * MEAM510 Final project test XY move control
 * 
 * MOVE in X,Y direction sperately 
 * or HEADing toward the pos.
 * 
 * IMU Motor and VIVE info from ESP_NOW(or some interrupt)
 * 
 * 
 */
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "vive510.h"
#define SIGNALPIN1 35//left
#define SIGNALPIN2 34//right
Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);
MPU6050 mpu6050(Wire);
#define IDPIN1 37
#define IDPIN2 38
#include <WiFi.h>
#include <WiFiUdp.h>
const char* ssid     = "TP-Link_05AF";
const char* password = "47543454";
WiFiUDP UdpRobotServer;
WiFiUDP UdpCanServer;
IPAddress ipTarget(192,168,1,255);//Broadcast
IPAddress ipLocal(192,168,1,210);
int CanPose[20][2];
int RoboPose[10][2];
int SelfId;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_DCMotor *RFMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RRMotor = AFMS.getMotor(2);
Adafruit_DCMotor *LRMotor = AFMS.getMotor(3);
Adafruit_DCMotor *LFMotor = AFMS.getMotor(4);
void goForwBackw(int);// control forward, backward ,brake
void goLeftRight(int);// control left,right
void rotLeftRight(int);// control rotate
void angleRot(int);
void ViveUpdate();
int GoalPos[2];
int LeftVIVE[2];
int RightVIVE[2];
int SelfPose[2];
int OrienVIVE;
void UdpSender(char*, int);
void UdpCanReader();
void UdpRobotReader();
void ViveUpdate();
unsigned long UdpTime = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 is connected");
  vive1.begin();//setup vive
  vive2.begin();
  while (!AFMS.begin()){
    Serial.println("Could not find motor shield.Disaster!!!");
    delay(2000);
  }
  Serial.println("Motor shield connected");
  for (int i=1;i<10;i++){
    ViveUpdate();
    }
  Serial.println("initalization finish");

  WiFi.config(ipLocal,IPAddress(192,168,1,1),IPAddress(255,255,254,0));
  WiFi.begin(ssid,password);
  UdpRobotServer.begin(2510);
  UdpCanServer.begin(1510);
}

void loop() {
// X,Y move;
  int ADiff = (0-OrienVIVE);
  angleRot(ADiff);
  Serial.println("finish angle");
  
  int XDiff; 
  ViveUpdate();
  XDiff = GoalPos[0]-SelfPose[0];
  while(abs(XDiff)>50){
    goForwBackw(-XDiff/10);
    delay(100);
    goForwBackw(0);
    ViveUpdate();
    XDiff = GoalPos[0]-SelfPose[0];
    Serial.print(SelfPose[0]);
    Serial.print(",1,");
    Serial.print(SelfPose[1]);
    Serial.print(",");
    Serial.print(OrienVIVE);
    Serial.println();
  }
  Serial.println("finish X");
  
  int YDiff;
  ViveUpdate();
  YDiff = GoalPos[1]-SelfPose[1];
  while(abs(YDiff)>50){
    goLeftRight(-YDiff/10);
    ViveUpdate();
    delay(100);
    goForwBackw(0);
    YDiff = GoalPos[1]-SelfPose[1];
    Serial.print(SelfPose[0]);
    Serial.print(",2,");
    Serial.print(SelfPose[1]);
    Serial.print(",");
    Serial.print(OrienVIVE);
    Serial.println();
  }
  Serial.println("finish Y");
  Serial.println("finish");
  delay(10000);
  ///--------------------------------------------------///
  /// or use direct move
  ///***************************************************///
//  int tAngle = int(atan2(SelfPose[1]-GoalPos[1],SelfPose[0]-GoalPos[0]));
//  int ADiff;
//  OrienVIVE -= 50;for sync
//  int TrueAngle = OrienVIVE;
//  ADiff = tAngle-OrienVIVE;
//  while(abs(ADiff>4)){
//    
//    angleRot(ADiff);
//    while (TrueAngle == OirenVIVE){delay(10);}
//    ADiff = tAngle-OrienVIVE;
//    TrueAngle = OrienVIVE;
//  }
//
//  int dis;
//  SelfPose[1] -= 300;
//  SelfPose[2] -= 300;
//  int TrueSelfPoseY = SelfPose[1];
//  int TrueSelfPoseX = SelfPose[0];
//  while(TrueSelfPoseY==SelfPose[1] && TrueSelfPoseX==SelfPose[0]){delay(10);}
//  dis = int(sqrt((SelfPose[1]-GoalPos[1])*(SelfPose[1]-GoalPos[1])+\
//        (SelfPose[0]-GoalPos[0])*(SelfPose[0]-GoalPos[0])));
//  while (abs(dis)>500){
//    
//    goForwBackw(dis/20);
//    while(TrueSelfPoseY==SelfPose[1] && TrueSelfPoseX==SelfPose[0]){delay(10);}
//    TrueSelfPoseY=SelfPose[1];
//    TrueSelfPoseX=SelfPose[0];
//    dis = int(sqrt((SelfPose[1]-GoalPos[1])*(SelfPose[1]-GoalPos[1])+\
//        (SelfPose[0]-GoalPos[0])*(SelfPose[0]-GoalPos[0])));
//    }
//  Serial.println("Finish");
   ///***********************************************/////  
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

void ViveUpdate(){
  //blocking update until read Valid value;
  bool V1 = false;
  bool V2 = false;
  while (not (V1&&V2)){
    if (not V1){
      if (vive1.status() != VIVE_LOCKEDON)    vive1.sync(15);
      else{
        if (((vive1.xCoord()>1000)) && (vive1.yCoord()>1000)){
          LeftVIVE[0] = vive1.xCoord();
          LeftVIVE[1] = vive1.yCoord();
          V1 = true;
        }
      }
    }
    if (not V2){
      if(vive2.status() != VIVE_LOCKEDON)     vive2.sync(15);
      else{
        if (((vive2.xCoord()>1000)) && (vive2.yCoord()>1000)){
          RightVIVE[0] = vive2.xCoord();
          RightVIVE[1] = vive2.yCoord();
          V2 = true;
        }
      }
    }
    delay(50);
    if (V1&&V2){
      SelfPose[0] = (int)(LeftVIVE[0]+RightVIVE[0])/2;
      SelfPose[1] = (int)(LeftVIVE[1]+RightVIVE[1])/2;
      OrienVIVE = int((atan2(RightVIVE[1]-LeftVIVE[1],RightVIVE[0]-LeftVIVE[0]))/PI*180+90);
    }
  }
  Serial.println("Updated");
}
