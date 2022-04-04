/*
 *   Communication ESP32 for MEAM510 Final Project
 *   class for tracking multiple diodes under a vive basestation (1.0)
 *   
 *   Upload to ESP32  2045
 *
 *   Main task : 
 *   Sensor read(VIVE, limit_sw, distance measurement (rear site), orientation measurement), 
 *   calculate self pos and ori,
 *   UDP broadcast and receive position, 
 *   ESPNOW send self pos ori to 2026/FeatherS2(onrequest or not),
 *   send Nearest CAN pos that outsides team bonus area to 2026/FeatherS2(onrequest or not)
 *   
 *   Finite State machine
 *   
 *   PIN reservation : 
 *   14,27 for vive; 21,22 for I2C; 25,26 for SelfID
 *   may still need SPI for device communication
 *   
 *   
 *
  */

#include "vive510.h"
#define SIGNALPIN1 14 // left VIVE  pin receiving signal from Vive circuit
#define SIGNALPIN2 27 //right VIVE
Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

#define IDPIN1 26
#define IDPIN2 25

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
int LeftVIVE[2];
int RightVIVE[2];
int OrienVIVE;
int SelfId;
int SelfPose[2];
int CenterBoundary[4];//x1,y1,x2,y2

//#include <esp_now.h>
//esp_now_peer_info_t peer1 = {
//  .peer_addr = {0xD8,0xA0,0x1D,0x5D,0xD7,0xC8},//Mac addr for 2024
//  .channel = 1,
//  .encrypt = false,
//};

// multitask define, using core 1;
// core 0 is used for wifi function, better avoid
// Task:read sensor if needed
//      read VIVE X,Y and get avr X,Y \theta
//      send X,Y at 1 Hz
//      read UDP to get CAN,X,Y
//      read UDP to get other's pos and avoid collision
//void TaskReadSensor(void *pvParameter);
//void TaskReadVive(void *pvParameter);
//void TasksendXY(void *pvParameter);

// or just call micro() and millis()



// some subroutine
void UdpSender(char*, int);
void UdpCanReader();
void UdpRobotReader();
void ViveUpdate();

unsigned long ViveTime = millis();
unsigned long UdpTime = millis();
unsigned long EspTime = millis();


//
void setup(){
  WiFi.mode(WIFI_AP_STA);

  pinMode(IDPIN1,INPUT_PULLUP);
  pinMode(IDPIN2,INPUT_PULLUP);
  Serial.begin(115200);
  vive1.begin();//setup vive
  vive2.begin();
  Serial.println("Vive trackers started");

  SelfId = 1 + digitalRead(IDPIN1) + 2*digitalRead(IDPIN2);
  Serial.println(SelfId);
  
  //setup UDP
  WiFi.config(ipLocal,IPAddress(192,168,1,1),IPAddress(255,255,254,0));
  WiFi.begin(ssid,password);
  UdpRobotServer.begin(2510);
  UdpCanServer.begin(1510);

  Serial.println("UDP started!");

  //setup ESP-NOW
//  if (esp_now_init() != ESP_OK){
//    Serial.println("Init ESP failed");
//    while (1);
//  }
//  
//  if (esp_now_add_peer(&peer1) != ESP_OK){
//    Serial.println("Pair failed");
//    while(1);
//  }
  

  //setup I2C sensor
}


void loop() {
  if ((millis()-ViveTime) > 100){
    ViveUpdate();
    ViveTime = millis();
  }
  if ((millis()-UdpTime) > 500){
    UdpTime = millis();
    for (int i=1;i<4;i++){
      UdpRobotReader();
      UdpCanReader();
      delay(10);
      //erial.println("UDP received");
    }
  }
  
  if ((millis()-EspTime) > 1000){
    EspTime = millis();
    SelfPose[0] = (int)(LeftVIVE[0]+RightVIVE[0])/2;
    SelfPose[1] = (int)(LeftVIVE[1]+RightVIVE[1])/2;
    OrienVIVE = int((atan2(RightVIVE[1]-LeftVIVE[1],RightVIVE[0]-LeftVIVE[0]))/PI*180+90);
    char s[13];
    SelfId = 1 + digitalRead(IDPIN1) + 2*digitalRead(IDPIN2);

    Serial.print(SelfId);
    Serial.print(",");
    Serial.print(SelfPose[0]);
    Serial.print(",");
    Serial.print(SelfPose[1]);
    Serial.print(",");
    Serial.print(OrienVIVE);
    Serial.println();
    if (SelfPose[0]<1000 || SelfPose[1]<1000 || SelfPose[0]>9000 || SelfPose[0]>9000){
      sprintf(s,"%1d:%4d,%4d",SelfId,0,0);
    }else{
      sprintf(s,"%1d:%4d,%4d",SelfId,SelfPose[0],SelfPose[1]);
    }
    s[12] = 0;
    UdpSender(s,13);

    }
    
//      uint8_t s[1]={1};
//      esp_now_send(peer1.peer_addr,s,sizeof(s));
//      Serial.println(SelfId);
//      Serial.print(SelfPose[0],SelfPose[1]);
//  }
} 
void UdpSender(char* datastr, int len){
  UdpRobotServer.beginPacket(ipTarget,2510);
  UdpRobotServer.write((uint8_t *)datastr,len);
  UdpRobotServer.endPacket();
}

void UdpCanReader(){
  uint8_t packetBuffer[14];
  int cb = UdpCanServer.parsePacket();
  if (cb){
    int CanId;
    packetBuffer[cb]=0;
    UdpCanServer.read(packetBuffer,14);
    CanId = atoi((char*)packetBuffer);
    CanPose[CanId][0] = atoi((char*)packetBuffer+2);
    CanPose[CanId][1] = atoi((char*)packetBuffer+7);
    Serial.print(CanId);
    Serial.print(",");
    Serial.print(CanPose[CanId][0]);
    Serial.print(",");
    Serial.print(CanPose[CanId][1]);
    Serial.println();    
    }
}

void UdpRobotReader(){
  uint8_t packetBuffer[14];
  int cb = UdpRobotServer.parsePacket();
  if (cb){
    int RobotId;
    packetBuffer[cb]=0;
    UdpCanServer.read(packetBuffer,14);
    RobotId = atoi((char*)packetBuffer);
    CanPose[RobotId-1][0] = atoi((char*)packetBuffer+2);
    CanPose[RobotId-1][1] = atoi((char*)packetBuffer+7);
  }
}

void ViveUpdate(){
  if (vive1.status() != VIVE_LOCKEDON)    vive1.sync(15);
  else{
    LeftVIVE[0] = vive1.xCoord();
    LeftVIVE[1] = vive1.yCoord();
//    Serial.print(LeftVIVE[0]);
//    Serial.print(LeftVIVE[1]);
      }
  if(vive2.status() != VIVE_LOCKEDON)     vive2.sync(15);
  else{
    RightVIVE[0] = vive2.xCoord();
    RightVIVE[1] = vive2.yCoord();
//    Serial.print(RightVIVE[0]);
//    Serial.print(RightVIVE[1]);}
  }
}
 
