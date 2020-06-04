/*
  Honda KPro to Renault Clio3 CANBUS Interface Code
  Rhys Oxenham <rdoxenham@gmail.com>
  August 2019
*/

#include "esp32_can.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>

// Bench mode boolean
bool bench_mode = false;

// Main variables
int rpm = 0;
int speed_kph = 0;
int gear = 0;
float voltage = 0;
int intake_temp = 0;
int coolant_temp = 0;
int tps = 0;
int map_kpa = 0;
int distance_from_start = 0;

void set_mil(bool value)
{
  //551#55007060FE0300 on
  //551#55007000FE0200 off
  CAN_FRAME milFrame;
  milFrame.rtr = 0;
  milFrame.id = 0x551;
  milFrame.extended = false;
  milFrame.length = 7;
  milFrame.data.byte[0] = 0x7B;
  milFrame.data.byte[1] = 0x00;
  milFrame.data.byte[2] = 0x70;
  milFrame.data.byte[4] = 0xFE;
  milFrame.data.byte[6] = 0x00;
  if(value == true) {
    milFrame.data.byte[3] = 0x60;
    milFrame.data.byte[5] = 0x03;
    Serial.println("DEBUG: Setting MIL Light ON!");
  }
  else {
    milFrame.data.byte[3] = 0x00;
    milFrame.data.byte[5] = 0x02;
    Serial.println("DEBUG: Setting MIL Light OFF!");
  }
  CAN0.sendFrame(milFrame);
}

void handleCAN0CB(CAN_FRAME *frame)
{
  CAN1.sendFrame(*frame);
}

void handleCAN1CB(CAN_FRAME *frame)
{
  CAN0.sendFrame(*frame);
}

void updateData(CAN_FRAME &frame)
{
  int value = frame.id;
  if(frame.id == 1632){
    rpm = ntohs(frame.data.s0);
    speed_kph = ntohs(frame.data.s1);
    gear = frame.data.byte[4];
    voltage = frame.data.byte[5];
  }
  else if(frame.id == 1633) {
    intake_temp = ntohs(frame.data.s0);
    coolant_temp = ntohs(frame.data.s1);
  }
  else if(frame.id == 1634) {
    tps = ntohs(frame.data.s0);
    map_kpa = ntohs(frame.data.s1);
  }
}

void debugFrame(CAN_FRAME &frame)
{
  // frame.id is a decimal value
  int value = frame.id;

  //Serial.println(frame.id,HEX);
  // decimal 1632 is hex 660
  if(frame.id == 1632){

    // Renault takes (dec*8)2hex for RPM signal
    rpm = ntohs(frame.data.s0);
    Serial.print("Honda RPM: ");
    Serial.println(rpm);
    int rdec_rpm = rpm * 8;

    // Renault clocks can read to 8192rpm in 16bits
    if(rdec_rpm > 65535){
      rdec_rpm = 65535;
    }
    Serial.print("Renault RPM Hex: ");
    Serial.println(rdec_rpm,HEX);

    // Renault takes (kph/0.0108) dec2hex
    speed_kph = ntohs(frame.data.s1);
    Serial.print("Honda Speed (KPH): ");
    Serial.println(speed_kph);
    int rdec_speed = speed_kph / 0.0108;
    Serial.print("Renault Speed Hex: ");
    Serial.println(rdec_speed,HEX);

    gear = frame.data.byte[4];
    // Both cars have 6 gears - avoid faults
    if(gear > 6){
      gear = 6;
    }
    Serial.print("Gear Selected: ");
    Serial.println(gear);

    voltage = frame.data.byte[5];
    Serial.print("Battery Voltage: ");
    float batt = voltage / 10;
    Serial.println(batt, 1);
  }
  // decimal 1633 is hex 661
  else if(frame.id == 1633) {
    intake_temp = ntohs(frame.data.s0);
    Serial.print("Intake Temp: ");
    Serial.println(intake_temp);
    
    coolant_temp = ntohs(frame.data.s1);
    Serial.print("Honda Coolant Temp: ");
    Serial.println(coolant_temp);
    // Renault hex reads +40C from actual
    int rtemp_dec = coolant_temp + 40;
    Serial.print("Renault Coolant Hex: ");
    Serial.println(rtemp_dec,HEX);
  }
  // decimal 1634 is hex 662
  else if(frame.id == 1634) {
    tps = ntohs(frame.data.s0);
    Serial.print("Honda TPS %: ");
    Serial.println(tps);
    Serial.print("Renault TPS Hex: ");
    // Renault TPS is % * 2.08 + 16 (2hex)
    int tps_dec = 16 + (tps * 2.08);
    Serial.println(tps_dec,HEX);

    map_kpa = ntohs(frame.data.s1);
    Serial.print("Boost PSI: ");
    float boost_psi = (map_kpa / 10) * 0.145;
    Serial.println(boost_psi,1);
  }
  else {
    //
  }
  //for(int i = 0;i < frame.length; i++) {
  //Serial.print(frame.data.uint8[i],HEX);
}

void update_rspeed(){
  //Serial.print("[DEBUG] Speed KPH: ");
  //Serial.println(speed_kph);
  CAN_FRAME speedFrame;
  speedFrame.rtr = 0;
  speedFrame.id = 0x354;
  speedFrame.extended = false;
  speedFrame.length = 8;
  
  int rdec_speed = speed_kph / 0.0108;
  speedFrame.data.s0 = ntohs(rdec_speed);
  // KPH to M/0.5s multiplied by 10 for Renault granularity of 0.1m
  float rdec_delta = speed_kph / 9.275;
  rdec_delta = rdec_delta * 25;
  int rdec_delta_int = round(rdec_delta);
  //Serial.print("[DEBUG] Delta: ");
  //Serial.println(rdec_delta_int);
  distance_from_start = distance_from_start + rdec_delta_int;
  speedFrame.data.s1 = ntohs(distance_from_start);
  //Serial.print("[DEBUG] Distance from Start: ");
  //Serial.println(distance_from_start);
  //This is always sent by Clio
  speedFrame.data.byte[6] = 0x04;
  CAN0.sendFrame(speedFrame);
}

void update_renault(){
  //Serial.print("[DEBUG] RPM: ");
  //Serial.println(rpm);
  CAN_FRAME rpmFrame;
  rpmFrame.rtr = 0;
  rpmFrame.id = 0x181;
  rpmFrame.extended = false;
  rpmFrame.length = 8;

  int rdec_rpm = rpm * 8;
  if(rdec_rpm > 65535){
    rdec_rpm = 65535;
  }
  rpmFrame.data.s0 = ntohs(rdec_rpm);
  //Set TPS value for clocks
  //Serial.print("[DEBUG] TPS%: ");
  //Serial.println(tps);
  // Renault TPS is % * 2.08 + 16 (2hex)
  int tps_dec = 16 + (tps * 2.08);
  if(tps_dec > 224){
    tps_dec = 224;
  }
  rpmFrame.data.byte[3] = tps_dec;
  CAN0.sendFrame(rpmFrame);
  
  //Serial.print("[DEBUG] Coolant Temp: ");
  //Serial.println(coolant_temp);
  CAN_FRAME coolantFrame;
  coolantFrame.rtr = 0;
  coolantFrame.id = 0x551;
  coolantFrame.extended = false;
  coolantFrame.length = 7;
  
  int rtemp_dec = coolant_temp + 40;
  coolantFrame.data.byte[0] = rtemp_dec;
  coolantFrame.data.byte[1] = rtemp_dec;
  CAN0.sendFrame(coolantFrame);

  if(bench_mode){
    CAN_FRAME wakeupFrame;
    wakeupFrame.rtr = 0;
    wakeupFrame.id = 0x35D;
    wakeupFrame.extended = false;
    wakeupFrame.length = 8;
    wakeupFrame.data.byte[0] = 0x90;
    wakeupFrame.data.byte[1] = 0x03;
    wakeupFrame.data.byte[2] = 0x00;
    wakeupFrame.data.byte[3] = 0x00;
    wakeupFrame.data.byte[4] = 0x44;
    wakeupFrame.data.byte[5] = 0x01;
    wakeupFrame.data.byte[6] = 0x51;
    wakeupFrame.data.byte[7] = 0x00;
    CAN0.sendFrame(wakeupFrame);

    CAN_FRAME ignitionFrame;
    ignitionFrame.rtr = 0;
    ignitionFrame.id = 0x625;
    ignitionFrame.extended = false;
    ignitionFrame.length = 6;
    ignitionFrame.data.byte[0] = 0x82;
    ignitionFrame.data.byte[1] = 0x40;
    ignitionFrame.data.byte[2] = 0xDD;
    ignitionFrame.data.byte[3] = 0x1D;
    ignitionFrame.data.byte[4] = 0xC0;
    ignitionFrame.data.byte[5] = 0x00;
    CAN0.sendFrame(ignitionFrame);

    //5FD#20CC2451B9400000
    CAN_FRAME wakeup3Frame;
    wakeup3Frame.rtr = 0;
    wakeup3Frame.id = 0x5FD;
    wakeup3Frame.extended = false;
    wakeup3Frame.length = 8;
    wakeup3Frame.data.byte[0] = 0x00;
    wakeup3Frame.data.byte[1] = 0x00;
    wakeup3Frame.data.byte[2] = 0x00;
    wakeup3Frame.data.byte[3] = 0x00;
    wakeup3Frame.data.byte[4] = 0x00;
    wakeup3Frame.data.byte[5] = 0x00;
    wakeup3Frame.data.byte[6] = 0x00;
    wakeup3Frame.data.byte[7] = 0x00;
    CAN0.sendFrame(wakeup3Frame);
    
    // This is a hack as car should send this.
    CAN_FRAME airbagFrame;
    airbagFrame.rtr = 0;
    airbagFrame.id = 0x651;
    airbagFrame.extended = false;
    airbagFrame.length = 2;
    airbagFrame.data.byte[0] = 0x04;
    airbagFrame.data.byte[1] = 0xF1;
    CAN0.sendFrame(airbagFrame);
  
    // This is a hack as car should send this.
    CAN_FRAME steeringFrame;
    steeringFrame.rtr = 0;
    steeringFrame.id = 0x5E4;
    steeringFrame.extended = false;
    steeringFrame.length = 3;
    steeringFrame.data.byte[0] = 0x00;
    steeringFrame.data.byte[1] = 0x09;
    steeringFrame.data.byte[2] = 0x20;
    CAN0.sendFrame(steeringFrame);
  
    // This is a hack as car should send this.
    CAN_FRAME doorlocksFrame;
    doorlocksFrame.rtr = 0;
    doorlocksFrame.id = 0x60D;
    doorlocksFrame.extended = false;
    doorlocksFrame.length = 8;
    doorlocksFrame.data.byte[0] = 0x04;
    doorlocksFrame.data.byte[1] = 0x16;
    doorlocksFrame.data.byte[2] = 0x00;
    doorlocksFrame.data.byte[3] = 0x00;
    doorlocksFrame.data.byte[4] = 0x2E;
    doorlocksFrame.data.byte[5] = 0x55;
    doorlocksFrame.data.byte[6] = 0x0F;
    doorlocksFrame.data.byte[7] = 0x00;  
    CAN0.sendFrame(doorlocksFrame);
  }
}

void *threadproc(void *arg)
{
    while(true)
    {
        sleep(1);
        update_rspeed();
    }
    return 0;
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("Initialising ...");

  // Initialise builtin CAN controller at the specified speed
  if(CAN0.begin(500000))
  {
    Serial.println("Renault CAN Controller Init OK ...");
  } else {
    Serial.println("Renault CAN Controller Init Failed ...");
  }
  
  // Initialise MCP2517FD CAN controller at the specified speed
  if(CAN1.begin(500000))
  {
    Serial.println("Honda CAN Controller Init OK ...");
  } else {
    Serial.println("Honda CAN Controller Init Failed ...");
  }

  CAN0.setRXFilter(0, 0x100, 0x700, false);
  CAN0.watchFor(); //allow everything else through
  CAN0.setCallback(0, handleCAN0CB);

  CAN1.setRXFilter(0, 0x230, 0x7F0, false);
  CAN1.watchFor(); //allow everything else through
  CAN1.setCallback(0, handleCAN1CB);

  Serial.println("Ready.");

  // Setup the speed/distance thread
  pthread_t tid;
  pthread_create(&tid, NULL, &threadproc, NULL);
}

void loop() {
  static unsigned long lastTime = 0;
  CAN_FRAME message;
  if (CAN1.read(message)) {
    //CAN0.sendFrame(message);
    //debugFrame(message);
    bench_mode = false;
    updateData(message);
    //speed_kph = 120;
    //rpm = 2500;
    update_renault();
  }
  if (CAN1.available() == 0) {
    if ((millis() - lastTime) > 2000) {
      lastTime = millis();
      speed_kph = 0;
    }
  }
}
