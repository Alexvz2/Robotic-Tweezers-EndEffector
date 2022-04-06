/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include "ESP32_S2_TimerInterrupt.h"

#include <stdlib.h>
#include <string.h>


#define hallA2 9
#define hallB2 8

#define clientPWM1 7
#define clientPWM2 6





int sense12,sense22,senseOld12,senseOld22;


float des;

ESP32Timer ITimer0(0);
void setupPins();
void calibrateMotor();
void interruptA();
void interruptB();

// REPLACE WITH THE MAC Address of your receiver 
//7c:df:a1:15:30:ce

uint8_t broadcastAddress[] = {0x7c, 0xdf, 0xa1, 0x15, 0x30, 0xce};

// Define variables to store BME280 readings to be sent
long pressure; //pressure
float positionM; //position

// Define variables to store incoming readings
long InPres;
float InPos;

// Variable to store if sending data was successful
String success;



//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
  long pres; //pressure
  float pos; //position
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message sentReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  InPos = incomingReadings.pos;
  InPres = incomingReadings.pres;
}

void hallSense2() {


    noInterrupts();

    sense12 = digitalRead(hallA2);
    sense22 = digitalRead(hallB2);

    if(senseOld12 && senseOld22){
      if(sense12 && !sense22)
        des = des + 1/65.25/2*(1.3/0.4);
      if(!sense12 && sense22)
        des = des - 1/65.25/2*(1.3/0.4);
    } else if(senseOld12 && !senseOld22){
      if(!sense12 && !sense22)
        des = des + 1/65.25/2*(1.3/0.4);
      if(sense12 && sense22)
        des = des - 1/65.25/2*(1.3/0.4);
    } else if(!senseOld12 && !senseOld22){
      if(!sense12 && sense22)
        des = des + 1/65.25/2*(1.3/0.4);
      if(sense12 && !sense22)
        des = des - 1/65.25/2*(1.3/0.4);
    } else if(!senseOld12 && senseOld22){
      if(sense12 && sense22)
        des = des + 1/65.25/2*(1.3/0.4);
      if(!sense12 && !sense22)
        des = des - 1/65.25/2*(1.3/0.4);
    }

    

    senseOld12 = sense12;
    senseOld22 = sense22;

    interrupts();

}

void calibrateMotor() {

    analogWrite(clientPWM1, 1024*0.04);
    analogWrite(clientPWM2, 0);
    delay(200);

    delay(1000);

    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 0);
    delay(1000);



    des = 1.3;


}



void setupPins() {
      // Setting PWM pins as outputs

    pinMode(clientPWM1, OUTPUT);
    pinMode(clientPWM2, OUTPUT);

    pinMode(hallA2, INPUT_PULLUP);
    pinMode(hallB2, INPUT_PULLUP);


}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int chan=11;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan,WIFI_SECOND_CHAN_NONE));
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 11;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  setupPins();

    calibrateMotor();



    attachInterrupt(digitalPinToInterrupt(hallA2), hallSense2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallB2), hallSense2, CHANGE);
    delay(1000);
    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 1024*0.04);
    delay(350);
    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 0);
    
    delay(2000);

    interrupts();

    delay(1000);

    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 1024*0.04);
}
 
void loop() {
  getReadings();
 
  // Set values to send
  sentReadings.pos = positionM;
  sentReadings.pres = pressure;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sentReadings, sizeof(sentReadings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  updateDisplay();

  if (des < 0){
      analogWrite(clientPWM1, 0);
      analogWrite(clientPWM2, 0);
    } else {
      analogWrite(clientPWM1, 0);
      analogWrite(clientPWM2, 1024*0.04);
    }
  delay(10);
}
void getReadings(){
  positionM = 1;
  pressure = 2;
}

void updateDisplay(){
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("Poisition: ");
  Serial.print(incomingReadings.pos);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(incomingReadings.pres);
  Serial.println(" hPa");
  Serial.println();
}
