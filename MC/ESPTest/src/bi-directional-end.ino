/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include "ESP32_S2_TimerInterrupt.h"

#include <stdlib.h>
#include <string.h>

#define PWMPin1 3
#define PWMPin2 2
#define hallA 5
#define hallB 4
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "HX711.h"

#define LoadCell_DOUT 11
#define LoadCell_CLK 10

float v;
int tick = 0;

float pos = 0;





float duty, des, w, posOld;



float vel, error, volt;

int sense1,sense2,senseOld1,senseOld2;

ESP32Timer ITimer0(0);
void setupPins();
void calibrateMotor();
void interruptA();
void interruptB();


HX711 loadcell;
// Global copy of slave
#define CHANNEL 11

// REPLACE WITH THE MAC Address of your receiver 
//7C:DF:A1:35:8E:E4

uint8_t broadcastAddress[] = {0x7c, 0xdf, 0xa1, 0x35, 0x8e, 0xe4};

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

bool IRAM_ATTR PWMLoop(void * timerNo) {

    if (des<0){
      error = pos;
    } else
    error = pos - des;

    if (abs(error) < 1.0 / 60)
        error = 0;

    w = w + error * 0.001;

    volt = -5.3183 * pos - 55.6771 * w;
    duty = volt * 100 / 5;

    if (duty > 0) {
        analogWrite(PWMPin2, duty * 1024 / 100);
        analogWrite(PWMPin1, 0);
    }
    else {
        analogWrite(PWMPin1, -duty * 1024 / 100);
        analogWrite(PWMPin2, 0);
    }

    return true;
}

void setupTimer() {



    ITimer0.attachInterruptInterval(1000, PWMLoop);

}

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

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void AddPeer(){
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 11;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void setUpLoadCell() {
    loadcell.begin(LoadCell_DOUT, LoadCell_CLK);
    Serial.println("Before setting up the scale:");
    Serial.print("read: \t\t");
    Serial.println(loadcell.read());      // print a raw reading from the ADC
  
    Serial.print("read average: \t\t");
    Serial.println(loadcell.read_average(20));   // print the average of 20 readings from the ADC
  
    Serial.print("get value: \t\t");
    Serial.println(loadcell.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)
  
    Serial.print("get units: \t\t");
    Serial.println(loadcell.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
              // by the SCALE parameter (not set yet)
  
    loadcell.set_scale(864673.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
    loadcell.tare();               // reset the scale to 0
  
    Serial.println("After setting up the scale:");
  
    Serial.print("read: \t\t");
    Serial.println(loadcell.read());                 // print a raw reading from the ADC
  
    Serial.print("read average: \t\t");
    Serial.println(loadcell.read_average(20));       // print the average of 20 readings from the ADC
  
    Serial.print("get value: \t\t");
    Serial.println(loadcell.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()
  
    Serial.print("get units: \t\t");
    Serial.println(loadcell.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
              // by the SCALE parameter set with set_scale
  
    Serial.println("Readings:");

    delay(1000);
  }
  
void hallSense() {


    noInterrupts();

    sense1 = digitalRead(hallA);
    sense2 = digitalRead(hallB);

    if(senseOld1 && senseOld2){
      if(sense1 && !sense2)
        tick--;
      if(!sense1 && sense2)
        tick++;
    } else if(senseOld1 && !senseOld2){
      if(!sense1 && !sense2)
        tick--;
      if(sense1 && sense2)
        tick++;
    } else if(!senseOld1 && !senseOld2){
      if(!sense1 && sense2)
        tick--;
      if(sense1 && !sense2)
        tick++;
    } else if(!senseOld1 && senseOld2){
      if(sense1 && sense2)
        tick--;
      if(!sense1 && !sense2)
        tick++;
    }

    pos = tick / 65.25 / 2;

    senseOld1 = sense1;
    senseOld2 = sense2;

    interrupts();

}

void calibrateMotor() {
    analogWrite(PWMPin1, 0);
    analogWrite(PWMPin2, 1024 * 0.04);

    delay(200);
    analogWrite(PWMPin1, 1024 * 0.04);
    analogWrite(PWMPin2, 0);
    delay(1000);
    analogWrite(PWMPin1, 0);
    analogWrite(PWMPin2, 0);

    delay(1000);

    tick = 0;
    pos = 0;
    des = 0;
    w = 0;
    Serial.println("Calibration Finished");
}



void setupPins() {
    pinMode(PWMPin1, OUTPUT);   // Setting PWM pins as outputs
    pinMode(PWMPin2, OUTPUT);

    pinMode(hallA, INPUT_PULLUP);
    pinMode(hallB, INPUT_PULLUP);



}

float averageReading(int pin) {
    float x = 0;
    for (int n = 0; n < 20; n++)
        x = x + analogRead(pin);
    x = x / 20;
    return x;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
//  Force Channel (this ifxes not found)
  ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL,WIFI_SECOND_CHAN_NONE));
  // This is the mac address of this in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init Load Cell
  setUpLoadCell();

  // Init ESP-NOW
  InitESPNow();

  AddPeer();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

    setupPins();

    calibrateMotor();

    senseOld1 = digitalRead(hallA);
    senseOld2 = digitalRead(hallB);

    attachInterrupt(digitalPinToInterrupt(hallA), hallSense, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallB), hallSense, CHANGE);

    delay(1000);

    delay(350);

    
    delay(2000);

    setupTimer();


    interrupts();

    delay(1000);

}

void sendData() {
  Serial.print("Sending: "); Serial.println(sentReadings.pres);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sentReadings, sizeof(sentReadings));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

 
void loop() {
  getReadings();
 
  // Set values to send
  sentReadings.pos = -1;
  sentReadings.pres = pressure;

  // Send message via ESP-NOW
  sendData();
  
  updateDisplay();

  if (pos < -0.1 || pos>2.1) {
        while (1) {
            noInterrupts();
            analogWrite(PWMPin1, 0);
            analogWrite(PWMPin2, 0);
        }
    }
  delay(1000);
}

void getReadings(){
  if (loadcell.is_ready()) {
        long reading = loadcell.read();
        Serial.print("HX711 reading: ");
        pressure = (reading - 864673)/1000;
        Serial.println(pressure);
        } else {
          Serial.println("HX711 not found.");
        }
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
