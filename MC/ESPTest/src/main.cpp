#include <Arduino.h>
#include "HX711-multi.h"

// Pins to the load cell amp
#define CLK D0      // clock pin to the load cell amp
#define DOUT1 D1    // data pin to the first lca
#define DOUT2 D2    // data pin to the second lca

// Button
#define button D3
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastState = LOW; 
int buttonState; 


#define BOOT_MESSAGE "Initiating Multi-Scale"

#define TARE_TIMEOUT_SECONDS 4

byte DOUTS[2] = {DOUT1, DOUT2};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

long int results[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

void tare();
bool buttonPress();

void setup() {
  pinMode(CLK,OUTPUT);
  Serial.begin(115200);
  Serial.println(BOOT_MESSAGE);
  scales.set_gain(32);
  
}

void tare() {
  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = scales.tare(20,10000);
  }
}

void sendRawData() {
  scales.read(results);
  for (int i=0; i<scales.get_count(); ++i) {;
    Serial.print( -results[i]);  
    Serial.print( (i!=scales.get_count()-1)?"\t":"\n");
  }  
  delay(10);
}

void loop() {
  if (scales.is_ready())
  {
    sendRawData();
    if (buttonPress()) {
      tare(); // Tare whenever the button is pressed
    }
  }
  delay(100);
}

bool buttonPress() {
  int reading = digitalRead(button);

  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        return true;
      }
    }
  }
  lastState = reading;
  return false;
}