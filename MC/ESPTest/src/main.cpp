#include <Arduino.h>

// Motor
#define enB D8
#define in4 D7
#define in3 D6


// Button
#define button D3
int pressed = false;

// Hall Sensors
#define HallA D5
#define HallB D2

// ISR
volatile int pulses = 0;//if the interrupt will change this value, it must be volatile

void IRAM_ATTR interruptA();

void IRAM_ATTR interruptB();
//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  // Pin Modes
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(button, INPUT);
  pinMode(HallA, INPUT);
  digitalWrite(HallA, HIGH);//enable internal pullup resistor
  pinMode(HallB, INPUT);
  digitalWrite(HallB, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(HallA), interruptA, CHANGE);//Interrupt initialization
  attachInterrupt(digitalPinToInterrupt(HallB), interruptB, CHANGE);//Interrupt initialization
  // Set initial rotation direction
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  Serial.begin(115200);
  analogWrite(enB, 50);
}
 
//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {
  Serial.println(pulses);//see the counts advance
  // Motor off if button is pressed
  if (digitalRead(button) == false) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  delay(100);
}

void IRAM_ATTR interruptA(){
  if (digitalRead(HallB) == 0) {
    if (digitalRead(HallA)== 0){
      pulses--; //moving in revers
    } else {
      pulses++; // Moving Forward
    }
  } else {
    if (digitalRead(HallA) ==0){
      pulses++; // Moving Forward
    } else {
      pulses--;
    } 
  }
}//end Interrupt Service Routine (ISR)

void IRAM_ATTR interruptB(){
  if (digitalRead(HallA) == 0) {
    if (digitalRead(HallB)== 0){
      pulses++; 
    } else {
      pulses--; 
    }
  } else {
    if (digitalRead(HallB) ==0){
      pulses--; 
    } else {
      pulses++;
    } 
  }
}//end Interrupt Service Routine (ISR)