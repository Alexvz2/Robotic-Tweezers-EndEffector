#include <Arduino.h>
// #include <ESP32Encoder.h>


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
bool lock = false;

void IRAM_ATTR interruptA();

// void IRAM_ATTR interruptB();
//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  // Pin Modes
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(button, INPUT);
  pinMode(HallA, INPUT_PULLUP);
  pinMode(HallB, INPUT_PULLUP);
  // pinMode(HallB, INPUT);
  // digitalWrite(HallA, HIGH);//enable internal pullup resistor
  // digitalWrite(HallB, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(HallA), interruptA, CHANGE);//Interrupt initialization
  // attachInterrupt(digitalPinToInterrupt(HallB), interruptB, CHANGE);//Interrupt initialization
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
  if (lock)
  {
    Serial.println(pulses);//see the counts advance
    lock = false;
  }
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
  pulses++;
  lock = true;
  // if( digitalRead(HallB) == 0 ) {
  //   if ( digitalRead(HallA) == 0 ) {
  //     // A fell, B is low
  //     pulses--; // moving reverse
  //   } else {
  //     // A rose, B is low
  //     pulses++; // moving forward
  //   }
  // }else {
  //   if ( digitalRead(HallA) == 0 ) {
  //     // B fell, A is high
  //     pulses++; // moving reverse
  //   } else {
  //     // B rose, A is high
  //     pulses--; // moving forward
  //   }
  // }
}//end Interrupt Service Routine (ISR)

// void IRAM_ATTR interruptB(){
//   if ( digitalRead(HallA) == 0 ) {
//     if ( digitalRead(HallB) == 0 ) {
//       // B fell, A is low
//       pulses++; // moving forward
//     } else {
//       // B rose, A is low
//       pulses--; // moving reverse
//     }
//  } else {
//     if ( digitalRead(HallB) == 0 ) {
//       // B fell, A is high
//       pulses--; // moving reverse
//     } else {
//       // B rose, A is high
//       pulses++; // moving forward
//     }
//   }
// }//end Interrupt Service Routine (ISR)