#include <Arduino.h>
#include "HX711.h"

#define enB D8
#define in3 D6
#define in4 D7

#define DOUT  D1
#define CLK  D2

#define button D3
int pressed = false;
 
HX711 scale;

//Change this calibration factor as per your load cell once it is found you many need to vary it in thousands
long calibration_factor = -115741; //-106600 worked for my 40Kg max scale setup 
 
//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, CLK);
  long calibration_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(calibration_factor);
  // Pin Modes
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(button, INPUT);

  // Set initial rotation direction
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

}
 
//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {
  analogWrite(enB, 96); // Send PWM signal to L298N Enable pin, 95 is the slowest possible 
  // Read button - Debounce
  if (digitalRead(button) == true) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  if (scale.is_ready()) {
    long reading = abs(scale.read())+calibration_factor;
    Serial.print("HX711 reading: ");
    Serial.println(reading);
  } else {
    Serial.println("HX711 not found.");
  }

  delay(1000);
  
}


// int pwmOutput = 0 ;

// void setup() {
//   Serial.begin(115200);
//   pinMode(enA, OUTPUT);
//   pinMode(in1, OUTPUT);
//   pinMode(in2, OUTPUT);
//   // Set initial rotation direction
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
// }

// void loop() {
//   digitalWrite(LED, HIGH);
//   Serial.println("LED is ON!");
//   delay(1000);
//   pinMode(LED, OUTPUT);
//   if (Serial.available() > 0) {
//     // read the incoming byte:
//     pwmOutput = Serial.read();

//     // say what you got:
//     Serial.print("I received: ");
//     Serial.println(pwmOutput, DEC);
//     // If -1 switch direction
//     if (pwmOutput == -1) {
//       digitalWrite(in1, HIGH);
//       digitalWrite(in2, LOW);
//     } else {
//       analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
//       digitalWrite(in1, LOW);
//       digitalWrite(in2, HIGH);
//     }
//   }
// }