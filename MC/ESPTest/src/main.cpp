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
#define RPM_RATIO 125
#define PPR 245 // pulses per revolution
#define ShaftCircumference 22 // 2*pi*r in mm
#define MaxDistance 100 //MM

// ISR
volatile int pulses = 0;//if the interrupt will change this value, it must be volatile
bool lock = false;

void IRAM_ATTR interruptA();
int motorLocation(int);
void motorRotation();

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
  attachInterrupt(digitalPinToInterrupt(HallA), interruptA, CHANGE);//Interrupt initialization
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
  motorRotation();
  delay(100);
}

void motorRotation(){
  if (lock){
    // set motor rotation based on distance
    if (motorLocation(pulses) < MaxDistance) { // true mean cw
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    } else  {
      digitalWrite(in3, LOW); // false mean ccw
      digitalWrite(in4, HIGH);
    }
  }
}

// Return displacement of claw based on pulses
int motorLocation(int pulses){
  return pulses*ShaftCircumference/PPR;
}

void IRAM_ATTR interruptA(){
  lock = false;
  pulses++;
  lock = true;
}//end Interrupt Service Routine (ISR)