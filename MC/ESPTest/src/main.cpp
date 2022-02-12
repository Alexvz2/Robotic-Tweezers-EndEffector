#include <Arduino.h>


// Motor
#define enB D8
#define in4 D7
#define in3 D6

// Potentiometer
#define PWM A0
int value;

// Direction of motor
#define Dir D3
int prevDir = 0;
int direction;

// Hall Sensors
#define HallA D5

// ISR
volatile int pulses = 0;//if the interrupt will change this value, it must be volatile
bool lock = false;

void IRAM_ATTR interruptA();
void motorRotation(bool);

//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  // Pin Modes
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(Dir, INPUT);
  pinMode(HallA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HallA), interruptA, CHANGE);//Interrupt initialization


  // Set initial rotation direction
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  direction = digitalRead(Dir);

  Serial.begin(115200);
  analogWrite(enB, 50);
}
 
//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {
  value = map(analogRead(PWM), 0, 1023, 30, 100);
  Serial.println(value);
  if (lock)
  {
    Serial.println(pulses);//see the counts advance
    lock = false;
  }

  // Direction Logic
  direction = digitalRead(Dir);
  if (direction != prevDir)
  {
    prevDir = direction;
    motorRotation(direction);
  }
  delay(100);
}

void motorRotation(bool rotation){
  if (rotation) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.println("Right");
  } else  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.println("Left");
  }
}

void IRAM_ATTR interruptA(){
  pulses++;
  lock = true;
}//end Interrupt Service Routine (ISR)