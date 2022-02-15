#include <Arduino.h>
#include "HX711-multi.h"

// Pins to the load cell amp
#define CLK D0      // clock pin to the load cell amp
#define DOUT1 D1    // data pin to the first lca
#define DOUT2 D2    // data pin to the second lca

// Load Cell Definitions
#define TARE_TIMEOUT_SECONDS 4

byte DOUTS[2] = {DOUT1, DOUT2};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

long int results[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

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

//=============================================================================================
//                         FNC Definition
//=============================================================================================

void IRAM_ATTR interruptA();
void motorRotation(bool);
void sendRawData();

//=============================================================================================
//                         SETUP
//=============================================================================================
void setup() {
  // Pin Modes
  pinMode(CLK,OUTPUT);
  scales.set_gain(32);
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
  Serial.print("RPMs:\t");
  Serial.print("Pulses:\t");
  Serial.begin(115200);
}
 
//=============================================================================================
//                         LOOP
//=============================================================================================
void loop() {
  Serial.println();
  value = map(analogRead(PWM), 0, 1023, 35, 85); //35 is cut off, -> 45
  analogWrite(enB, value);
  Serial.print(value);
  Serial.print("\t");
  Serial.print(pulses);

  // Load Cells
  if (scales.is_ready())
  {
    sendRawData();}

  // Direction Logic
  direction = digitalRead(Dir);
  if (direction != prevDir) // conditional to only call function on change
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

void sendRawData() {
  scales.read(results);
  for (int i=0; i<scales.get_count(); ++i) {;
    Serial.print("\t");
    Serial.print( -results[i]);  
    Serial.print( (i!=scales.get_count()-1)?"\t":"\n");
  }  
  delay(10);
}

void IRAM_ATTR interruptA(){
  if (direction){
    pulses++;
  } else {
    pulses--;
  }
}//end Interrupt Service Routine (ISR)