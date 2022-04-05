#include <Arduino.h>
#include "ESP32_S2_TimerInterrupt.h"

#include <stdlib.h>
#include <string.h>

#define PWMPin1 3
#define PWMPin2 2
#define hallA 5
#define hallB 4
#define inputPin 1
float v;
int tick = 0;
float pos = 0;

int test = 0;



float duty, des, w, posOld;
bool forw;

int lastHall = 2;
float vel, error, volt;
float testCurrent = 0;
int sense1, sense2, senseOld1, senseOld2;

ESP32Timer ITimer0(0);
void setupPins();
void calibrateMotor();
void interruptA();
void interruptB();




bool IRAM_ATTR PWMLoop(void* timerNo) {

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
    test = test + 1;
    return true;
}

void setupTimer() {



    ITimer0.attachInterruptInterval(1000, PWMLoop);

}


void setup() {

    Serial.begin(9600);

    setupPins();

    calibrateMotor();

    senseOld1 = digitalRead(hallA);
    senseOld2 = digitalRead(hallB);

    attachInterrupt(digitalPinToInterrupt(hallA), hallSense, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallB), hallSense, CHANGE);
    delay(2000);


    /*
    Timer1.initialize(1000);                // Initializes as 1ms timer
    Timer1.attachInterrupt(PWMLoop);
    */

    setupTimer();


    interrupts();

    delay(1000);

}


void hallSense() {


    noInterrupts();

    sense1 = digitalRead(hallA);
    sense2 = digitalRead(hallB);

    if (senseOld1 && senseOld2) {
        if (sense1 && !sense2)
            tick--;
        if (!sense1 && sense2)
            tick++;
    }
    else if (senseOld1 && !senseOld2) {
        if (!sense1 && !sense2)
            tick--;
        if (sense1 && sense2)
            tick++;
    }
    else if (!senseOld1 && !senseOld2) {
        if (!sense1 && sense2)
            tick--;
        if (sense1 && !sense2)
            tick++;
    }
    else if (!senseOld1 && senseOld2) {
        if (sense1 && sense2)
            tick--;
        if (!sense1 && !sense2)
            tick++;
    }

    pos = tick / 65.25 / 2;

    senseOld1 = sense1;
    senseOld2 = sense2;

    interrupts();

}


void interruptA() {


    noInterrupts();

    if (lastHall == 2)
        lastHall = 0;

    if (lastHall == 0)
        forw = !forw;

    lastHall = 0;

    if (forw)
        tick++;
    else
        tick--;

    pos = tick / 65.25 / 2;

    interrupts();

}

void interruptB() {
    noInterrupts();

    if (lastHall == 2)
        lastHall = 1;

    if (lastHall == 1)
        forw = !forw;

    lastHall = 1;

    if (forw)
        tick++;
    else
        tick--;

    pos = tick / 65.25 / 2;

    interrupts();


}






void loop() {
    // put your main code here, to run repeatedly:
    float x;


    Serial.print("pos: ");
    Serial.println(pos);
    Serial.print("des: ");
    Serial.println(des);

    x = averageReading(inputPin) * 0.393938 * 3.3 / 8192;

    des = x;



    des = x;        // des is 1.5 at 3.3v reading

    delay(1);

    if (pos < -0.1 || pos>2.1) {
        while (1) {
            noInterrupts();
            analogWrite(PWMPin1, 0);
            analogWrite(PWMPin2, 0);
        }
    }
    Serial.println(test);
    test = 0;
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
    pinMode(inputPin, INPUT);

}

float averageReading(int pin) {
    float x = 0;
    for (int n = 0; n < 20; n++)
        x = x + analogRead(pin);
    x = x / 20;
    return x;
}