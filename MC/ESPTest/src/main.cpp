#include <Arduino.h>

#include <TimerThree.h>
#include <TimerOne.h>
#include <stdlib.h>
#include <string.h>


float v;
int tick = 0;
float pos = 0;
int PWMPin1 = 7;
int PWMPin2 = 8;

float duty, des, w, posOld;
bool forw;
int hallb = 2;
float i;
float vel, error, volt;
float testCurrent = 0;

void setup() {
    forw = false;
    des = 0;
    posOld = 0;
    w = 0;
    Serial.begin(9600);
    pinMode(PWMPin1, OUTPUT);   // Setting PWM pins as outputs
    pinMode(PWMPin2, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(33, INPUT_PULLUP);
    pinMode(34, INPUT_PULLUP);
    pinMode(38, INPUT);
    pinMode(14, INPUT);
    pinMode(39, INPUT_PULLUP);
    digitalWrite(35, 1);







    analogWrite(PWMPin1, 1024 * 0.06);
    analogWrite(PWMPin2, 0);
    delay(1000);
    analogWrite(PWMPin1, 0);
    analogWrite(PWMPin2, 0);
    delay(1000);

    Serial.println("Calibration Finished");

    attachInterrupt(digitalPinToInterrupt(33), interruptA, CHANGE);

    delay(2000);

    tick = 0;
    pos = 0;
    des = 0;

    Serial.println(3.3 / 1024 * analogRead(38));
    Timer1.initialize(1000);                // Initializes as 1ms timer
    Timer1.attachInterrupt(PWMLoop);
    interrupts();



    delay(1000);
    i = 0;
    for (int n = 0; n < 10; n++)
        i = i + analogRead(14);

    i = i / 10;
    i = (778 - i) * 3.3 / 1024 / 0.185 - 0.35;
    Serial.println(i);



}

void interruptA() {


    noInterrupts();



    if (hallb == 2)
        hallb = digitalRead(34);

    if (digitalRead(34) == hallb)
        forw = !forw;

    hallb = digitalRead(34);

    if (forw)
        tick++;
    else
        tick--;

    pos = tick / 65.25;

    interrupts();


}

void PWMLoop(void) {






    vel = (pos - posOld) / 0.001;



    error = pos - des;

    if (abs(error) < 1.0 / 60)
        error = 0;

    w = w + error * 0.001;

    // volt = -1.632*pos-0.0074*i-5.6408*w;
    volt = -5.3183 * pos - 0.0 * i - 55.6771 * w;
    duty = volt * 100 / 5;

    if (duty > 0) {
        analogWrite(PWMPin2, duty * 1024 / 100);
        analogWrite(PWMPin1, 0);
    }
    else {
        analogWrite(PWMPin1, -duty * 1024 / 100);
        analogWrite(PWMPin2, 0);
    }


    posOld = pos;

}


void loop() {
    // put your main code here, to run repeatedly:
    float x;


    Serial.println(3.3 / 1024 * analogRead(38));
    Serial.print("pos: ");
    Serial.println(pos);
    Serial.print("des: ");
    Serial.println(des);

    x = averageReading(38);


    if (3.3 / 1024 * x > 3.2)
        des = 2;
    else if (3.3 / 1024 * x < 3.1)
        des = 0;
    else
        des = (3.3 / 1024 * x - 3.1) * 20;

    delay(10);

    if (pos < -0.1 || pos>2.1) {
        while (1) {
            noInterrupts();
            analogWrite(PWMPin1, 0);
            analogWrite(PWMPin2, 0);
        }
    }

}


float averageReading(int pin) {
    float x = 0;
    for (int n = 0; n < 20; n++)
        x = x + analogRead(pin);
    x = x / 20;
    return x;



}