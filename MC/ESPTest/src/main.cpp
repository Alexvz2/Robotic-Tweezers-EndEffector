#include <Arduino.h>


#include <stdlib.h>
#include <string.h>


float v;
int tick = 0;
float pos = 0;
int PWMPin1 = 7;
int PWMPin2 = 8;
int hallA = 33;
int hallB = 34;
int inputPin = 38;
float duty, des, w, posOld;
bool forw;

int lastHall = 2;
float vel, error, volt;
float testCurrent = 0;




void setup() {

    Serial.begin(9600);

    setupPins();

    calibrateMotor();

    attachInterrupt(digitalPinToInterrupt(hallA), interruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallB), interruptB, CHANGE);
    delay(2000);


    /*
    Timer1.initialize(1000);                // Initializes as 1ms timer
    Timer1.attachInterrupt(PWMLoop);
    */

    setupTimer();


    interrupts();

    delay(1000);

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

void setupTimer() {



    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &PWMLoop, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

}

void IRAM_ATTR PWMLoop(void) {

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
}




void loop() {
    // put your main code here, to run repeatedly:
    float x;

    Serial.println(3.3 / 1024 * analogRead(inputPin));
    Serial.print("pos: ");
    Serial.println(pos);
    Serial.print("des: ");
    Serial.println(des);

    x = averageReading(inputPin) * 0.454543632 * 3.3 / 1024;
    if (x - des > 0.01) {
        des = des + 0.01;
    }
    else if (x - des < -0.01) {
        des = des - 0.01;
    }
    else {
        des = x;
    }


    des = x;        // des is 1.5 at 3.3v reading

    delay(10);

    if (pos < -0.1 || pos>2.1) {
        while (1) {
            noInterrupts();
            analogWrite(PWMPin1, 0);
            analogWrite(PWMPin2, 0);
        }
    }
}

void calibrateMotor() {
    analogWrite(PWMPin1, 0);
    analogWrite(PWMPin2, 1024 * 0.06);
    delay(200);
    analogWrite(PWMPin1, 1024 * 0.06);
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
    pinMode(35, OUTPUT);
    pinMode(hallA, INPUT_PULLUP);
    pinMode(hallB, INPUT_PULLUP);
    pinMode(inputPin, INPUT);
    digitalWrite(35, 1);

}

float averageReading(int pin) {
    float x = 0;
    for (int n = 0; n < 20; n++)
        x = x + analogRead(pin);
    x = x / 20;
    return x;
}