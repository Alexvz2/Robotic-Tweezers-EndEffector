#include <Arduino.h>
#include "ESP32_S2_TimerInterrupt.h"

#include <stdlib.h>
#include <string.h>


#define hallA2 9
#define hallB2 8

#define clientPWM1 7
#define clientPWM2 6




int test = 0;



float des;





int sense12, sense22, senseOld12, senseOld22;

ESP32Timer ITimer0(0);
void setupPins();
void calibrateMotor();
void interruptA();
void interruptB();









void setup() {

    Serial.begin(9600);

    setupPins();

    calibrateMotor();



    attachInterrupt(digitalPinToInterrupt(hallA2), hallSense2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hallB2), hallSense2, CHANGE);
    delay(1000);
    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 1024 * 0.04);
    delay(350);
    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 0);

    delay(2000);









    interrupts();

    delay(1000);

    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 1024 * 0.04);
}




void hallSense2() {


    noInterrupts();

    sense12 = digitalRead(hallA2);
    sense22 = digitalRead(hallB2);

    if (senseOld12 && senseOld22) {
        if (sense12 && !sense22)
            des = des + 1 / 65.25 / 2 * (1.3 / 0.4);
        if (!sense12 && sense22)
            des = des - 1 / 65.25 / 2 * (1.3 / 0.4);
    }
    else if (senseOld12 && !senseOld22) {
        if (!sense12 && !sense22)
            des = des + 1 / 65.25 / 2 * (1.3 / 0.4);
        if (sense12 && sense22)
            des = des - 1 / 65.25 / 2 * (1.3 / 0.4);
    }
    else if (!senseOld12 && !senseOld22) {
        if (!sense12 && sense22)
            des = des + 1 / 65.25 / 2 * (1.3 / 0.4);
        if (sense12 && !sense22)
            des = des - 1 / 65.25 / 2 * (1.3 / 0.4);
    }
    else if (!senseOld12 && senseOld22) {
        if (sense12 && sense22)
            des = des + 1 / 65.25 / 2 * (1.3 / 0.4);
        if (!sense12 && !sense22)
            des = des - 1 / 65.25 / 2 * (1.3 / 0.4);
    }



    senseOld12 = sense12;
    senseOld22 = sense22;

    interrupts();

}






void loop() {
    // put your main code here, to run repeatedly:




    Serial.print("des: ");
    Serial.println(des);

    if (des < 0) {
        analogWrite(clientPWM1, 0);
        analogWrite(clientPWM2, 0);
    }
    else {
        analogWrite(clientPWM1, 0);
        analogWrite(clientPWM2, 1024 * 0.04);
    }


    delay(1);




}

void calibrateMotor() {

    analogWrite(clientPWM1, 1024 * 0.04);
    analogWrite(clientPWM2, 0);
    delay(200);

    delay(1000);

    analogWrite(clientPWM1, 0);
    analogWrite(clientPWM2, 0);
    delay(1000);



    des = 1.3;

    Serial.println("Calibration Finished");
}



void setupPins() {
    // Setting PWM pins as outputs

    pinMode(clientPWM1, OUTPUT);
    pinMode(clientPWM2, OUTPUT);

    pinMode(hallA2, INPUT_PULLUP);
    pinMode(hallB2, INPUT_PULLUP);


}

