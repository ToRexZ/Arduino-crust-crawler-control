#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

Controller* controller;

void setup(){
    controller = new Controller();
}

void loop(){
    controller->debugPrint();
    delay(200);
}
