//#include <DynamixelShield.h>
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#define RADS true

Controller* controller;


void setup(){
    controller = new Controller(RADS);
}

void loop(){
    controller->debugPrint();
    //delay(300);
}
