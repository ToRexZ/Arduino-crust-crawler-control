//#include <DynamixelShield.h>
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

Controller* controller;
//Dynamixel2Arduino* p_dynamixel;

void setup(){
    controller = new Controller();

    /*
    p_dynamixel = new Dynamixel2Arduino(Serial, 2);
    p_dynamixel->begin(57600);
    
    p_dynamixel->torqueOff(4);

    p_dynamixel->writeControlTableItem(ControlTableItem::OPERATING_MODE, 4, OperatingMode::OP_PWM);
    p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, 4, 200);

    p_dynamixel->torqueOn(4);
    */
}

void loop(){
    controller->debugPrint();
    //delay(300);
}
