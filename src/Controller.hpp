#ifndef CONTROLLER
#define CONTROLLER

#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#include "Joints.hpp"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
#define DXL_SERIAL Serial
#define DEBUG_SERIAL Serial3
const uint8_t DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

class Controller
{
public:
    Controller();
    ~Controller();

private:
    

};

#endif