#ifndef CONTROLLER
#define CONTROLLER

#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

#include "Joints.hpp"

#include "PerformanceTester.hpp"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
#define DYNAMIXEL_SERIAL Serial
#define DEBUG_SERIAL Serial3
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

struct eePosition
{
    int x{0}, y{0}, z{0};
};

class Controller
{
public:
    Controller();
    ~Controller();

    void SetGoalPosition();

private:
    void _UpdateChain();

    void _UpdateArmThetas();
    void _UpdateFingerThetas();

    void _InverseKinematics();
    void _ForwardDynamics();
    void _InverseDynamics();

    Dynamixel2Arduino *p_dynamixel = NULL;

    const int l1{66}, l2{220}, l3{147};
    double theta1, theta2, theta3, theta4, theta5;
    const bool rads;

    eePosition m_eePosition;

public:
    void _ForwardKinematics();
    void debugPrint();
};

#endif