#ifndef CONTROLLER
#define CONTROLLER

// For dynamixel control
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

// For mathematics
#include <BasicLinearAlgebra.h>
#include <math.h>


#include <ElementStorage.h>

// Custom headers
#include "DataStructures.hpp"
#include "PerformanceTester.hpp"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DYNAMIXEL_SERIAL Serial
#define DEBUG_SERIAL Serial3
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

enum UnitType{
        Degree,
        Radians
    };

class Controller
{
public:
    Controller(const bool rads);
    ~Controller();

    void SetGoalPosition();
    void PID_PWM(uint8_t id, int16_t desired_angle, double Kp = 0, double Ki = 0, double Kd = 0);

private:
    // Update functions
    void _UpdateChain();

    void _UpdateArmThetas();
    void _UpdateFingerThetas();

    // Kinematics
    JointAngles _ThetaConverter(UnitType unit);
    void _ForwardKinematics();
    void _InverseKinematics();

    // Dynamics
    void _ForwardDynamics();
    void _InverseDynamics();

    // PID variables
    double proportional{0}, integral{0}, derivative{0}, last_PID_error{0}, PID_error{0}, PID_Value{0};

    // General private objects or variables
    Dynamixel2Arduino* p_dynamixel = NULL;

    Joints m_Joints[5];

    const int l1{66}, l2{220}, l3{147};

    double theta1, theta2, theta3, theta4, theta5;
    eePosition m_eePosition;

    bool m_rads;

public:
    void debugPrint();

};

#endif