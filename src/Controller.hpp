#ifndef CONTROLLER
#define CONTROLLER

// For dynamixel control
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

// For mathematics
#include <BasicLinearAlgebra.h>
#include <math.h>

// Custom headers
#include "DataStructures.hpp"
#include "PerformanceTester.hpp"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DYNAMIXEL_SERIAL Serial
#define DEBUG_SERIAL Serial3
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif


class Controller
{
public:
    Controller();
    ~Controller();

    void PID_PWM(uint8_t id, int16_t desired_angle);

private:
    // Update functions
    void _UpdateChain();

    void _UpdateArmThetas();
    void _UpdateFingerThetas();

    // Converts the current input thetas to the desired UnitType
    void _ArmThetaConverter(UnitType unit);
    void _FingerThetaConverter(UnitType unit);
    // Converts any JointAngles object to the desired UnitType
    JointAngles _ThetaConverter(UnitType unit, JointAngles& JointAngles);

    // Kinematics
    void _ForwardKinematics();
    void _InverseKinematics(eePosition eePos);

    // Dynamics
    void _ForwardDynamics();
    void _InverseDynamics();

    // PID variables
    double _PID(int id, int desiredAngle);
    double m_proportional{0}, m_integral{0}, m_derivative{0}, m_lastError{0}, steadyStateError{0};


    // General private objects or variables
    Dynamixel2Arduino* p_dynamixel = NULL;

    Joints m_Joint1, m_Joint2, m_Joint3, m_Joint4, m_Joint5;

    JointAngles inputArmThetas, outputArmThetas, inputFingerThetas, outputFingerThetas;
    eePosition m_eePosition;

    double m_currentTime;
    bool m_elbowDown{false};

public:
    void debugPrint();

};

#endif