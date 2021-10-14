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
    void PID_PWM(uint8_t id, int16_t desired_angle, double Kp = 0, double Ki = 0, double Kd = 0);

private:
    // Update functions
    void _UpdateChain();

    void _UpdateArmThetas();
    void _UpdateFingerThetas();

    // Kinematics
    void _ForwardKinematics();
    void _InverseKinematics();

    // Dynamics
    void _ForwardDynamics();
    void _InverseDynamics();

    // PID variables
    double proportional{0}, integral{0}, derivative{0}, last_PID_error{0}, PID_error{0}, PID_Value{0};

    // General private objects or variables
    Dynamixel2Arduino* p_dynamixel = NULL;

    const int l1{66}, l2{220}, l3{147};

    double theta1_raw, theta2_raw, theta3_raw, theta4_raw, theta5_raw;
    double theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg;
    eePosition m_eePosition;

public:
    void debugPrint();
    
};

#endif