#include "Controller.hpp"

Controller::Controller(bool &rads)
{
    rads = *rads;
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL)
    {
    }

    p_dynamixel = new Dynamixel2Arduino(DYNAMIXEL_SERIAL, DIRECTION_PIN);
    p_dynamixel->begin(57600);

    // Initial motor setup
    for (unsigned int i = 1; i < 6; i++)
    {
        p_dynamixel->torqueOff(i);
        p_dynamixel->writeControlTableItem(ControlTableItem::OPERATING_MODE, i, OperatingMode::OP_PWM);
        p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, i, 600);
        p_dynamixel->torqueOn(i);
    }
}

Controller::~Controller()
{
    delete p_dynamixel;
}

void Controller::_UpdateChain()
{
    _UpdateArmThetas();
    _ForwardKinematics();
    _InverseKinematics();
    _ForwardDynamics();
    _InverseDynamics();
}

void Controller::SetGoalPosition()
{
    int a = 10;
}

void Controller::_UpdateArmThetas(){
    theta1_raw = p_dynamixel->getPresentPosition(1, UNIT_RAW);
    theta2_raw = p_dynamixel->getPresentPosition(2, UNIT_RAW);
    theta3_raw = p_dynamixel->getPresentPosition(3, UNIT_RAW);
}
void Controller::_UpdateFingerThetas(){   
    theta4_raw = p_dynamixel->getPresentPosition(4, UNIT_RAW);
    theta5_raw = p_dynamixel->getPresentPosition(5, UNIT_RAW);
    
}

void Controller::debugPrint(){
    /*
    _ForwardKinematics();
    DEBUG_SERIAL.print("eeX: ");
    DEBUG_SERIAL.println(m_eePosition.x);
    DEBUG_SERIAL.print("eeY: ");
    DEBUG_SERIAL.println(m_eePosition.y);
    DEBUG_SERIAL.print("eeZ: ");
    DEBUG_SERIAL.println(m_eePosition.z);
    DEBUG_SERIAL.print("\n");
    */
    double presentPWM = p_dynamixel->readControlTableItem(ControlTableItem::PRESENT_PWM, 4);
    DEBUG_SERIAL.print("Present PWM: ");
    DEBUG_SERIAL.print(presentPWM);
    DEBUG_SERIAL.print("\n");


    _UpdateFingerThetas();
    PID_PWM(4, 180, 2, 0.01, 2.5);
    DEBUG_SERIAL.print("Error: ");
    DEBUG_SERIAL.print(last_PID_error);
    DEBUG_SERIAL.print("       PID: ");
    DEBUG_SERIAL.print(PID_Value);
    DEBUG_SERIAL.print("       Integral: ");
    DEBUG_SERIAL.println(integral);  
    DEBUG_SERIAL.print("\n");
    
    }

void Controller::_ForwardKinematics()
{
    _UpdateArmThetas();
<<<<<<< HEAD
    m_eePosition.x = -cos(theta1_deg)*(l3*sin(theta2_deg+theta3_deg)+l2*sin(theta2_deg));
    m_eePosition.y = -sin(theta1_deg)*(l3*sin(theta2_deg+theta3_deg)+l2*sin(theta2_deg));
    m_eePosition.z = l1+l3*cos(theta2_deg+theta3_deg)+l2*cos(theta2_deg);
=======
    m_eePosition.x = -cos(theta1) * (l3 * sin(theta2 + theta3) + l2 * sin(theta2));
    m_eePosition.y = -sin(theta1) * (l3 * sin(theta2 + theta3) + l2 * sin(theta2));
    m_eePosition.z = l1 + l3 * cos(theta2 + theta3) + l2 * cos(theta2);
>>>>>>> b1e871d84ba9c20168e7c3a29d355d045b9a4e60
}

void Controller::PID_PWM(uint8_t id, int16_t desired_angle, double Kp, double Ki, double Kd){
    PerformanceTester tester(&DEBUG_SERIAL, "PID function");
    switch (id)
    {
    case 1:
        PID_error = desired_angle - theta1;
        break;
    case 2:
        PID_error = desired_angle - theta2;
        break;
    case 3: 
        PID_error = desired_angle - theta3;
        break;
    case 4: 
        PID_error = desired_angle - theta4;
        break;
    case 5: 
        PID_error = desired_angle - theta5;
        break;
    default:
        DEBUG_SERIAL.print("Invalid ID call")
        break;
    }

    proportional = Kp * PID_error;
    integral = (integral + PID_error) * Ki;
    derivative = (PID_error - last_PID_error) * Kd;

    last_PID_error = PID_error;

    double sum = proportional + integral + derivative;
    int limit = p_dynamixel->readControlTableItem(ControlTableItem::PWM_LIMIT, id);

    PID_Value = abs(sum) > limit ? copysign(limit, sum) : sum;

    p_dynamixel->setGoalPWM(id, PID_Value, UNIT_RAW);
}