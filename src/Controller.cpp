#include "Controller.hpp"

Controller::Controller()
{
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
        p_dynamixel->writeControlTableItem(ControlTableItem::CONTROL_MODE, i, OperatingMode::OP_POSITION);
        p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, i, 885);
        //p_dynamixel->torqueOn(i);
    }
    
}

Controller::~Controller()
{
    delete p_dynamixel;
}

void Controller::_UpdateChain(){
    _UpdateArmThetas();
    _ForwardKinematics();
    _InverseKinematics();
    _ForwardDynamics();
    _InverseDynamics();
}

void Controller::SetGoalPosition(){
    int a = 10;
}

void Controller::_UpdateArmThetas(){
    theta1 = p_dynamixel->getPresentPosition(1, UNIT_DEGREE) * DEG_TO_RAD;
    theta2 = p_dynamixel->getPresentPosition(2, UNIT_DEGREE) * DEG_TO_RAD;
    theta3 = p_dynamixel->getPresentPosition(3, UNIT_DEGREE) * DEG_TO_RAD;
    
    /*
    theta1 = p_dynamixel->getPresentPosition(1, UNIT_DEGREE);
    theta2 = p_dynamixel->getPresentPosition(2, UNIT_DEGREE);
    theta3 = p_dynamixel->getPresentPosition(3, UNIT_DEGREE);

    */
}
void Controller::_UpdateFingerThetas(){
    theta4 = p_dynamixel->getPresentPosition(4, UNIT_DEGREE) * DEG_TO_RAD;
    theta5 = p_dynamixel->getPresentPosition(5, UNIT_DEGREE) * DEG_TO_RAD;

    theta4 = p_dynamixel->getPresentPosition(4, UNIT_DEGREE);
    theta5 = p_dynamixel->getPresentPosition(5, UNIT_DEGREE);
}

void Controller::debugPrint(){
    DEBUG_SERIAL.print("eeX: ");
    DEBUG_SERIAL.println(m_eePosition.x);
    DEBUG_SERIAL.print("eeY: ");
    DEBUG_SERIAL.println(m_eePosition.y);
    DEBUG_SERIAL.print("eeZ: ");
    DEBUG_SERIAL.println(m_eePosition.z);
    DEBUG_SERIAL.print("\n");
    _ForwardKinematics();
    }

void Controller::_ForwardKinematics(){
    _UpdateArmThetas();
    m_eePosition.x = -cos(theta1)*(l3*sin(theta2+theta3)+l2*sin(theta2));
    m_eePosition.y = -sin(theta1)*(l3*sin(theta2+theta3)+l2*sin(theta2));
    m_eePosition.z = l1+l3*cos(theta2+theta3)+l2*cos(theta2);
}
