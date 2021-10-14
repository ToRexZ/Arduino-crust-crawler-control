#include "Controller.hpp"

Controller::Controller(bool rads)
{
    m_rads = rads;
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL)
    {
    }
    
    m_Joints[1] = Joints(1, 10, 66, 10, 10);
    m_Joints[2] = Joints(2, 10, 220, 10, 10);
    m_Joints[3] = Joints(3, 10, 147, 10, 10);
    m_Joints[4] = Joints(4, 10, 10, 10, 10);
    m_Joints[5] = Joints(5, 10, 10, 10, 10);

    p_dynamixel = new Dynamixel2Arduino(DYNAMIXEL_SERIAL, DIRECTION_PIN);
    p_dynamixel->begin(57600);

    // Initial motor setup
    for (unsigned int i = 1; i < 6; i++)
    {
        p_dynamixel->torqueOff(i);
        p_dynamixel->writeControlTableItem(ControlTableItem::OPERATING_MODE, i, OperatingMode::OP_PWM);
        p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, i, 600);
        //p_dynamixel->torqueOn(i);
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
    theta1 = p_dynamixel->getPresentPosition(1, UNIT_RAW);
    theta2 = p_dynamixel->getPresentPosition(2, UNIT_RAW);
    theta3 = p_dynamixel->getPresentPosition(3, UNIT_RAW);
}
void Controller::_UpdateFingerThetas(){   
    theta4 = p_dynamixel->getPresentPosition(4, UNIT_RAW);
    theta5 = p_dynamixel->getPresentPosition(5, UNIT_RAW);
    
}

void Controller::debugPrint(){
    _UpdateArmThetas();
    _ForwardKinematics();
    DEBUG_SERIAL.print("eeX: ");
    DEBUG_SERIAL.println(m_eePosition.x);
    DEBUG_SERIAL.print("eeY: ");
    DEBUG_SERIAL.println(m_eePosition.y);
    DEBUG_SERIAL.print("eeZ: ");
    DEBUG_SERIAL.println(m_eePosition.z);

    DEBUG_SERIAL.print("Length 1: ");
    DEBUG_SERIAL.println(m_Joints[1].m_length);
    DEBUG_SERIAL.print("Length 2: ");
    DEBUG_SERIAL.println(m_Joints[2].m_length);
    DEBUG_SERIAL.print("Length 3: ");
    DEBUG_SERIAL.println(m_Joints[3].m_length);
    DEBUG_SERIAL.print("\n");
    
    /*
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
    */
    
}

JointAngles Controller::_ThetaConverter(UnitType unit){
    JointAngles thetas;
    switch (unit)
    {
    case Degree:
    {    
        double conversion = 4095/360;
        thetas.m_Theta1 = theta1/conversion;
        thetas.m_Theta2 = theta2/conversion;
        thetas.m_Theta3 = theta3/conversion;
        thetas.m_Theta4 = theta4/conversion;
        thetas.m_Theta5 = theta5/conversion;
        break;
    }
    case Radians:
    {
        double conversion = 4095/360*M_PI/180;
        thetas.m_Theta1 = theta1/conversion;
        thetas.m_Theta2 = theta2/conversion;
        thetas.m_Theta3 = theta3/conversion;
        thetas.m_Theta4 = theta4/conversion;
        thetas.m_Theta5 = theta5/conversion;
        break;
    }
    
    default:
        DEBUG_SERIAL.print("Invalid conversion parameter");
        break;
    }
    return thetas;
}

void Controller::_ForwardKinematics()
{
    JointAngles thetas = _ThetaConverter(Degree);
    m_eePosition.x = -cos(thetas.m_Theta1) * (m_Joints[3].m_length * sin(thetas.m_Theta2 + thetas.m_Theta3) + m_Joints[2].m_length * sin(thetas.m_Theta2));
    m_eePosition.y = -sin(thetas.m_Theta1) * (m_Joints[3].m_length * sin(thetas.m_Theta2 + thetas.m_Theta3) + m_Joints[2].m_length * sin(thetas.m_Theta2));
    m_eePosition.z = m_Joints[2].m_length + m_Joints[3].m_length * cos(thetas.m_Theta2 + thetas.m_Theta3) + m_Joints[2].m_length * cos(thetas.m_Theta2);

    /*
    m_eePosition.x = -cos(theta1) * (l3 * sin(theta2 + theta3) + l2 * sin(theta2));
    m_eePosition.y = -sin(theta1) * (l3 * sin(theta2 + theta3) + l2 * sin(theta2));
    m_eePosition.z = l1 + l3 * cos(theta2 + theta3) + l2 * cos(theta2);
    */
}

void Controller::PID_PWM(uint8_t id, int16_t desired_angle, double Kp, double Ki, double Kd){
    PerformanceTester tester(&DEBUG_SERIAL, "PID function");
    switch (id)
    {
        case 1:
        {
            PID_error = desired_angle - theta1;
            break;
        }
        case 2:
        {
            PID_error = desired_angle - theta2;
            break;
        }
        case 3:
        {
            PID_error = desired_angle - theta3;
            break;
        }
        case 4: 
        {
            PID_error = desired_angle - theta4;
            break;
        }
        case 5: 
        {
            PID_error = desired_angle - theta5;
            break;
        }
        default:
        {
            DEBUG_SERIAL.print("Invalid ID call");
            break;
        }
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