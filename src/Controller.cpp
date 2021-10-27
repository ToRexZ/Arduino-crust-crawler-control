#include "Controller.hpp"

Controller::Controller()
{
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL)
    {
    }
    
    m_Joint1 = Joints(1, 10, 6.6, -180, 180, 10);
    m_Joint2 = Joints(2, 10, 22.0, 10, 10, 10);
    m_Joint3 = Joints(3, 10, 14.7, 10, 10, 10);
    m_Joint4 = Joints(4, 10, 10, 10, 10, 10);
    m_Joint5 = Joints(5, 10, 10, 10, 10, 10);

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

    m_currentTime = millis();
}

Controller::~Controller()
{
    delete p_dynamixel;
}

void Controller::_UpdateChain()
{
    
    _UpdateArmThetas();
    _ForwardKinematics();
    //_InverseKinematics();
    _ForwardDynamics();
    _InverseDynamics();
}

void Controller::_UpdateArmThetas(){    
    inputArmThetas.m_Theta1 = p_dynamixel->getPresentPosition(1, UNIT_RAW);
    inputArmThetas.m_Theta2 = p_dynamixel->getPresentPosition(2, UNIT_RAW);
    inputArmThetas.m_Theta3 = p_dynamixel->getPresentPosition(3, UNIT_RAW);
    inputArmThetas.currentUnitType = RAW;
}

void Controller::_UpdateFingerThetas(){   
    inputFingerThetas.m_Theta4 = p_dynamixel->getPresentPosition(4, UNIT_RAW);
    inputFingerThetas.m_Theta5 = p_dynamixel->getPresentPosition(5, UNIT_RAW);
    inputFingerThetas.currentUnitType = RAW;
}

void Controller::_ArmThetaConverter(UnitType unit){
    inputArmThetas = _ThetaConverter(unit, inputArmThetas);
}

void Controller::_FingerThetaConverter(UnitType unit){
    inputArmThetas = _ThetaConverter(unit, inputFingerThetas);
}

JointAngles Controller::_ThetaConverter(UnitType unit, JointAngles& r_JointAngles){
    if(unit == r_JointAngles.currentUnitType){
        return r_JointAngles;
    }

    JointAngles convertedThetas;
    double conversion;

    switch (unit)
    {
    case Degree:
    {   
        if(r_JointAngles.currentUnitType == RAW){
            conversion = 360/4095; 
            break;
        }
        if(r_JointAngles.currentUnitType == Radians){
            conversion = RAD_TO_DEG;
            break;
        }
    }

    case Radians:
    {
        if (r_JointAngles.currentUnitType == RAW)
        {
            conversion = 2*M_PI/4095;
            break;
        }
        if (r_JointAngles.currentUnitType == Degree)
        {
            conversion = DEG_TO_RAD;
            break;
        }
    }

    case RAW:
    {
        if (r_JointAngles.currentUnitType == Degree)
        {
            conversion = 4095/360;    
            break;
        }
        if (r_JointAngles.currentUnitType == Radians)
        {
            conversion = 4095/2*M_PI;
            break;
        }
    }
    
    default:
        DEBUG_SERIAL.print(F("Invalid conversion parameter"));
    }

    convertedThetas.m_Theta1 = r_JointAngles.m_Theta1*conversion;
    convertedThetas.m_Theta2 = r_JointAngles.m_Theta2*conversion;
    convertedThetas.m_Theta3 = r_JointAngles.m_Theta3*conversion;
    convertedThetas.m_Theta4 = r_JointAngles.m_Theta4*conversion;
    convertedThetas.m_Theta5 = r_JointAngles.m_Theta5*conversion;
    convertedThetas.currentUnitType = unit;

    return convertedThetas;
}

void Controller::_ForwardKinematics(){
    _ArmThetaConverter(Radians);
    m_eePosition.x = -cos(inputArmThetas.m_Theta1) * (m_Joint2.m_length * sin(inputArmThetas.m_Theta2) + m_Joint3.m_length * sin(inputArmThetas.m_Theta2 + inputArmThetas.m_Theta3));
    m_eePosition.y = -sin(inputArmThetas.m_Theta1) * (m_Joint2.m_length * sin(inputArmThetas.m_Theta2) + m_Joint3.m_length * sin(inputArmThetas.m_Theta2 + inputArmThetas.m_Theta3));
    m_eePosition.z = m_Joint1.m_length + m_Joint2.m_length * cos(inputArmThetas.m_Theta2) + m_Joint3.m_length * cos(inputArmThetas.m_Theta2 + inputArmThetas.m_Theta3);
}

void Controller::_InverseKinematics(eePosition eePos){
    _ArmThetaConverter(Radians);
    outputArmThetas.currentUnitType = inputArmThetas.currentUnitType;

    // THETA1
    // We calculate theta 1 and set it according to the joint constraints
    outputArmThetas.m_Theta1 = atan2(eePos.y, eePos.x);
    if(outputArmThetas.m_Theta1 > m_Joint1.m_maxTheta) { outputArmThetas.m_Theta1 = m_Joint1.m_maxTheta; }
    if(outputArmThetas.m_Theta1 < m_Joint1.m_minTheta) { outputArmThetas.m_Theta1 = m_Joint1.m_minTheta; }


    // THETA2
    // We calculate the endeffector position w.r.t frame 1 in the x-z plane
    double x1w = sqrt(pow(eePos.x,2)+pow(eePos.y,2));
    double z1w = eePos.z - m_Joint1.m_length;

    // 
    m_elbowDown = (z1w < 0 && x1w > 30) ? true : false;    

    // The we set sign used for the theta 2 and 3 calculation
    double sign = m_elbowDown ? -1 : 1;

    // Then calculate theta 2 according to the inverse kinematics formula for the robot
    outputArmThetas.m_Theta2 = atan2(z1w, x1w) + sign*acos(
            (pow(m_Joint2.m_length, 2)+pow(x1w, 2)+pow(z1w, 2)-pow(m_Joint3.m_length, 2)) /
            (2*m_Joint2.m_length*sqrt(pow(x1w, 2)+pow(z1w, 2)))
            ) - M_PI_2;
    
    // And then we set its contraints
    //if(outputArmThetas.m_Theta2 > m_Joint2.m_maxTheta) { outputArmThetas.m_Theta2 = m_Joint2.m_maxTheta; }
    //if(outputArmThetas.m_Theta2 < m_Joint2.m_minTheta) { outputArmThetas.m_Theta2 = m_Joint2.m_minTheta; }


    // THETA3
    // And then we calculate theta 3
    outputArmThetas.m_Theta3 = -sign*(M_PI - acos(
            (pow(m_Joint2.m_length, 2)+pow(m_Joint3.m_length, 2)-pow(x1w, 2)-pow(z1w, 2)) /
            (2*m_Joint2.m_length*m_Joint3.m_length)
            ));
    
    // And then we set its contraints
    //if(outputArmThetas.m_Theta3 > m_Joint3.m_maxTheta) { outputArmThetas.m_Theta3 = m_Joint3.m_maxTheta; }
    //if(outputArmThetas.m_Theta3 < m_Joint3.m_minTheta) { outputArmThetas.m_Theta3 = m_Joint3.m_minTheta; }
    

    DEBUG_SERIAL.print("x1w: ");
    DEBUG_SERIAL.println(x1w);
    DEBUG_SERIAL.print("z1w: ");
    DEBUG_SERIAL.println(z1w);
    DEBUG_SERIAL.print("Theta2 calculated (rad): ");
    DEBUG_SERIAL.println(outputArmThetas.m_Theta2);
    DEBUG_SERIAL.print("greater than: ");
    DEBUG_SERIAL.println(((tan(M_PI_2-abs(outputArmThetas.m_Theta2)))*x1w));
}

double Controller::_PID(int currentValue, int desiredValue){
    double Kp{1}, Ki{1}, Kd{1};

    double error = desiredValue - currentValue;

    m_proportional = Kp * error;
    m_derivative   = Kd * ( error - m_lastError );

    if(error < steadyStateError){
        m_integral     = Ki * ( m_integral + error );
    }
    m_lastError = error;

    return m_proportional + m_integral + m_derivative;
}

void Controller::PID_PWM(uint8_t id, int16_t desiredAngle){
    if (id >= 1 && id <= 3){ _ArmThetaConverter(RAW); }
    if (id > 3 && id <= 5) { _FingerThetaConverter(RAW); }

    double PIDsum;
    switch(id){
        case 1: PIDsum = _PID(inputArmThetas.m_Theta1, desiredAngle);
        case 2: PIDsum = _PID(inputArmThetas.m_Theta2, desiredAngle);
        case 3: PIDsum = _PID(inputArmThetas.m_Theta3, desiredAngle);
        case 4: PIDsum = _PID(inputFingerThetas.m_Theta4, desiredAngle);
        case 5: PIDsum = _PID(inputFingerThetas.m_Theta5, desiredAngle);

        default: PIDsum = 0;
    }

    int limit = p_dynamixel->readControlTableItem(ControlTableItem::PWM_LIMIT, id);

    double PID_Value = abs(PIDsum) > limit ? copysign(limit, PIDsum) : PIDsum;

    p_dynamixel->setGoalPWM(id, PID_Value, UNIT_RAW);
    
    if(PIDsum == 0){
        m_proportional = 0;
        m_integral = 0;   
        m_derivative = 0;
        m_lastError = 0;
    }
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
    DEBUG_SERIAL.print("\n");    
    
}

