#ifndef DATASTRUCTURES
#define DATASTRUCTURES

struct Joints
{
    unsigned int m_id;
    unsigned int m_mass;
    unsigned int m_length;
    double m_minTheta, m_maxTheta;

    Joints(){};
    Joints(unsigned int&& id, unsigned int&& mass, unsigned int&& length, double&& minTheta, double&& maxTheta){
         m_id = id;
         m_mass = mass;
         m_length = length;
         m_minTheta = minTheta;
         m_maxTheta = maxTheta;
     }

};

struct eePosition
{
    int x{0}, y{0}, z{0};
};

struct JointAngles
{
    double m_Theta1, m_Theta2, m_Theta3, m_Theta4, m_Theta5;
};



#endif