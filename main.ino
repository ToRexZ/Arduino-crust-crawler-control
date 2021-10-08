#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <DynamixelShield.h>

#include <math.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DXL_SERIAL Serial
#define DEBUG_SERIAL Serial3
const uint8_t DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);

struct vector3D
{
  int x;
  int y;
  int z;
};

vector3D endEffectorPosition;

const int l1 = 66; //Length in mm
const int l2 = 220;
const int l3 = 147;

void forwardKinematics(vector3D &eePosition)
{
  // Joint angles in radians
  // int theta1 = (dxl.getPresentPosition(1, UNIT_DEGREE) - 180) * DEG_TO_RAD;
  double theta2 = (dxl.getPresentPosition(2, UNIT_DEGREE) - 180) * DEG_TO_RAD;
  double theta3 = (dxl.getPresentPosition(3, UNIT_DEGREE) - 180) * DEG_TO_RAD;

  DEBUG_SERIAL.print("Theta2: ");
  DEBUG_SERIAL.println(theta2);
  DEBUG_SERIAL.print("Theta3: ");
  DEBUG_SERIAL.println(theta3);

  eePosition.x = l2 * cos(theta2 - M_PI_2) + l3 * cos(theta2 - M_PI_2 + theta3 - M_PI);
  eePosition.y = l2 * sin(theta2 - M_PI_2) + l3 * sin(theta2 - M_PI_2 + theta3 - M_PI);
}

class Controller
{
public:
private:
};

class Joint
{
public:
private:
};

double getJointAngle(uint8_t &&id, enum ParamUnit unit)
{
  return dxl.getPresentPosition(id, unit);
}

void setup()
{
  // DXL_SERIAL.begin(115200);
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
  {
  }
  dxl.begin(57600);

  dxl.torqueOff(1);
  dxl.writeControlTableItem(ControlTableItem::HOMING_OFFSET, 1, -1023);

  for (size_t i = 1; i < 6; i++)
  {
    dxl.torqueOff(i);
    // dxl.setOperatingMode(i, OperatingMode::OP_POSITION);
    dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, i, 1);
    dxl.writeControlTableItem(ControlTableItem::HOMING_OFFSET, i, 0);
    dxl.torqueOn(i);
  }
}

void loop()
{
  // forwardKinematics(endEffectorPosition);
  // DEBUG_SERIAL.print(endEffectorPosition.x);
  // DEBUG_SERIAL.print(" ; ");
  // DEBUG_SERIAL.println(endEffectorPosition.y);
  DEBUG_SERIAL.print("1: ");
  DEBUG_SERIAL.println(getJointAngle(1, UNIT_RAW));
  DEBUG_SERIAL.print("2: ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(2, UNIT_RAW));
  DEBUG_SERIAL.print("3: ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(3, UNIT_RAW));
  DEBUG_SERIAL.println();

  dxl.setGoalPosition(1, 0, UNIT_RAW);
  dxl.setGoalPosition(2, 0, UNIT_RAW);
  dxl.setGoalPosition(3, 0, UNIT_RAW);
  // DEBUG_SERIAL.println(dxl.getPresentPosition(4,UNIT_DEGREE));
  delay(100);
}
