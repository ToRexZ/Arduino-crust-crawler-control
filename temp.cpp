//#include <DynamixelShield.h>
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#include <math.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
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

const int l1 = 0;
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

void setup()
{
  // DXL_SERIAL.begin(115200);
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
  {
  }
  dxl.begin(57600);

  for (size_t i = 1; i < 6; i++)
  {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OperatingMode::OP_POSITION);
    dxl.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, i, 1);
    dxl.writeControlTableItem(ControlTableItem::HOMING_OFFSET, i, 0);
    dxl.torqueOn(i);
  }
  dxl.torqueOff(2);
  dxl.torqueOff(3);
}

void loop()
{
  forwardKinematics(endEffectorPosition);
  DEBUG_SERIAL.print(endEffectorPosition.x);
  DEBUG_SERIAL.print(" ; ");
  DEBUG_SERIAL.println(endEffectorPosition.y);
}
