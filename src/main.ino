//#include <DynamixelShield.h>
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

Controller *controller;

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

struct Joint
{
  int id;
  int length;
  int mass;
  //Mass matrix.
  //interia tensor matrix.
};

vector3D endEffectorPosition;

//Calculate with radians[true] or degrees[false];
const bool rads = false;

const int l1 = 66; //Length in mm
const int l2 = 220;
const int l3 = 147;

double proportional = 0;
double integral = 0;
double derivative = 0;
double last_error;
double temp_PID;

double PID_PWM(uint8_t id, int16_t desired_raw_angle, double Kp = 0, double Ki = 0, double Kd = 0)
{
  double error = desired_raw_angle - dxl.getPresentPosition(id, UNIT_RAW);

  proportional = Kp * error;
  integral = (integral + error) * Ki;
  derivative = (error - last_error) * Kd;

  last_error = error;

  double sum = proportional + integral + derivative;
  int limit = dxl.readControlTableItem(ControlTableItem::PWM_LIMIT, id);

  return abs(sum) > limit ? copysign(limit, sum) : sum;
}

void setup()
{
  controller = new Controller(&rads);
}

void loop()
{
  controller->debugPrint();
  delay(30);

  temp_PID = PID_PWM(4, 4096 / 2, 2, 0.9, 2.5);
  dxl.setGoalPWM(4, temp_PID, UNIT_RAW);
  DEBUG_SERIAL.print("Error: ");
  DEBUG_SERIAL.print(last_error);
  DEBUG_SERIAL.print("       PID: ");
  DEBUG_SERIAL.print(temp_PID);
  DEBUG_SERIAL.print("       Integral: ");
  DEBUG_SERIAL.println(integral);
}
