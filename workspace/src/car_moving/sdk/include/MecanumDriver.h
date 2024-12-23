#ifndef MECANUM_DRIVER_H
#define MECANUM_DRIVER_H

#include "MotorDriver.h"

class MecanumDriver {
public:
  MecanumDriver(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6,
                int pin7, int pin8);

  ~MecanumDriver();

  void begin();

  void setDutyCycle(int duty_cycle1, int duty_cycle2, int duty_cycle3,
                    int duty_cycle4);

private:
  MotorDriver motor1;
  MotorDriver motor2;
  MotorDriver motor3;
  MotorDriver motor4;
};

#endif