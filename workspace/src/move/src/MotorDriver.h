#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

class MotorDriver {
public:
  MotorDriver(int pin1, int pin2);

  ~MotorDriver();

  void begin();

  void setDutyCycle(int duty_cycle);

private:
  int pin_a;
  int pin_b;
};

#endif