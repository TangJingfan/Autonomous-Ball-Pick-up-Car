#include "MecanumDriver.h"
#include "Arduino.h"


MecanumDriver::MecanumDriver(int pin1, int pin2, int pin3, int pin4, int pin5,
                             int pin6, int pin7, int pin8)
    : motor1(pin1, pin2), motor2(pin4, pin3), motor3(pin5, pin6),
      motor4(pin8, pin7) {}

MecanumDriver::~MecanumDriver() {}

void MecanumDriver::begin() {
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  setDutyCycle(0, 0, 0, 0);
}

void MecanumDriver::setDutyCycle(int duty_cycle1, int duty_cycle2,
                                 int duty_cycle3, int duty_cycle4) {
  motor1.setDutyCycle(duty_cycle1);
  motor2.setDutyCycle(duty_cycle2);
  motor3.setDutyCycle(duty_cycle3);
  motor4.setDutyCycle(duty_cycle4);
}