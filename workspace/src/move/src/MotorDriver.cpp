#include "MotorDriver.h"
#include <Arduino.h>


MotorDriver::MotorDriver(int pin1, int pin2) : pin_a(pin1), pin_b(pin2) {}

MotorDriver::~MotorDriver() {}

void MotorDriver::begin() {
  pinMode(pin_a, OUTPUT);
  pinMode(pin_b, OUTPUT);
  setDutyCycle(0);
}

void MotorDriver::setDutyCycle(int duty_cycle) {
  if (duty_cycle >= 0) {
    analogWrite(pin_a, constrain(abs(duty_cycle), 0, 255));
    analogWrite(pin_b, 0);
  } else {
    analogWrite(pin_a, 0);
    analogWrite(pin_b, constrain(abs(duty_cycle), 0, 255));
  }
}