#include "motor.h"
#include <Arduino.h>

motor::motor(int pin1, int pin2) : pin_a(pin1), pin_b(pin2) {}

motor::~motor() {}

void motor::begin() {
  pinMode(pin_a, OUTPUT);
  pinMode(pin_b, OUTPUT);
  set_Cycle(0);
}

void motor::set_Cycle(int cycle) {
  if (cycle >= 0) {
    analogWrite(pin_a, constrain(abs(cycle), 0, 255));
    analogWrite(pin_b, 0);
  } else {
    analogWrite(pin_a, 0);
    analogWrite(pin_b, constrain(abs(cycle), 0, 255));
  }
}