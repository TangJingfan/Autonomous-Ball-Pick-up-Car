#ifndef MOTOR_H
#define MOTOR_H

class motor {
private:
  int pin_a, pin_b;

public:
  motor(int pin1, int pin2);

  ~motor();

  void begin();

  void set_cycle(int cycle);
};

#endif