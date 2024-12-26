#include "src/MecanumDriver.h"

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

void setup() {
  Serial.begin(115200);
  mecanum.begin();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == "w") {
      mecanum.setDutyCycle(100, 100, 100, 100);
    } else {
      mecanum.setDutyCycle(0, 0, 0, 0);
    }
  } else {
    mecanum.setDutyCycle(0, 0, 0, 0);
  }
  delay(100);
}
