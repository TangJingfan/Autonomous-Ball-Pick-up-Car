#include "workspace/src/car_moving/sdk/include/MecanumDriver.h"

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

void setup() {
  serial.begin(115200);
  mecanum.begin();
}

void loop() {

}
