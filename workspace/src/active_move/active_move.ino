#include "src/MecanumDriver.h"

// define pin for mecanum car
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

// define an array to record speed for 4 wheels
int speed[4] = {0, 0, 0, 0};

// define a string to receive command from serial
String command;

void setup() {
  // init mecanum car
  mecanum.begin();

  // start serial; set baudrate at 115200
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {

    // read command from serial
    char temp = char(Serial.read());
    command += temp;
  }

  if (command.length() > 0) {
    int begin = command.indexOf('<');
    int end = command.indexOf('>') + 1;
    if (begin != -1 && end != -1 && begin < end) {

      // assign value to speed array
      sscanf(command.substring(begin, end).c_str(), "<%d,%d,%d,%d>", &speed[0],
             &speed[1], &speed[2], &speed[3]);
    }
  }

  Serial.println(command);

  command = "";

  // make mecanum car run
  mecanum.setDutyCycle(speed[0], speed[1], speed[2], speed[3]);
  delay(200);
}
