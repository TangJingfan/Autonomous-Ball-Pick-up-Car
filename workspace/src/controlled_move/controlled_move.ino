#include "src/MecanumDriver.h"

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

String command;
String lastCommand = "x"; // init chassis; setting as 'stop'

void setup() { 
    mecanum.begin();
    Serial.begin(115200);
}

void loop() {
    while (Serial.available() > 0) {
        char temp = char(Serial.read());
        if (temp != '\n' && temp != '\r') { 
            command = temp; // keep latest useful command
        }
    }

    if (command != lastCommand) { // execute last command if new command is entered
        if (command == "w") {
            mecanum.setDutyCycle(100, 100, 100, 100); // forward
        } else if (command == "a") {
            mecanum.setDutyCycle(-100, 100, -100, 100); // turn left
        } else if (command == "s") {
            mecanum.setDutyCycle(-100, -100, -100, -100); // backward
        } else if (command == "d") {
            mecanum.setDutyCycle(100, -100, 100, -100); // turn right
        } else if (command == "x") {
            mecanum.setDutyCycle(0, 0, 0, 0); // stop
        }

        lastCommand = command; // update command
    }
}
