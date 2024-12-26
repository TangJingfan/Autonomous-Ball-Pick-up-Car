#include "src/MecanumDriver.h"

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");
  mecanum.begin();
  Serial.println("Mecanum initialized");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString();  // 读取完整输入字符串
    command.trim();  // 去掉多余的空格和换行符
    Serial.print("Received command: ");
    Serial.println(command);

    if (command == "w") {
      mecanum.setDutyCycle(100, 100, 100, 100);
      Serial.println("Motors running forward");
    } else {
      mecanum.setDutyCycle(0, 0, 0, 0);
      Serial.println("Motors stopped");
    }
  } else {
    mecanum.setDutyCycle(0, 0, 0, 0);
  }
  delay(100);
}

