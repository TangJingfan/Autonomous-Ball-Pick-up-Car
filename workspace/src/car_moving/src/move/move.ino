#include "workspace/src/car_moving/sdk/include/MecanumDriver.h"
#include "workspace/src/car_moving/sdk/include/MotorDriver.h"
#include <Servo.h> 

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);


void setup() {
  Serial.begin(115200); // 初始化串口通信
  mecanum.begin();
}

void loop() {
  // 检查是否有串口数据可读
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); // 按行读取指令
    parseCmd(cmd); // 解析并处理指令
  }

  // 控制电机输出
  mecanum.setDutyCycle(left_speed,left_speed,right_speed,right_speed);
}

void parseCmd(String cmd) {
  float linear = 0.0;
  float angular = 0.0;

  // 检查格式并解析: 期望格式 "linear:0.2 angular:0.5"
  if (cmd.startsWith("linear:")) {
    int lin_index = cmd.indexOf("linear:") + 7;
    int ang_index = cmd.indexOf(" angular:");
    linear = cmd.substring(lin_index, ang_index).toFloat();
    angular = cmd.substring(ang_index + 9).toFloat();

    // 将线速度和角速度转为左右电机速度
    calculateMotorSpeed(linear, angular);
  }
}

void calculateMotorSpeed(float linear, float angular) {
  // 使用差速转向模型计算左右轮速度
  float left_vel = linear - angular * (wheel_base / 2.0);
  float right_vel = linear + angular * (wheel_base / 2.0);

  // 将速度映射到 PWM 范围
  left_speed = constrain(map(left_vel * 1000, -1000, 1000, -max_pwm, max_pwm), 0, max_pwm);
  right_speed = constrain(map(right_vel * 1000, -1000, 1000, -max_pwm, max_pwm), 0, max_pwm);
}
