#include <ros.h>
#include <geometry_msgs/Twist.h>

// 包含自定义的 MecanumDriver 类
#include "workspace/src/car_moving/sdk/include/MecanumDriver.h"

// 定义 MecanumDriver 对象
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

// 全局变量
const float wheel_base = 0.3; // 车轮间距
const int max_pwm = 255;      // 最大 PWM 值
int left_speed = 0, right_speed = 0;

// ROS 节点和消息
ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel_msg;

// 回调函数：处理接收到的 /cmd_vel 消息
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear = msg.linear.x;  // 提取线速度
  float angular = msg.angular.z; // 提取角速度

  // 计算电机速度
  float left_vel = linear - angular * (wheel_base / 2.0);
  float right_vel = linear + angular * (wheel_base / 2.0);

  // 映射到 PWM 范围
  left_speed = constrain(map(left_vel * 1000, -1000, 1000, -max_pwm, max_pwm), -max_pwm, max_pwm);
  right_speed = constrain(map(right_vel * 1000, -1000, 1000, -max_pwm, max_pwm));

  // 更新电机速度
  mecanum.setDutyCycle(left_speed, left_speed, right_speed, right_speed);
}

// 订阅 /cmd_vel 话题
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", cmdVelCallback);

void setup() {
  // 初始化 ROS 节点
  nh.initNode();
  nh.subscribe(cmd_vel_sub);

  // 初始化电机
  mecanum.begin();
}

void loop() {
  // 处理 ROS 消息
  nh.spinOnce();
  delay(10); // 避免过度占用资源
}
