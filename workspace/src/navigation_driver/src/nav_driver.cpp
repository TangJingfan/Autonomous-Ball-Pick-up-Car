#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <iostream>

serial::Serial ser;

// 生成控制指令并发送给Arduino
void cmd_vel_call_back(const geometry_msgs::Twist::ConstPtr &msg) {
  double linear_x = msg->linear.x;   // 前进/后退速度
  double angular_z = msg->angular.z; // 旋转速度

  std::string generate_control_command(double linear_x, double angular_z) {
  std::string command = "<0,0,0,0>";  // 初始化指令为全零
  
  // 根据线速度生成前进/后退指令
  if (linear_x > 0) {
    // 前进
    command = "<" + std::to_string(static_cast<int>(linear_x * 100)) + ",0,0,0>";
  } else if (linear_x < 0) {
    // 后退
    command = "<0,0," + std::to_string(static_cast<int>(-linear_x * 100)) + ",0>";
  }

  // 根据角速度生成转向指令（顺时针/逆时针旋转）
  if (angular_z > 0) {
    // 顺时针旋转
    // 如果有前进指令，就同时加上旋转指令
    command = "<" + std::to_string(static_cast<int>(linear_x * 100)) + ",0," + 
              std::to_string(static_cast<int>(angular_z * 100)) + ",0>";
  } else if (angular_z < 0) {
    // 逆时针旋转
    // 如果有前进指令，就同时加上旋转指令
    command = "<" + std::to_string(static_cast<int>(linear_x * 100)) + ",0,0," + 
              std::to_string(static_cast<int>(-angular_z * 100)) + ">";
  }


  try {
    // 将指令发送给Arduino
    ser.write(command);
    std::cout << "Sent Command: " << command << std::endl;  // 打印发送的指令
    ser.flushOutput();  // 刷新输出缓冲区
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable to send data to Arduino.");
  }
}

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "nav_driver");
  ros::NodeHandle nh;

  try {
    // 初始化串口通信
    ser.setPort("/dev/arduino");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable to open port.");
    return -1;
  }

  if (ser.isOpen()) {
    ROS_INFO("Serial Port initialized.");
  } else {
    return -1;
  }

  // 订阅"/cmd_vel"话题
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmd_vel_call_back);

  // 让节点持续运行
  ros::spin();

  // 关闭串口
  ser.close();
  return 0;
}
