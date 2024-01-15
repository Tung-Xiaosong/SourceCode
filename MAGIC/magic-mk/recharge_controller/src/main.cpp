#include "recharge_controller/recharge_controller.h"

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "recharge_controller");

  // 创建RechargeController对象
  recharge_controller_ns::RechargeController recharge_controller;
//  recharge_controller.start();
  
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

//  ros::spin();

//  recharge_controller.stop();

  return 0;
}
