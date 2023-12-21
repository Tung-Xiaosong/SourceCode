#include "recharge_controller/recharge_controller.h"

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "recharge_controller");

  // 创建RechargeController对象
  recharge_controller_ns::RechargeController recharge_controller;
//  recharge_controller.start();
  
  ros::MultiThreadedSpinner spinner(2); //ros/ros.h
                                        //ros::MultiThreadedSpinner是一种用于处理多个ROS节点（Node）的并发执行的方式。
                                        //可以在不同的线程中并发地执行ROS节点。这在处理多个传感器、执行多个任务或并发控制多个机器人组件时非常有用。
                                        //使用2个线程来执行ROS节点
  spinner.spin();                       //用于运行ros::MultiThreadedSpinner对象的方法，它使ROS节点执行器开始运行

//  ros::spin();

//  recharge_controller.stop();

  return 0;
}
