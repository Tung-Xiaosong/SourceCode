/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "pointcloud_to_laserscan_node");
  ros::NodeHandle private_nh("~");//当节点创建一个ros::NodeHandle对象时，可以选择传递一个命名空间作为参数。
                                  //在这种情况下，~符号表示该节点的私有命名空间。
                                  //使用私有命名空间可以使节点的参数、话题和服务与其他节点保持独立，避免冲突，并提供更好的代码封装性。
  int concurrency_level;
  private_nh.param<int>("concurrency_level", concurrency_level, 0);

  nodelet::Loader nodelet;//创建一个nodelet::Loader对象，命名为nodelet
  nodelet::M_string remap(ros::names::getRemappings());//使用ros::names::getRemappings()获取当前ROS节点的重映射（remapping）信息，并将其存储在nodelet::M_string类型的remap变量中。
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet", remap, nargv);//调用nodelet.load()函数来加载Nodelet。

  boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
  if(concurrency_level) {
    spinner.reset(new ros::MultiThreadedSpinner(concurrency_level));
  }else{
    spinner.reset(new ros::MultiThreadedSpinner());
  }
  spinner->spin();//调用spinner->spin()函数开始自旋（spin）。自旋器会循环执行ROS回调函数，以处理接收到的消息和事件，直到节点被关闭或spinner->stop()被调用。
  return 0;
  /*创建了一个多线程的自旋器，并开始执行自旋，以处理ROS回调函数。
  自旋器的并发级别由concurrency_level决定，如果未指定并发级别，则使用默认值。
  自旋器的作用是确保ROS回调函数可以及时地被调用和处理，以保持节点的响应性。*/
}
