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

#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_to_laserscan
{

  PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet() {}

  void PointCloudToLaserScanNodelet::onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
    private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

    private_nh_.param<double>("angle_min", angle_min_, - M_PI);
    private_nh_.param<double>("angle_max", angle_max_, M_PI);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
    private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
    private_nh_.param<double>("range_min", range_min_, 0.0);
    private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
    private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);

    //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
      nh_ = getNodeHandle();//使用getNodeHandle()函数创建一个非多线程的节点句柄（nh_）
    }
    else
    {
      nh_ = getMTNodeHandle();//使用getMTNodeHandle()函数创建一个多线程的节点句柄（nh_）
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
      input_queue_size_ = concurrency_level;//输入队列的大小与并发级别相同
    }
    else
    {
      input_queue_size_ = boost::thread::hardware_concurrency();//使用boost::thread::hardware_concurrency()函数获取系统的硬件并发级别，并将其赋值给input_queue_size_
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
      message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
      message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
    }
    else // otherwise setup direct subscription
    {
      sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    }

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
                                                 boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),//连接回调函数boost::bind(&PointCloudToLaserScanNodelet::connectCb, this)在有订阅者连接到发布器时被调用。
                                                                                                             //这意味着当有节点订阅了"scan"话题时，会触发connectCb函数。
                                                 boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));//断开连接回调函数boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this)在所有订阅者断开连接时被调用。
                                                                                                                 //当没有节点订阅"scan"话题时，会触发disconnectCb函数。
                                                                                                                 /*
                                                                                                                 这两个回调函数的目的是在订阅者连接或断开时执行相应的操作。
                                                                                                                 例如，在有订阅者连接时，可以开始订阅输入的点云数据；在所有订阅者断开时，可以停止订阅输入的点云数据。
                                                                                                                 这样可以动态地根据订阅者的状态来控制数据流。
                                                                                                                 */
  }

  void PointCloudToLaserScanNodelet::connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)//pub_.getNumSubscribers()用于获取pub_发布器当前的订阅者数量。sub_.getSubscriber().getNumPublishers()用于获取sub_订阅器当前订阅的发布者数量。
    {                                                                                //如果pub_的订阅者数量大于0且sub_的订阅者数量为0，说明已经有节点订阅了"scan"话题，但是sub_订阅器没有订阅任何发布者。
      NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");//表示有节点开始订阅激光扫描数据，并且准备订阅输入的点云数据。
      sub_.subscribe(nh_, "cloud_in", input_queue_size_);//调用sub_.subscribe来订阅名为"cloud_in"的点云数据消息。这样可以根据订阅者的状态动态地控制数据的流动。
    }
  }

  void PointCloudToLaserScanNodelet::disconnectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
      sub_.unsubscribe();
    }
  }

  void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                               tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
        << message_filter_->getTargetFramesString() << " at time " << cloud_msg->header.stamp << ", reason: " << reason);
  }

  void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    //build laserscan output
    sensor_msgs::LaserScan output;
    output.header = cloud_msg->header;
    if (!target_frame_.empty())
    {
      output.header.frame_id = target_frame_;
    }

    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);//使用了 std::ceil() 函数来向上取整。

    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range确定没有障碍物数据的激光扫描射线是否将计算为无穷大或 max_range
    if (use_inf_)//希望将没有障碍物数据的射线视为无穷大
    {
      output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());//将 output.ranges 数组填充为长度为 ranges_size 的数组，每个元素都设置为无穷大。
                                                                                 //assign()用于将指定的值分配给 std::vector 容器的元素
    }
    else//将没有障碍物数据的射线视为最大射程
    {
      output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);//将 output.ranges 数组填充为长度为 ranges_size 的数组，每个元素都设置为 output.range_max + inf_epsilon_。
                                                                         //这里的 output.range_max 表示激光扫描的最大射程，inf_epsilon_ 是一个非负的小增量，用于确保没有障碍物数据的射线的值略微超过最大射程。
    }

    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;

    // Transform cloud if necessary
    if (!(output.header.frame_id == cloud_msg->header.frame_id))
    {
      try
      {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
        cloud_out = cloud;
      }
      catch (tf2::TransformException &ex)
      {
        NODELET_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
      cloud_out = cloud_msg;
    }

    // Iterate through pointcloud遍历点云
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");//sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x");创建了三个迭代器对象 iter_x、iter_y、iter_z
              iter_x != iter_x.end();//使用 iter_x 进行循环迭代，直到迭代器 iter_x 到达尾部位置 iter_x.end()（即遍历完所有的点云数据）为止
              ++iter_x, ++iter_y, ++iter_z)//在每次循环迭代时，通过 ++iter_x、++iter_y、++iter_z 分别将迭代器指向下一个点云数据的 x、y、z 坐标。
    {

      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))//用于判断一个浮点数是否为 NaN。它接受一个浮点数作为参数，并返回一个布尔值，指示该浮点数是否为 NaN。NaN 值通常表示无效或缺失的数据。
      {
        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
        continue;//使用 continue 跳过当前的迭代，继续下一个迭代。
      }

      if (*iter_z > max_height_ || *iter_z < min_height_)//点云高度大于max_height_，或小于min_height_
      {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);//用于计算两个数的平方和的平方根。即计算出的距离。
      if (range < range_min_)//点云水平面距离小于range_min_
      {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }
      if (range > range_max_)
      {
        NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);//计算给定的 y 和 x 坐标值的反正切值,即计算出的角度

      if(cloud_msg->header.frame_id == "laser_link")
      {
        if (/*angle < output.angle_min || angle > output.angle_max*/ angle > -0.52 && angle < 0.52)
        {
          NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
          continue;
        }
      }
      else
      {
        if (angle < output.angle_min || angle > output.angle_max)
        {
          NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
          continue;
        }
      }
//---------
      if (/*angle < output.angle_min || angle > output.angle_max*/ angle > -0.52 && angle < 0.52 && cloud_msg->header.frame_id == "laser_link")
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
      }

      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;//计算出 angle 对应的激光扫描数据中的索引位置。
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;//根据新的距离值更新 output 对象中相应射线的距离值，以确保存储的距离值是最小的。
      }

    }
    pub_.publish(output);
  }

}

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)

/*
迭代器（Iterator）是一种用于遍历容器（如数组、列表、集合等）元素的对象。
它提供了一种统一的访问容器元素的方式，使得我们可以按顺序访问容器中的每个元素，而无需了解容器的内部实现细节。

迭代器通常提供以下操作：

begin()：返回指向容器第一个元素的迭代器。
end()：返回指向容器尾部的迭代器，它不指向有效元素。通常用于表示遍历结束的位置。
operator++（前置或后置递增）：将迭代器前进到下一个元素。
operator*：返回当前迭代器指向的元素的引用或值。
operator!= 或 operator==：用于比较两个迭代器是否相等。
*/