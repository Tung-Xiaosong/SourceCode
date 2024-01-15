#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <mutex>
#include <cerrno>
#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

#define CAR_TYPE  4   //1: DGt  2: FW  3: FR  4:MK-01

#if CAR_TYPE == 1
  #include "yhs_msgs/DgtCtrlCmd.h"
  #include "yhs_msgs/DgtIoCmd.h"
  #include "yhs_msgs/DgtChassisInfoFb.h"
#elif CAR_TYPE == 2
  #include "yhs_msgs/FwCtrlCmd.h"
  #include "yhs_msgs/FwIoCmd.h"
  #include "yhs_msgs/FwChassisInfoFb.h"
#elif CAR_TYPE == 3
  #include "yhs_msgs/FrCtrlCmd.h"
  #include "yhs_msgs/FrIoCmd.h"
  #include "yhs_msgs/FrChassisInfoFb.h"
#elif CAR_TYPE == 4
  #include "yhs_msgs/MkCtrlCmd.h"
  #include "yhs_msgs/MkIoCmd.h"
  #include "yhs_msgs/MkChassisInfoFb.h"
#else
#endif

#include "yhs_msgs/Ultrasonic.h"


namespace yhs_tool {
class CanControl
{
	public:
	CanControl();
	~CanControl();

	void run();
private:
  std::mutex mutex_;
	std::string odomFrame_, baseFrame_;
	bool tfUsed_;
  int dev_handler_;
	can_frame recv_frames_;

#if CAR_TYPE == 1
	ros::NodeHandle nh_;
	ros::Publisher chassis_info_fb_pub_;
	ros::Publisher odom_pub_;
	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	ros::Subscriber cmd_sub_;

  bool wait_for_can_frame();
	void io_cmdCallBack(const yhs_msgs::DgtIoCmd::ConstPtr& io_cmd_msg);
	void ctrl_cmdCallBack(const yhs_msgs::DgtCtrlCmd::ConstPtr& ctrl_cmd_msg);
	void cmdCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg);
	void odomPub(const float linear,const float angular);
	void recvData();
	void sendData();

#elif CAR_TYPE == 2
	ros::NodeHandle nh_;
	ros::Publisher odom_pub_;
  ros::Publisher angular_pub_;
	ros::Publisher linear_pub_;
	ros::Publisher ultrasonic_pub_;
	ros::Publisher scan_pub_;
	ros::Publisher chassis_info_fb_pub_;
	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	bool have_cmd_vel_;

	bool wait_for_can_frame();
	void io_cmdCallBack(const yhs_msgs::FwIoCmd::ConstPtr& io_cmd_msg);
	void ctrl_cmdCallBack(const yhs_msgs::FwCtrlCmd::ConstPtr& ctrl_cmd_msg);
	void cmdCallBack(const geometry_msgs::TwistStamped msg);
  void odomPub(const float linear,const float angular,const unsigned char gear,const float slipangle);
	void recvData();
	void sendData();
	
#elif CAR_TYPE == 3
	ros::NodeHandle nh_;
	ros::Publisher odom_pub_;
	ros::Publisher ultrasonic_pub_;
	ros::Publisher scan_pub_;
	ros::Publisher chassis_info_fb_pub_;
	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	ros::Subscriber imu_sub_; //dxs_add
	bool have_cmd_vel_;

	double imu_roll_,imu_pitch_,imu_yaw_; //dxs_add

	bool wait_for_can_frame();
	void io_cmdCallBack(const yhs_msgs::FrIoCmd::ConstPtr& io_cmd_msg);
	void ctrl_cmdCallBack(const yhs_msgs::FrCtrlCmd::ConstPtr& ctrl_cmd_msg);
  void cmdCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg);
  void odomPub(const float velocity,const float steering);
	void recvData();
	void imu_cmdCallBack(const sensor_msgs::Imu msg);//dxs_add
	
#elif CAR_TYPE == 4
	ros::NodeHandle nh_;
	ros::Publisher odom_pub_;
	ros::Publisher ultrasonic_pub_;
	ros::Publisher scan_pub_;
	ros::Publisher chassis_info_fb_pub_;
	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber cmd_sub_;
	ros::Subscriber io_cmd_sub_;

	bool wait_for_can_frame();
	void io_cmdCallBack(const yhs_msgs::MkIoCmd::ConstPtr& io_cmd_msg);
	void ctrl_cmdCallBack(const yhs_msgs::MkCtrlCmd::ConstPtr& ctrl_cmd_msg);
  void cmdCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg);
  void odomPub(const float velocity,const float steering);
	void recvData();

#else

#endif

};
}
#endif

