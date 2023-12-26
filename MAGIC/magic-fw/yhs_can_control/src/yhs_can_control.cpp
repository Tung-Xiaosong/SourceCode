#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"

#include <fstream>


namespace yhs_tool {

CanControl::CanControl()
{
	ros::NodeHandle private_node("~");

	private_node.param("odom_frame", odomFrame_, std::string("odom"));
	private_node.param("base_link_frame", baseFrame_, std::string("base_link"));
	private_node.param("tfUsed", tfUsed_, false);

}

CanControl::~CanControl()
{
}

bool CanControl::wait_for_can_frame() 
{
  struct timeval tv;
  fd_set rdfs;
  FD_ZERO(&rdfs);
  FD_SET(dev_handler_, &rdfs);
  tv.tv_sec = 0;
  tv.tv_usec = 50000;  // 50ms

  int ret = select(dev_handler_ + 1, &rdfs, NULL, NULL, &tv);
  if (ret == -1) 
  {
    ROS_ERROR("Error waiting for CAN frame: %s", std::strerror(errno));
    return false;
  } 
  else if (ret == 0) 
  {
    ROS_WARN("Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
whether the can line is connected correctly, and whether the chassis is powered on.");
    return false;
  } 
  else 
  {
    return true;
  }
  return false;
}

#if CAR_TYPE == 1
void CanControl::io_cmdCallBack(const yhs_msgs::DgtIoCmd::ConstPtr& io_cmd_msg)
{
  const yhs_msgs::DgtIoCmd msg = *io_cmd_msg;

	static unsigned char count = 0;

	unsigned char sendDataTemp[8] = {0};

	std::lock_guard<std::mutex> lock(mutex_);

  if(msg.io_cmd_lamp_ctrl)
    sendDataTemp[0] |= 0x01;
  if(msg.io_cmd_unlock)
    sendDataTemp[0] |= 0x02;

  if(msg.io_cmd_lower_beam_headlamp)
    sendDataTemp[1] |= 0x01;
  if(msg.io_cmd_upper_beam_headlamp)
    sendDataTemp[1] |= 0x02;

  if(msg.io_cmd_turn_lamp == 0)
    sendDataTemp[1] |= 0x00;
  if(msg.io_cmd_turn_lamp == 1)
    sendDataTemp[1] |= 0x04;
  if(msg.io_cmd_turn_lamp == 2)
    sendDataTemp[1] |= 0x08;

  if(msg.io_cmd_braking_lamp)
    sendDataTemp[1] |= 0x10;
  if(msg.io_cmd_clearance_lamp)
    sendDataTemp[1] |= 0x20;
  if(msg.io_cmd_fog_lamp)
    sendDataTemp[1] |= 0x40;

  sendDataTemp[2] = msg.io_cmd_speaker;

  count ++;
  if(count > 15) count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D7D0;
	send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
	if (ret <= 0) 
	{
		ROS_ERROR("Send message failed : %s",std::strerror(errno));
	}
}


void CanControl::ctrl_cmdCallBack(const yhs_msgs::DgtCtrlCmd::ConstPtr& ctrl_cmd_msg)
{
  yhs_msgs::DgtCtrlCmd msg = *ctrl_cmd_msg; 
	const short linear = msg.ctrl_cmd_linear * 1000;
	const short angular = msg.ctrl_cmd_angular * 100;
	const unsigned char gear = msg.ctrl_cmd_gear;

	static unsigned char count = 0;
	unsigned char sendDataTemp[8] = {0};

	std::lock_guard<std::mutex> lock(mutex_);

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendDataTemp[1] = (linear >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

	count ++;
  if(count > 15) count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
	send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
	if (ret <= 0) 
	{
		ROS_ERROR("Send message failed : %s",std::strerror(errno));
	}

}

void CanControl::cmdCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg)
{
	const short linear = cmd_msg->twist.linear.x * 1000;
	const short angular = cmd_msg->twist.angular.z / 3.14 * 180 * 100;
	const unsigned char gear = 3;
	static unsigned char count = 0;

	unsigned char sendDataTemp[8] = {0};

	std::lock_guard<std::mutex> lock(mutex_);

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendDataTemp[1] = (linear >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

  count ++;
  if(count > 15) count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
	send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
	if (ret <= 0) 
	{
		ROS_ERROR("Send message failed : %s",std::strerror(errno));
	}
}

void CanControl::recvData()
{
	ros::Rate loop(100);

	static yhs_msgs::DgtChassisInfoFb chassis_info_msg;
	while(ros::ok())
	{
    if(!wait_for_can_frame()) continue;

		if(read(dev_handler_, &recv_frames_, sizeof(recv_frames_)) >= 0)
		{
			switch (recv_frames_.can_id)
			{
				case 0x98C4D1EF:
				{
					yhs_msgs::DgtCtrlFb msg;

					msg.ctrl_fb_target_gear = recv_frames_.data[0] & 0x0f;
					msg.ctrl_fb_linear = static_cast<float>(static_cast<short>((recv_frames_.data[2] & 0x0f) << 12) | (recv_frames_.data[1] << 4) | ((recv_frames_.data[0] & 0xf0) >> 4)) / 1000.0;
					msg.ctrl_fb_angular = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12) | (recv_frames_.data[3] << 4) | ((recv_frames_.data[2] & 0xf0) >> 4)) / 100.0;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{

						chassis_info_msg.header.stamp=ros::Time::now();
						chassis_info_msg.ctrl_fb = msg;
						chassis_info_fb_pub_.publish(chassis_info_msg);

						odomPub(msg.ctrl_fb_linear * 1.0, msg.ctrl_fb_angular/180*3.14);
					}

					break;
				}

			case 0x98C4D7EF:
			{
				break;
			}

			//
			case 0x98C4D8EF:
			{
				break;
			}

			//io反馈
			case 0x98C4DAEF:
			{
				yhs_msgs::DgtIoFb msg;

				msg.io_fb_lamp_ctrl = (recv_frames_.data[0] & 0x01) != 0;
				msg.io_fb_unlock = (recv_frames_.data[1] & 0x02) != 0;
				msg.io_fb_lower_beam_headlamp = (recv_frames_.data[1] & 0x01) != 0;
				msg.io_fb_upper_beam_headlamp = (recv_frames_.data[1] & 0x02) != 0;
				msg.io_fb_turn_lamp = (recv_frames_.data[1] & 0xc0) >> 2;
				msg.io_fb_braking_lamp = (recv_frames_.data[1] & 0x10) != 0;
				msg.io_fb_clearance_lamp = (recv_frames_.data[1] & 0x20) != 0;
				msg.io_fb_fog_lamp = (recv_frames_.data[1] & 0x40) != 0;
				msg.io_fb_speaker = (recv_frames_.data[2] & 0x01) != 0;
				msg.io_fb_fl_impact_sensor = (recv_frames_.data[3] & 0x01) != 0;
				msg.io_fb_fm_impact_sensor = (recv_frames_.data[3] & 0x02) != 0;
				msg.io_fb_fr_impact_sensor = (recv_frames_.data[3] & 0x04) != 0;
				msg.io_fb_rl_impact_sensor = (recv_frames_.data[3] & 0x08) != 0;
				msg.io_fb_rm_impact_sensor = (recv_frames_.data[3] & 0x10) != 0;
				msg.io_fb_rr_impact_sensor = (recv_frames_.data[3] & 0x20) != 0;
				msg.io_fb_fl_drop_sensor = (recv_frames_.data[4] & 0x01) != 0;
				msg.io_fb_fm_drop_sensor = (recv_frames_.data[4] & 0x02) != 0;
				msg.io_fb_fr_drop_sensor = (recv_frames_.data[4] & 0x04) != 0;
				msg.io_fb_rl_drop_sensor = (recv_frames_.data[4] & 0x08) != 0;
				msg.io_fb_rm_drop_sensor = (recv_frames_.data[4] & 0x10) != 0;
				msg.io_fb_rr_drop_sensor = (recv_frames_.data[4] & 0x20) != 0;
				msg.io_fb_estop = (recv_frames_.data[5] & 0x01) != 0;
				msg.io_fb_joypad_ctrl = (recv_frames_.data[5] & 0x02) != 0;
				msg.io_fb_charge_state = (recv_frames_.data[5] & 0x04) != 0;

				unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

				if(crc == recv_frames_.data[7])
				{
					chassis_info_msg.io_fb = msg;
				}

				break;
			}


			//bms反馈
			case 0x98C4E1EF:
			{
				yhs_msgs::DgtBmsFb msg;

				msg.bms_fb_voltage = static_cast<float>(reinterpret_cast<unsigned short&>(recv_frames_.data[0])) / 100;
				msg.bms_fb_current = static_cast<float>(reinterpret_cast<short&>(recv_frames_.data[2])) / 100;
				msg.bms_fb_remaining_capacity = static_cast<float>(reinterpret_cast<unsigned short&>(recv_frames_.data[4])) / 100;

				unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

				if(crc == recv_frames_.data[7])
				{
					chassis_info_msg.bms_fb = msg;
				}

				break;
			}

			//bms_flag反馈
			case 0x98C4E2EF:
			{
				yhs_msgs::DgtBmsFlagFb msg;

				msg.bms_flag_fb_soc = recv_frames_.data[0];
				msg.bms_flag_fb_single_ov = (0x01 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_single_uv = (0x02 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_ov = (0x04 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_uv = (0x08 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_charge_ot = (0x10 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_charge_ut = (0x20 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_discharge_ot = (0x40 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_discharge_ut = (0x80 & recv_frames_.data[1]) != 0;
				msg.bms_flag_fb_charge_oc = (0x01 & recv_frames_.data[2]) != 0;
				msg.bms_flag_fb_discharge_oc = (0x02 & recv_frames_.data[2]) != 0;
				msg.bms_flag_fb_short = (0x04 & recv_frames_.data[2]) != 0;
				msg.bms_flag_fb_ic_error = (0x08 & recv_frames_.data[2]) != 0;
				msg.bms_flag_fb_lock_mos = (0x10 & recv_frames_.data[2]) != 0;
				msg.bms_flag_fb_charge_flag = (0x20 & recv_frames_.data[2]) != 0;

				const float kTemperatureConversionFactor = 0.1;
				msg.bms_flag_fb_hight_temperature = static_cast<float>((static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4))) * kTemperatureConversionFactor;
				msg.bms_flag_fb_low_temperature = static_cast<float>((static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5]))) * kTemperatureConversionFactor;

				unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

				if(crc == recv_frames_.data[7])
				{
					chassis_info_msg.bms_flag_fb = msg;
				}

				break;
			}

				default:
				break;
		 }			
		}
	}
}

void CanControl::odomPub(const float linear,const float angular)
{
	static double x = 0.0;
	static double y = 0.0;
	static double th = 0.0;

	static double lastYaw = 0;

	static tf::TransformBroadcaster odom_broadcaster;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time;


	double vx = linear;
	double vth = angular;

	current_time = ros::Time::now();

	double dt = (current_time - last_time).toSec();

	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odomFrame_;
	odom_trans.child_frame_id = baseFrame_;

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	if(tfUsed_)
	odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odomFrame_;

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.child_frame_id = baseFrame_;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0]  = 0.1;   	
	odom.pose.covariance[7]  = 0.1;		
	odom.pose.covariance[35] = 0.2;   

	odom.pose.covariance[14] = 1e10; 	
	odom.pose.covariance[21] = 1e10; 	
	odom.pose.covariance[28] = 1e10; 	
	odom_pub_.publish(odom);

	last_time = current_time;
}

void CanControl::sendData()
{
	ros::Rate loop(100);
	while(ros::ok())
	{
		loop.sleep();
	}
}

///////////////////////////////////////////
//////////////////////////////////////////

#elif CAR_TYPE == 2
//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_msgs::FwIoCmd::ConstPtr& io_cmd_msg)
{
  const yhs_msgs::FwIoCmd msg = *io_cmd_msg;
  
	static unsigned char count = 0;
	unsigned char sendDataTemp[8] = {0};
  
	std::lock_guard<std::mutex> lock(mutex_);

	memset(sendDataTemp,0,8);

	sendDataTemp[0] = 0xff;
	if(msg.io_cmd_lamp_ctrl)
		sendDataTemp[0] &= 0xff;
	else sendDataTemp[0] &= 0xfe;
	if(msg.io_cmd_unlock)
		sendDataTemp[0] &= 0xff;
	else sendDataTemp[0] &= 0xfd;

	sendDataTemp[1] = 0xff;
	if(msg.io_cmd_lower_beam_headlamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xfe;
	if(msg.io_cmd_upper_beam_headlamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendDataTemp[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendDataTemp[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendDataTemp[1] &= 0xfb;

	if(msg.io_cmd_braking_lamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xef;
	if(msg.io_cmd_clearance_lamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xdf;
	if(msg.io_cmd_fog_lamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xbf;

	sendDataTemp[2] = msg.io_cmd_speaker;

	count ++;
	if(count == 16)	count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];
	
	can_frame send_frame;

	send_frame.can_id = 0x98C4D7D0;
  send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
	{
    ROS_ERROR("Send message failed : %s",std::strerror(errno));
  }
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_msgs::FwCtrlCmd::ConstPtr& ctrl_cmd_msg)
{
	yhs_msgs::FwCtrlCmd msg = *ctrl_cmd_msg;
	
	const short linear = msg.ctrl_cmd_linear * 1000;
	const short angular = msg.ctrl_cmd_angular * 100;
  const unsigned char gear = msg.ctrl_cmd_gear;
	const short slipangle = msg.ctrl_cmd_slipangle * 100;

	static unsigned char count = 0;
	unsigned char sendDataTemp[8] = {0};

	std::lock_guard<std::mutex> lock(mutex_);
	
	have_cmd_vel_ = true;

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);
	
	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendDataTemp[1] = (linear >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

	sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendDataTemp[5] = (slipangle >> 4) & 0xff;

	sendDataTemp[6] = sendDataTemp[6] | (0x0f & (slipangle >> 12));

	count ++;
	if(count == 16)	count = 0;

	sendDataTemp[6] =  sendDataTemp[6] | (count << 4);

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];
	
	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
  send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
  {
    ROS_ERROR("Send message failed : %s",std::strerror(errno));
  }
}

void CanControl::cmdCallBack(const geometry_msgs::TwistStamped msg)
{
	const short linear = msg.twist.linear.x * 1000;
	const short angular = msg.twist.angular.z / 3.14 * 180 * 100;
	const short slipangle = 0;
	const int gear = 6;

	static unsigned char count = 0;
	unsigned char sendDataTemp[8] = {0};

	std::lock_guard<std::mutex> lock(mutex_);

  have_cmd_vel_ = true;

	memset(sendDataTemp,0,8);

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);
	
	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendDataTemp[1] = (linear >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));
	
	sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendDataTemp[5] = (slipangle >> 4) & 0xff;

	sendDataTemp[6] = sendDataTemp[6] | (0x0f & (slipangle >> 12));

	count ++;
	if(count == 16)	count = 0;

	sendDataTemp[6] =  sendDataTemp[6] | (count << 4);
	
	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];
	
	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
  send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
	{
    ROS_ERROR("Send message failed : %s",std::strerror(errno));
  }
}

//数据接收解析线程
void CanControl::recvData()
{

	static yhs_msgs::FwChassisInfoFb chassis_info_msg;
	while(ros::ok())
	{
    if(!wait_for_can_frame()) continue;
    
		if(read(dev_handler_, &recv_frames_, sizeof(recv_frames_)) >= 0)
		{
			switch (recv_frames_.can_id)
			{
				//
				case 0x98C4D1EF:
				{
					yhs_msgs::FwCtrlFb msg;
					
					msg.ctrl_fb_gear = 0x0f & recv_frames_.data[0];
					
					msg.ctrl_fb_linear = static_cast<float>(static_cast<short>((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;
					
					msg.ctrl_fb_angular = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

					msg.ctrl_fb_slipangle = static_cast<float>(static_cast<short>((recv_frames_.data[6] & 0x0f) << 12 | recv_frames_.data[5] << 4 | (recv_frames_.data[4] & 0xf0) >> 4)) / 100;
					

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.header.stamp=ros::Time::now();
						chassis_info_msg.ctrl_fb = msg;
						chassis_info_fb_pub_.publish(chassis_info_msg);
						
						odomPub(msg.ctrl_fb_linear, msg.ctrl_fb_angular/180*3.14,msg.ctrl_fb_gear,msg.ctrl_fb_slipangle/180*3.14);

						std_msgs::Float32 linear_msg;
						linear_msg.data = msg.ctrl_fb_linear;
						linear_pub_.publish(linear_msg);
					}

					break;
				}

				//io反馈
				case 0x98C4DAEF:
				{
					yhs_msgs::FwIoFb msg;
					if(0x01 & recv_frames_.data[0]) msg.io_fb_lamp_ctrl = true;	else msg.io_fb_lamp_ctrl = false;

					if(0x02 & recv_frames_.data[1]) msg.io_fb_unlock = true;	else msg.io_fb_unlock = false;

					if(0x01 & recv_frames_.data[1]) msg.io_fb_lower_beam_headlamp = true;	else msg.io_fb_lower_beam_headlamp = false;

					if(0x02 & recv_frames_.data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

					msg.io_fb_turn_lamp = (0x0c & recv_frames_.data[1]) >> 2;

					if(0x10 & recv_frames_.data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

					if(0x20 & recv_frames_.data[1]) msg.io_fb_clearance_lamp = true;	else msg.io_fb_clearance_lamp = false;

					if(0x40 & recv_frames_.data[1]) msg.io_fb_fog_lamp = true;	else msg.io_fb_fog_lamp = false;

					if(0x01 & recv_frames_.data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

					if(0x01 & recv_frames_.data[3]) msg.io_fb_fl_impact_sensor = true;	else msg.io_fb_fl_impact_sensor = false;

					if(0x02 & recv_frames_.data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

					if(0x04 & recv_frames_.data[3]) msg.io_fb_fr_impact_sensor = true;	else msg.io_fb_fr_impact_sensor = false;

					if(0x08 & recv_frames_.data[3]) msg.io_fb_rl_impact_sensor = true;	else msg.io_fb_rl_impact_sensor = false;

					if(0x10 & recv_frames_.data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

					if(0x20 & recv_frames_.data[3]) msg.io_fb_rr_impact_sensor = true;	else msg.io_fb_rr_impact_sensor = false;

					if(0x01 & recv_frames_.data[4]) msg.io_fb_fl_drop_sensor = true;	else msg.io_fb_fl_drop_sensor = false;

					if(0x02 & recv_frames_.data[4]) msg.io_fb_fm_drop_sensor = true;	else msg.io_fb_fm_drop_sensor = false;

					if(0x04 & recv_frames_.data[4]) msg.io_fb_fr_drop_sensor = true;	else msg.io_fb_fr_drop_sensor = false;

					if(0x08 & recv_frames_.data[4]) msg.io_fb_rl_drop_sensor = true;	else msg.io_fb_rl_drop_sensor = false;

					if(0x10 & recv_frames_.data[4]) msg.io_fb_rm_drop_sensor = true;	else msg.io_fb_rm_drop_sensor = false;

					if(0x20 & recv_frames_.data[4]) msg.io_fb_rr_drop_sensor = true;	else msg.io_fb_rr_drop_sensor = false;

					if(0x01 & recv_frames_.data[5]) msg.io_fb_estop = true;	else msg.io_fb_estop = false;

					if(0x02 & recv_frames_.data[5]) msg.io_fb_joypad_ctrl = true;	else msg.io_fb_joypad_ctrl = false;

					if(0x04 & recv_frames_.data[5]) msg.io_fb_charge_state = true;	else msg.io_fb_charge_state = false;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.io_fb = msg;
					}

					break;
				}

				//
				case 0x98C4DCEF:
				{
					float front_angle_fb_l = static_cast<float>(static_cast<short>(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

					float front_angle_fb_r = static_cast<float>(static_cast<short>(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						std_msgs::Float32 angular_msg;
						angular_msg.data = front_angle_fb_l;
						angular_pub_.publish(angular_msg);
					}

					break;
				}

				//bms反馈
				case 0x98C4E1EF:
				{
					yhs_msgs::FwBmsFb msg;
					msg.bms_fb_voltage = static_cast<float>(static_cast<short>(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

					msg.bms_fb_current = static_cast<float>(static_cast<short>(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

					msg.bms_fb_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frames_.data[5] << 8 | recv_frames_.data[4])) / 100;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.bms_fb = msg;
					}

					break;
				}

				//bms_flag反馈
				case 0x98C4E2EF:
				{
					yhs_msgs::FwBmsFlagFb msg;
					msg.bms_flag_fb_soc = recv_frames_.data[0];

					if(0x01 & recv_frames_.data[1]) msg.bms_flag_fb_single_ov = true;	else msg.bms_flag_fb_single_ov = false;

					if(0x02 & recv_frames_.data[1]) msg.bms_flag_fb_single_uv = true;	else msg.bms_flag_fb_single_uv = false;

					if(0x04 & recv_frames_.data[1]) msg.bms_flag_fb_ov = true;	else msg.bms_flag_fb_ov = false;

					if(0x08 & recv_frames_.data[1]) msg.bms_flag_fb_uv = true;	else msg.bms_flag_fb_uv = false;

					if(0x10 & recv_frames_.data[1]) msg.bms_flag_fb_charge_ot = true;	else msg.bms_flag_fb_charge_ot = false;

					if(0x20 & recv_frames_.data[1]) msg.bms_flag_fb_charge_ut = true;	else msg.bms_flag_fb_charge_ut = false;

					if(0x40 & recv_frames_.data[1]) msg.bms_flag_fb_discharge_ot = true;	else msg.bms_flag_fb_discharge_ot = false;

					if(0x80 & recv_frames_.data[1]) msg.bms_flag_fb_discharge_ut = true;	else msg.bms_flag_fb_discharge_ut = false;

					if(0x01 & recv_frames_.data[2]) msg.bms_flag_fb_charge_oc = true;	else msg.bms_flag_fb_charge_oc = false;

					if(0x02 & recv_frames_.data[2]) msg.bms_flag_fb_discharge_oc = true;	else msg.bms_flag_fb_discharge_oc = false;

					if(0x04 & recv_frames_.data[2]) msg.bms_flag_fb_short = true;	else msg.bms_flag_fb_short = false;

					if(0x08 & recv_frames_.data[2]) msg.bms_flag_fb_ic_error = true;	else msg.bms_flag_fb_ic_error = false;

					if(0x10 & recv_frames_.data[2]) msg.bms_flag_fb_lock_mos = true;	else msg.bms_flag_fb_lock_mos = false;

					if(0x20 & recv_frames_.data[2]) msg.bms_flag_fb_charge_flag = true;	else msg.bms_flag_fb_charge_flag = false;

					msg.bms_flag_fb_hight_temperature = static_cast<float>(static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4)) / 10;

					msg.bms_flag_fb_low_temperature = static_cast<float>(static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5])) / 10;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.bms_flag_fb = msg;
					}

					break;
				}
				default:
					break;
			}				
		}
	}
}


void CanControl::odomPub(const float linear,const float angular,const unsigned char gear,const float slipangle)
{
	static double x = 0.0;
	static double y = 0.0;
	static double th = 0.0;

	static double lastYaw = 0;

	static tf::TransformBroadcaster odom_broadcaster;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time;

	static float last_slipangle = slipangle;

	double vx = linear;
	double vth = angular;

	current_time = ros::Time::now();

	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time - last_time).toSec();

	if(gear == 7)
	{
		th += slipangle - last_slipangle;
	}
	
	last_slipangle = slipangle;

	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odomFrame_;
	odom_trans.child_frame_id = baseFrame_;

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;


	if(tfUsed_)
		odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odomFrame_;

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = baseFrame_;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0]  = 0.1;   
	odom.pose.covariance[7]  = 0.1;	
	odom.pose.covariance[35] = 0.2;  

	odom.pose.covariance[14] = 1e10; 	
	odom.pose.covariance[21] = 1e10; 	
	odom.pose.covariance[28] = 1e10; 	

	//publish the message
	odom_pub_.publish(odom);

	last_time = current_time;

}

//数据发送线程
void CanControl::sendData()
{
	ros::Rate loop(9);

	while(ros::ok())
	{
		if(!have_cmd_vel_)
		{
			const unsigned char gear = 6;
			static unsigned char count = 0;

	    unsigned char sendDataTemp[8] = {0};

			sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);
	
			count ++;
			if(count == 16)	count = 0;

			sendDataTemp[6] =  sendDataTemp[6] | (count << 4);

			sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];
			
			can_frame send_frame;

			send_frame.can_id = 0x98C4D1D0;
			send_frame.can_dlc = 8;

			memcpy(send_frame.data, sendDataTemp, 8);

      std::lock_guard<std::mutex> lock(mutex_);
			int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
			if (ret <= 0) 
			{
			  ROS_ERROR("send message failed, error code: %d",ret);
			}
		}

		have_cmd_vel_ = false;
		loop.sleep();
	}
}
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#elif CAR_TYPE == 3
void CanControl::io_cmdCallBack(const yhs_msgs::FrIoCmd::ConstPtr& io_cmd_msg)
{
  const yhs_msgs::FrIoCmd msg = *io_cmd_msg;

	static unsigned char count = 0;

  unsigned char sendDataTemp[8] = {0};

  sendDataTemp[0] = msg.io_cmd_enable;

  if(msg.io_cmd_upper_beam_headlamp)
    sendDataTemp[1] |= 0x20;

  if(msg.io_cmd_turn_lamp == 0)
    sendDataTemp[1] |= 0x00;
  if(msg.io_cmd_turn_lamp == 1)
    sendDataTemp[1] |= 0x04;
  if(msg.io_cmd_turn_lamp == 2)
    sendDataTemp[1] |= 0x08;

  sendDataTemp[2] = msg.io_cmd_speaker;

  count ++;
  if(count > 15) count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D7D0;
	send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
	if (ret <= 0) 
	{
		ROS_ERROR("Send message failed : %s",std::strerror(errno));
	}
}


void CanControl::ctrl_cmdCallBack(const yhs_msgs::FrCtrlCmd::ConstPtr& ctrl_cmd_msg)
{
  yhs_msgs::FrCtrlCmd msg = *ctrl_cmd_msg; 
  const unsigned short vel = msg.ctrl_cmd_velocity * 1000;
  const short angular = msg.ctrl_cmd_steering * 100;
  const unsigned char gear = msg.ctrl_cmd_gear;
  const unsigned char brake = msg.ctrl_cmd_Brake;

  static unsigned char count = 0;
  unsigned char sendDataTemp[8] = {0};

  if(msg.ctrl_cmd_velocity < 0) return;

  sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

  sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((vel & 0x0f) << 4));

  sendDataTemp[1] = (vel >> 4) & 0xff;

  sendDataTemp[2] = sendDataTemp[2] | (0x0f & (vel >> 12));

  sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

  sendDataTemp[3] = (angular >> 4) & 0xff;

  sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((brake & 0x0f) << 4));

  sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

  sendDataTemp[5] = (brake >> 4) & 0x0f;

	count ++;
  if(count > 15) count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
	send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
	if (ret <= 0) 
	{
		ROS_ERROR("Send message failed : %s",std::strerror(errno));
	}

}

void CanControl::cmdCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg)
{
  const geometry_msgs::TwistStamped msg = *cmd_msg;
	const short vel = abs(msg.twist.linear.x * 1000);
	const unsigned char ctrl_cmd_gear = msg.twist.linear.x < 0 ? 2 : 4;
	const short angular = msg.twist.angular.z / 3.14 * 180 * 100;
	static unsigned char count = 0;

  unsigned char sendDataTemp[8] = {0};

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & ctrl_cmd_gear);
	
	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendDataTemp[1] = (vel >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (vel >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((0 & 0x0f) << 4));

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

	sendDataTemp[5] = 0;

  count ++;
  if(count > 15) count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
	send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
	if (ret <= 0) 
	{
		ROS_ERROR("Send message failed : %s",std::strerror(errno));
	}
}

void CanControl::recvData()
{
	ros::Rate loop(100);

	static yhs_msgs::FrChassisInfoFb chassis_info_msg;
	while(ros::ok())
	{
    if(!wait_for_can_frame()) continue;

		if(read(dev_handler_, &recv_frames_, sizeof(recv_frames_)) >= 0)
		{
			switch (recv_frames_.can_id)
			{
				//
        case 0x98C4D2EF:
        {
          yhs_msgs::FrCtrlFb msg;
        
          msg.ctrl_fb_gear = 0x0f & recv_frames_.data[0];
        
          msg.ctrl_fb_velocity = static_cast<float>(static_cast<unsigned int>((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;
        
          msg.ctrl_fb_steering = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

          msg.ctrl_fb_Brake = (recv_frames_.data[4] & 0xf0) >> 4 | (recv_frames_.data[5] & 0x0f) << 4;
        
          msg.ctrl_fb_mode = (recv_frames_.data[5] & 0x30) >> 4;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if(crc == recv_frames_.data[7]){	
            chassis_info_msg.header.stamp = ros::Time::now();
            chassis_info_msg.ctrl_fb = msg;
            chassis_info_fb_pub_.publish(chassis_info_msg);
            if(msg.ctrl_fb_gear == 2) msg.ctrl_fb_velocity = -msg.ctrl_fb_velocity;
            odomPub(msg.ctrl_fb_velocity, msg.ctrl_fb_steering/180*3.1415);
          }

          break;
        }

        //
        case 0x98C4DAEF:
        {
          yhs_msgs::FrIoFb msg;
        
          msg.io_fb_enable = (recv_frames_.data[0] & 0x01) != 0;
          msg.io_fb_upper_beam_headlamp = (recv_frames_.data[1] & 0x02) != 0;

          msg.io_fb_turn_lamp = (0x0c & recv_frames_.data[1]) >> 2;

          msg.io_fb_braking_lamp = (0x10 & recv_frames_.data[1]) != 0;
          msg.io_fb_speaker = (0x01 & recv_frames_.data[2]) != 0;
          msg.io_fb_fm_impact_sensor = (0x02 & recv_frames_.data[3]) != 0;
          msg.io_fb_rm_impact_sensor = (0x10 & recv_frames_.data[3]) != 0;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if(crc == recv_frames_.data[7]){
            chassis_info_msg.io_fb = msg;	
          }

          break;
        }


        //
        case 0x98C4E1EF:
        {
          yhs_msgs::FrBmsInfoFb msg;
        
          msg.bms_info_voltage = static_cast<float>(static_cast<short>(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          msg.bms_info_current = static_cast<float>(static_cast<short>(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          msg.bms_info_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frames_.data[5] << 8 | recv_frames_.data[4])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if(crc == recv_frames_.data[7]){
            chassis_info_msg.bms_info_fb = msg;	
          }

          break;
        }

        //
        case 0x98C4E2EF:
        {
          yhs_msgs::FrBmsFlagInfoFb msg;
        
          msg.bms_flag_info_soc = recv_frames_.data[0];

          msg.bms_flag_info_single_ov = (recv_frames_.data[1] & 0x01) != 0;
          msg.bms_flag_info_single_uv = (recv_frames_.data[1] & 0x02) != 0;
          msg.bms_flag_info_ov = (recv_frames_.data[1] & 0x04) != 0;
          msg.bms_flag_info_uv = (recv_frames_.data[1] & 0x08) != 0;
          msg.bms_flag_info_charge_ot = (recv_frames_.data[1] & 0x10) != 0;
          msg.bms_flag_info_charge_ut = (recv_frames_.data[1] & 0x20) != 0;
          msg.bms_flag_info_discharge_ot = (recv_frames_.data[1] & 0x40) != 0;
          msg.bms_flag_info_discharge_ut = (recv_frames_.data[1] & 0x80) != 0;

          msg.bms_flag_info_charge_oc = (recv_frames_.data[2] & 0x01) != 0;
          msg.bms_flag_info_discharge_oc = (recv_frames_.data[2] & 0x02) != 0;
          msg.bms_flag_info_short = (recv_frames_.data[2] & 0x04) != 0;
          msg.bms_flag_info_ic_error = (recv_frames_.data[2] & 0x08) != 0;
          msg.bms_flag_info_lock_mos = (recv_frames_.data[2] & 0x10) != 0;
          msg.bms_flag_info_charge_flag = (recv_frames_.data[2] & 0x20) != 0;


          msg.bms_flag_info_hight_temperature = static_cast<float>(static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4)) / 10;
          msg.bms_flag_info_low_temperature = static_cast<float>(static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5])) / 10;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if(crc == recv_frames_.data[7]){
            chassis_info_msg.bms_flag_info_fb = msg;	
          }

          break;
        }

        //
        case 0x98C4EAEF:
        {
          yhs_msgs::FrVehDiagFb msg;
        
          msg.veh_fb_fault_level = 0x0f & recv_frames_.data[0];

          msg.veh_fb_auto_can_ctrl_cmd = (recv_frames_.data[0] & 0x10) != 0;
          msg.veh_fb_auto_io_can_cmd = (recv_frames_.data[0] & 0x20) != 0;
          msg.veh_fb_eps_dis_on_line = (recv_frames_.data[1] & 0x01) != 0;
          msg.veh_fb_eps_fault = (recv_frames_.data[1] & 0x02) != 0;
          msg.veh_fb_eps_mosf_et_ot = (recv_frames_.data[1] & 0x04) != 0;
          msg.veh_fb_eps_warning = (recv_frames_.data[1] & 0x08) != 0;
          msg.veh_fb_eps_dis_work = (recv_frames_.data[1] & 0x10) != 0;
          msg.veh_fb_eps_over_current = (recv_frames_.data[1] & 0x20) != 0;
          msg.veh_fb_ehb_ecu_fault = (recv_frames_.data[2] & 0x10) != 0;
          msg.veh_fb_ehb_dis_on_line = (recv_frames_.data[2] & 0x20) != 0;
          msg.veh_fb_ehb_work_model_fault = (recv_frames_.data[2] & 0x40) != 0;
          msg.veh_fb_ehb_dis_en = (recv_frames_.data[2] & 0x80) != 0;
          msg.veh_fb_ehb_anguler_fault = (recv_frames_.data[3] & 0x01) != 0;
          msg.veh_fb_ehb_ot = (recv_frames_.data[3] & 0x02) != 0;
          msg.veh_fb_ehb_power_fault = (recv_frames_.data[3] & 0x04) != 0;
          msg.veh_fb_ehb_sensor_abnomal = (recv_frames_.data[3] & 0x08) != 0;
          msg.veh_fb_ehb_motor_fault = (recv_frames_.data[3] & 0x10) != 0;
          msg.veh_fb_ehb_oil_press_sensor_fault = (recv_frames_.data[3] & 0x20) != 0;
          msg.veh_fb_ehb_oil_fault = (recv_frames_.data[3] & 0x40) != 0;
          msg.veh_fb_drv_mcu_dis_on_line = (recv_frames_.data[4] & 0x01) != 0;
          msg.veh_fb_drv_mcu_ot = (recv_frames_.data[4] & 0x02) != 0;
          msg.veh_fb_drv_mcu_ov = (recv_frames_.data[4] & 0x04) != 0;
          msg.veh_fb_drv_mcu_uv = (recv_frames_.data[4] & 0x08) != 0;
          msg.veh_fb_drv_mcu_short = (recv_frames_.data[4] & 0x10) != 0;
          msg.veh_fb_drv_mcu_scram = (recv_frames_.data[4] & 0x20) != 0;
          msg.veh_fb_drv_mcu_hall = (recv_frames_.data[4] & 0x40) != 0;
          msg.veh_fb_drv_mcu_mosf_ef = (recv_frames_.data[4] & 0x80) != 0;
          msg.veh_fb_aux_bms_dis_on_line = (recv_frames_.data[5] & 0x10) != 0;
          msg.veh_fb_aux_scram = recv_frames_.data[5] & 0x20;
          msg.veh_fb_aux_remote_close = recv_frames_.data[5] & 0x40;
          msg.veh_fb_aux_remote_dis_on_line = recv_frames_.data[5] & 0x80;


          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if(crc == recv_frames_.data[7]){
            chassis_info_msg.veh_diag_fb = msg;
          }


          break;
        }

        default:
          break;
		 }			
		}
	}
}

void CanControl::odomPub(const float velocity,const float steering)
{
	static double x = 0.0;
	static double y = 0.0;
	static double th = 0.0;

	double x_mid = 0.0;
	double y_mid = 0.0;

	static tf::TransformBroadcaster odom_broadcaster;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time;

	double vx = velocity;
	double vth = vx * tan(steering) / 0.66;

	current_time = ros::Time::now();

	double dt = (current_time - last_time).toSec();

	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	x_mid = x + 0.33 * cos(th);
	y_mid = y + 0.33 * sin(th);

	//转换为四元素
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odomFrame_;
	odom_trans.child_frame_id = baseFrame_;

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//是否发布tf转换
	if(tfUsed_)
		odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odomFrame_;

	//set the position
	odom.pose.pose.position.x = x_mid;
	odom.pose.pose.position.y = y_mid;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = baseFrame_;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0]  = 0.1;   	
	odom.pose.covariance[7]  = 0.1;		
	odom.pose.covariance[35] = 0.2;   	

	odom.pose.covariance[14] = 1e10; 	
	odom.pose.covariance[21] = 1e10; 	
	odom.pose.covariance[28] = 1e10; 	

	//publish the message
	odom_pub_.publish(odom);

	last_time = current_time;

}

#else
#endif

void CanControl::run()
{

#if CAR_TYPE == 1
	ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::DgtCtrlCmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_msgs::DgtIoCmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	cmd_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("smoother_cmd_vel", 5, &CanControl::cmdCallBack, this);

	chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::DgtChassisInfoFb>("chassis_info_fb",5);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

//////////////////////////////////
/////////////////////////////////

#elif CAR_TYPE == 2
	ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::FwCtrlCmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_msgs::FwIoCmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	cmd_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("smoother_cmd_vel", 5, &CanControl::cmdCallBack, this);

	chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::FwChassisInfoFb>("chassis_info_fb",5);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
	angular_pub_ = nh_.advertise<std_msgs::Float32>("angular", 5);
	linear_pub_ = nh_.advertise<std_msgs::Float32>("linear", 5);
	ultrasonic_pub_ = nh_.advertise<yhs_msgs::Ultrasonic>("ultrasonic",5);
	scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan3", 5);
	
//////////////////////////////////
/////////////////////////////////

#elif CAR_TYPE == 3
  ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::FrCtrlCmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_msgs::FrIoCmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	cmd_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("smoother_cmd_vel", 5, &CanControl::cmdCallBack, this);

	chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::FrChassisInfoFb>("chassis_info_fb",5);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
	ultrasonic_pub_ = nh_.advertise<yhs_msgs::Ultrasonic>("ultrasonic",5);
	scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan3", 5);

#else
#endif

	//打开设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR("Open can deivce error : %s",std::strerror(errno));
		return;
	}
	else
	{
		ROS_INFO("Open can deivce success!");
	}

	struct ifreq ifr;
	std::string can_name("can0");
	strcpy(ifr.ifr_name,can_name.c_str());
	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR("Bind dev_handler error : %s",std::strerror(errno));
		return;
	}
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));

#if CAR_TYPE == 2
	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));
#endif

	ros::spin();
	if(recvdata_thread.joinable()) recvdata_thread.join();
	close(dev_handler_);
}

}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

  std::cout << "Yhs_can_control_node Stop." << std::endl;
	return 0;
}
