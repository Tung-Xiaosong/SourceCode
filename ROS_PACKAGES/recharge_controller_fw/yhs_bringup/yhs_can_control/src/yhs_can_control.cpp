#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"


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

//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = 0xff;
	if(msg.io_cmd_lamp_ctrl)
		sendData_u_io_[0] &= 0xff;
	else sendData_u_io_[0] &= 0xfe;
	if(msg.io_cmd_unlock)
		sendData_u_io_[0] &= 0xff;
	else sendData_u_io_[0] &= 0xfd;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_lower_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfe;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	if(msg.io_cmd_braking_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xef;
	if(msg.io_cmd_clearance_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xdf;
	if(msg.io_cmd_fog_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xbf;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	short linear = msg.ctrl_cmd_linear * 1000;
	short angular = msg.ctrl_cmd_angular * 100;
	short slipangle = msg.ctrl_cmd_slipangle * 100;
	static unsigned char count = 0;


	cmd_mutex_.lock();
	have_cmd_vel_ = true;

	memset(sendData_u_vel_,0,8);

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendData_u_vel_[1] = (linear >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (linear >> 12));


	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	
	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendData_u_vel_[5] = (slipangle >> 4) & 0xff;


	sendData_u_vel_[6] = sendData_u_vel_[6] | (0x0f & (slipangle >> 12));


	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  sendData_u_vel_[6] | (count << 4);
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D1D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}

void CanControl::cmdCallBack(const geometry_msgs::Twist msg)
{
	short linear = msg.linear.x * 1000;
	short angular = msg.angular.z / 3.14 * 180 * 100;
	short slipangle = 0;
	int gear = 6;
	static unsigned char count = 0;

	cmd_mutex_.lock();
    have_cmd_vel_ = true;

	memset(sendData_u_vel_,0,8);

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendData_u_vel_[1] = (linear >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (linear >> 12));


	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	
	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendData_u_vel_[5] = (slipangle >> 4) & 0xff;


	sendData_u_vel_[6] = sendData_u_vel_[6] | (0x0f & (slipangle >> 12));


	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  sendData_u_vel_[6] | (count << 4);
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D1D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


void CanControl::steering_ctrl_cmdCallBack(const yhs_can_msgs::steering_ctrl_cmd msg)
{
	short linear = msg.steering_ctrl_cmd_velocity * 1000;
	short angular = msg.steering_ctrl_cmd_steering * 100;
	short slipangle = msg.steering_ctrl_cmd_slipangle * 100;
	static unsigned char count_2 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();
	have_cmd_vel_ = true;

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendData_u_tem_[1] = (linear >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linear >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_tem_[3] = (angular >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angular >> 12));

	
	sendData_u_tem_[4] = sendData_u_tem_[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendData_u_tem_[5] = (slipangle >> 4) & 0xff;

	sendData_u_tem_[6] = sendData_u_tem_[6] | (0x0f & (slipangle >> 12));


	count_2 ++;

	if(count_2 == 16)	count_2 = 0;

	sendData_u_tem_[6] =  count_2 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];


	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}

void CanControl::front_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::front_angle_free_ctrl_cmd msg)
{
	short angularl = msg.free_ctrl_cmd_angle_lf * 100;
	short angularr = msg.free_ctrl_cmd_angle_rf * 100;
	static unsigned char count_4 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();
	have_cmd_vel_ = true;

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((angularl & 0x0f) << 4));

	sendData_u_tem_[1] = (angularl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (angularl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angularr & 0x0f) << 4));

	sendData_u_tem_[3] = (angularr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angularr >> 12));


	count_4 ++;

	if(count_4 == 16)	count_4 = 0;

	sendData_u_tem_[6] =  count_4 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D5D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();

}

void CanControl::front_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::front_velocity_free_ctrl_cmd msg)
{
	short linearl = msg.free_ctrl_cmd_velocity_lf * 1000;
	short linearr = msg.free_ctrl_cmd_velocity_rf * 1000;
	static unsigned char count_3 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();
	have_cmd_vel_ = true;

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

	sendData_u_tem_[1] = (linearl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

	sendData_u_tem_[3] = (linearr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

	count_3 ++;

	if(count_3 == 16)	count_3 = 0;

	sendData_u_tem_[6] =  count_3 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D3D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


void CanControl::rear_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_angle_free_ctrl_cmd msg)
{
	short angularl = msg.free_ctrl_cmd_angle_lr * 100;
	short angularr = msg.free_ctrl_cmd_angle_rr * 100;
	static unsigned char count_6 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();
	have_cmd_vel_ = true;

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((angularl & 0x0f) << 4));

	sendData_u_tem_[1] = (angularl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (angularl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angularr & 0x0f) << 4));

	sendData_u_tem_[3] = (angularr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angularr >> 12));


	count_6 ++;

	if(count_6 == 16)	count_6 = 0;

	sendData_u_tem_[6] =  count_6 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D6D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();

}

void CanControl::rear_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_velocity_free_ctrl_cmd msg)
{
	short linearl = msg.free_ctrl_cmd_velocity_lr * 1000;
	short linearr = msg.free_ctrl_cmd_velocity_rr * 1000;
	static unsigned char count_5 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	have_cmd_vel_ = true;

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

	sendData_u_tem_[1] = (linearl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

	sendData_u_tem_[3] = (linearr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

	count_5 ++;

	if(count_5 == 16)	count_5 = 0;

	sendData_u_tem_[6] =  count_5 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D4D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }
	

	cmd_mutex_.unlock();

}

//数据接收解析线程
void CanControl::recvData()
{

	while(ros::ok())
	{

		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
		{
			for(int j=0;j<1;j++)
			{
				
				switch (recv_frames_[0].can_id)
				{
					//
					case 0x98C4D1EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						
						msg.ctrl_fb_linear = (float)((short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_angular = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_slipangle = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 12 | recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] & 0xf0) >> 4)) / 100;
						

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
							ctrl_fb_pub_.publish(msg);
							odomPub(msg.ctrl_fb_linear,msg.ctrl_fb_angular/180*3.14);

							std_msgs::Float32 linear_msg;
							linear_msg.data = msg.ctrl_fb_linear;
							linear_pub_.publish(linear_msg);
						}

						break;
					}

					//
					case 0x98C4D2EF:
					{
						yhs_can_msgs::steering_ctrl_fb msg;
						msg.steering_ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						
						msg.steering_ctrl_fb_velocity = (float)((short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						msg.steering_ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

						msg.steering_ctrl_fb_slipangle = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 12 | recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] & 0xf0) >> 4)) / 100;
						

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							steering_ctrl_fb_pub_.publish(msg);
						}

						break;
					}

					//
					case 0x98C4D6EF:
					{
						yhs_can_msgs::lf_wheel_fb msg;
						msg.lf_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.lf_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							lf_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//
					case 0x98C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//
					case 0x98C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//
					case 0x98C4D9EF:
					{
						yhs_can_msgs::rf_wheel_fb msg;
						msg.rf_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rf_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							rf_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x98C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_lamp_ctrl = true;	else msg.io_fb_lamp_ctrl = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_unlock = true;	else msg.io_fb_unlock = false;

						if(0x01 & recv_frames_[0].data[1]) msg.io_fb_lower_beam_headlamp = true;	else msg.io_fb_lower_beam_headlamp = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x20 & recv_frames_[0].data[1]) msg.io_fb_clearance_lamp = true;	else msg.io_fb_clearance_lamp = false;

						if(0x40 & recv_frames_[0].data[1]) msg.io_fb_fog_lamp = true;	else msg.io_fb_fog_lamp = false;

						if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x01 & recv_frames_[0].data[3]) msg.io_fb_fl_impact_sensor = true;	else msg.io_fb_fl_impact_sensor = false;

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x04 & recv_frames_[0].data[3]) msg.io_fb_fr_impact_sensor = true;	else msg.io_fb_fr_impact_sensor = false;

						if(0x08 & recv_frames_[0].data[3]) msg.io_fb_rl_impact_sensor = true;	else msg.io_fb_rl_impact_sensor = false;

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

						if(0x20 & recv_frames_[0].data[3]) msg.io_fb_rr_impact_sensor = true;	else msg.io_fb_rr_impact_sensor = false;

						if(0x01 & recv_frames_[0].data[4]) msg.io_fb_fl_drop_sensor = true;	else msg.io_fb_fl_drop_sensor = false;

						if(0x02 & recv_frames_[0].data[4]) msg.io_fb_fm_drop_sensor = true;	else msg.io_fb_fm_drop_sensor = false;

						if(0x04 & recv_frames_[0].data[4]) msg.io_fb_fr_drop_sensor = true;	else msg.io_fb_fr_drop_sensor = false;

						if(0x08 & recv_frames_[0].data[4]) msg.io_fb_rl_drop_sensor = true;	else msg.io_fb_rl_drop_sensor = false;

						if(0x10 & recv_frames_[0].data[4]) msg.io_fb_rm_drop_sensor = true;	else msg.io_fb_rm_drop_sensor = false;

						if(0x20 & recv_frames_[0].data[4]) msg.io_fb_rr_drop_sensor = true;	else msg.io_fb_rr_drop_sensor = false;

						if(0x01 & recv_frames_[0].data[5]) msg.io_fb_estop = true;	else msg.io_fb_estop = false;

						if(0x02 & recv_frames_[0].data[5]) msg.io_fb_joypad_ctrl = true;	else msg.io_fb_joypad_ctrl = false;

						if(0x04 & recv_frames_[0].data[5]) msg.io_fb_charge_state = true;	else msg.io_fb_charge_state = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//
					case 0x98C4DCEF:
					{
						yhs_can_msgs::front_angle_fb msg;
						msg.front_angle_fb_l = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.front_angle_fb_r = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							front_angle_fb_pub_.publish(msg);
							//angular_ = msg.front_angle_fb_l;

							std_msgs::Float32 angular_msg;
							angular_msg.data = msg.front_angle_fb_l;
							angular_pub_.publish(angular_msg);

						}

						break;
					}

					//
					case 0x98C4DDEF:
					{
						yhs_can_msgs::rear_angle_fb msg;
						msg.rear_angle_fb_l = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.rear_angle_fb_r = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							rear_angle_fb_pub_.publish(msg);
						}

						break;
					}

					//bms反馈
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_fb msg;
						msg.bms_fb_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_fb_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_fb_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag反馈
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_fb msg;
						msg.bms_flag_fb_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_fb_single_ov = true;	else msg.bms_flag_fb_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_fb_single_uv = true;	else msg.bms_flag_fb_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_fb_ov = true;	else msg.bms_flag_fb_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_fb_uv = true;	else msg.bms_flag_fb_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_fb_charge_ot = true;	else msg.bms_flag_fb_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_fb_charge_ut = true;	else msg.bms_flag_fb_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_fb_discharge_ot = true;	else msg.bms_flag_fb_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_fb_discharge_ut = true;	else msg.bms_flag_fb_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_fb_charge_oc = true;	else msg.bms_flag_fb_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_fb_discharge_oc = true;	else msg.bms_flag_fb_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_fb_short = true;	else msg.bms_flag_fb_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_fb_ic_error = true;	else msg.bms_flag_fb_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_fb_lock_mos = true;	else msg.bms_flag_fb_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_fb_charge_flag = true;	else msg.bms_flag_fb_charge_flag = false;

						msg.bms_flag_fb_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_fb_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_fb_pub_.publish(msg);
						}

						break;
					}

					//ultrasonic 301~304

					static yhs_can_msgs::ultrasonic ulmsg;

					case 0x301:
					{
						ulmsg.front_left = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.left_front = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;
						break;
					}

					case 0x302:
					{
						ulmsg.right_rear = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.rear_right = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;
						break;
					}

					case 0x303:
					{
						ulmsg.front_right = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.right_front = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;
						break;
					}

					case 0x304:
					{
						ulmsg.rear_left = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.left_rear = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;

						ultrasonic_pub_.publish(ulmsg);

						sensor_msgs::LaserScan scan;
						scan.header.stamp = ros::Time::now();
						scan.header.frame_id = "ultrasonic_laser_link";
						scan.angle_min = -3.14;
						scan.angle_max = 3.14;
						scan.angle_increment = 0.175;
						scan.time_increment = 1 / 10 / 36;
						scan.range_min = 0.0;
						scan.range_max = 20.0;

						scan.ranges.resize(36);
						scan.intensities.resize(36);

						for(unsigned int i = 0; i < 36; ++i)
						{
 							scan.ranges[i] = std::numeric_limits<float>::infinity();
							scan.intensities[i] = 1;
						}
                                                
						scan.ranges[19] = ((double)ulmsg.front_left / 100 > 0.3 || ulmsg.front_left == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.front_left / 100 + 0.45;
						scan.ranges[17] = ((double)ulmsg.front_right / 100 > 0.3 || ulmsg.front_right == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.front_right / 100 + 0.45;
						scan.ranges[33] = ((double)ulmsg.rear_left / 100 > 0.3 || ulmsg.rear_left == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.rear_left / 100+ 0.45;
                        scan.ranges[3] = ((double)ulmsg.rear_right / 100 > 0.3 || ulmsg.rear_right == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.rear_right / 100 + 0.45;
						scan.ranges[12] = ((double)ulmsg.right_front / 100 > 0.3 || ulmsg.right_front == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.right_front / 100 + 0.45;
                        scan.ranges[5] = ((double)ulmsg.right_rear / 100 > 0.3 || ulmsg.right_rear == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.right_rear / 100 + 0.45;
						scan.ranges[30] = ((double)ulmsg.left_rear / 100 > 0.3 || ulmsg.left_rear == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.left_rear / 100 + 0.45;
                        scan.ranges[23] = ((double)ulmsg.left_front / 100 > 0.3 || ulmsg.left_front == 16) ? std::numeric_limits<float>::infinity() : (double)ulmsg.left_front / 100 + 0.45;

						scan_pub_.publish(scan);
						break;
					}

					default:
						break;
				}

			}

					
		}

	}
}


void CanControl::odomPub(float linear,float angular)
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

	//compute odometry in a typical way given the velocities of the robot
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

	odom.pose.covariance[0]  = 0.1;   	// 
	odom.pose.covariance[7]  = 0.1;		// 
	odom.pose.covariance[35] = 0.2;   	//

	odom.pose.covariance[14] = 1e10; 	// set a non-zero covariance on unused    theta x axis
	odom.pose.covariance[21] = 1e10; 	// dimensions (z, pitch and roll); this   theta y  axis
	odom.pose.covariance[28] = 1e10; 	// is a requirement of robot_pose_ekf     theta z axis

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
		cmd_mutex_.lock();
		if(!have_cmd_vel_)
		{
			short linear = 0;
			short angular = 0;
			short slipangle = 0;
			int gear = 6;
			static unsigned char count = 0;

			memset(sendData_u_vel_,0,8);

			sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & gear);
	
			sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((linear & 0x0f) << 4));

			sendData_u_vel_[1] = (linear >> 4) & 0xff;

			sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (linear >> 12));


			sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

			sendData_u_vel_[3] = (angular >> 4) & 0xff;

			sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	
			sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((slipangle & 0x0f) << 4));

			sendData_u_vel_[5] = (slipangle >> 4) & 0xff;


			sendData_u_vel_[6] = sendData_u_vel_[6] | (0x0f & (slipangle >> 12));


			count ++;

			if(count == 16)	count = 0;

			sendData_u_vel_[6] =  sendData_u_vel_[6] | (count << 4);
	

			sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

			send_frames_[0].can_id = 0x98C4D1D0;
			send_frames_[0].can_dlc = 8;

			memcpy(send_frames_[0].data, sendData_u_vel_, 8);

			int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
			if (ret <= 0) 
			{
			  ROS_ERROR("send message failed, error code: %d",ret);
			}
		}
		have_cmd_vel_ = false;
		cmd_mutex_.unlock();
		loop.sleep();
	}
}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	steering_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::steering_ctrl_cmd>("steering_ctrl_cmd", 5, &CanControl::steering_ctrl_cmdCallBack, this);
	front_angle_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::front_angle_free_ctrl_cmd>("front_angle_free_ctrl_cmd", 5, &CanControl::front_angle_free_ctrl_cmdCallBack, this);
	front_velocity_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::front_velocity_free_ctrl_cmd>("front_velocity_free_ctrl_cmd", 5, &CanControl::front_velocity_free_ctrl_cmdCallBack, this);
	rear_angle_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::rear_angle_free_ctrl_cmd>("rear_angle_free_ctrl_cmd", 5, &CanControl::rear_angle_free_ctrl_cmdCallBack, this);
	rear_velocity_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::rear_velocity_free_ctrl_cmd>("rear_velocity_free_ctrl_cmd", 5, &CanControl::rear_velocity_free_ctrl_cmdCallBack, this);

	cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &CanControl::cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);
	lf_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lf_wheel_fb>("lf_wheel_fb",5);
	rf_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rf_wheel_fb>("rf_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	bms_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_fb>("bms_fb",5);
	bms_flag_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_fb>("bms_flag_fb",5);
	steering_ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::steering_ctrl_fb>("steering_ctrl_fb",5);
	front_angle_fb_pub_ = nh_.advertise<yhs_can_msgs::front_angle_fb>("front_angle_fb",5);
	rear_angle_fb_pub_ = nh_.advertise<yhs_can_msgs::rear_angle_fb>("rear_angle_fb",5);

	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

	angular_pub_ = nh_.advertise<std_msgs::Float32>("angular", 5);
	linear_pub_ = nh_.advertise<std_msgs::Float32>("linear", 5);

	ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic",5);
	scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan3", 5);

	//打开设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}


	struct ifreq ifr;
	
	std::string can_name("can0");

	strcpy(ifr.ifr_name,can_name.c_str());

	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);


    // bind socket to network interface
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	ros::spin();
	
	close(dev_handler_);
}

}



//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
