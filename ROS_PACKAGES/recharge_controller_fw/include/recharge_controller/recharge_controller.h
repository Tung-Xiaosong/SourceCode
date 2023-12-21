#ifndef RECHARGE_CONTROLLER_H
#define RECHARGE_CONTROLLER_H

#include <cmath>
#include <thread>          
#include <chrono>         
#include <mutex>
#include <tf/transform_listener.h>
#include <condition_variable>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "yhs_can_msgs/ChassisInfoFb.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/Ultrasonic.h"
#include "yhs_can_msgs/PathIsValid.h"
#include "yhs_can_msgs/Recharge.h"
#include "yhs_can_msgs/DisRecharge.h"

#include <time.h>//dxscpp

using namespace Eigen;

namespace recharge_controller_ns
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  class RechargeController
  {
  public:
    RechargeController();
    ~RechargeController();
    void start();
    void stop();

  private:
    // 机器人当前位置和回充点的坐标
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double recharge_x_;
    double recharge_y_;
    double recharge_yaw_;
    
    //临时坐标点
    geometry_msgs::PoseStamped temp_point_;

    // 临时坐标点的距离
    double recharge_dist_;

    //回充失败重试次数
    int recharge_repeat_;

    //后退时遇到障碍物累积时长计数
    int time_obs_taltol_;

    int charge_time_out_;//dxscpp
    
    //前进脱离充电桩时遇到障碍物累积时长计数
    int front_time_obs_taltol_;

    //充电反馈成功后仍然后退时间
    int time_chage_done_taltol_;

    //控制频率
    int control_frequency_;

    //回充超时时间
    double time_out_;

    //角度误差
    double yaw_delta_;

    //距离误差
    double dist_delta_;

    //底盘角度反馈
    double angular_fb_;

    // 控制指令发布器
    ros::Publisher cmd_vel_pub_;
    
    // 回充是否成功标志
    bool is_charge_done_;
    
    //充电失败
    bool is_charge_failed_;

    //bms反馈充电成功标志
    bool bms_charge_done_;

    bool flag;
    //bool static reset_last_time ;//dxscpp
    //bool reset_last_time;
    // 回充点坐标更新回调函数
    bool rechargeCallback(yhs_can_msgs::Recharge::Request &req, yhs_can_msgs::Recharge::Response &res);
    bool disRechargeCallback(yhs_can_msgs::DisRecharge::Request &req, yhs_can_msgs::DisRecharge::Response &res);

    // 机器人位置更新回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    //角度反馈回调函数
    void angularCallback(const std_msgs::Float32::ConstPtr& angular_msg);

    // 控制指令发布函数
    void publishCmdVel(const double linear_vel, const double angular_vel);
    void publishCtrlCmd(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle);

    //计算最短路径
    inline double distanceToLine(Vector2d A, Vector2d dir, Vector2d B);

    // 回充控制函数
    void rechargeControl();
    bool execute();
    
    // 发布临时回充导航点
    bool goToTempPoint();

    //复位
    void reset();

    //bms反馈
    void chassisInfoCallback(const yhs_can_msgs::io_fb::ConstPtr& chassis_info_msg);//dxs
    
    // 线程相关的变量和函数
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    bool continue_;
    bool is_running_;
    void threadFunction();
    
    //tf
    tf::TransformListener tf_listener_;

    // ROS节点句柄
    ros::NodeHandle nh_;

    // ROS订阅器
    ros::Subscriber odom_sub_;
    ros::Subscriber recharge_sub_;
    ros::Subscriber bms_flag_sub_;

    ros::Subscriber angular_fb_sub_;

    ros::Publisher ctrl_cmd_pub_;
    
    //Service
    ros::ServiceClient client_;
    ros::ServiceServer recharge_service_;
    ros::ServiceServer dis_recharge_service_;

  };
} // namespace recharge_controller_ns

#endif // RECHARGE_CONTROLLER_H
