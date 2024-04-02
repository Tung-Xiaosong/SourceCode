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
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "yhs_msgs/DgtChassisInfoFb.h" 
#include "yhs_msgs/FwChassisInfoFb.h" 
#include "yhs_msgs/FwCtrlCmd.h"
#include "yhs_msgs/MkChassisInfoFb.h" 
#include "yhs_msgs/FrChassisInfoFb.h" 

#include "yhs_msgs/FwCtrlCmd.h"
#include "yhs_msgs/Ultrasonic.h"
#include "yhs_msgs/PathIsValid.h"
#include "yhs_msgs/Recharge.h"
#include "yhs_msgs/DisRecharge.h"
#include <std_msgs/UInt8.h>//dxs

using namespace Eigen;

//机器人实时坐标和方向
struct RobotPose
{
  double x;
  double y;
  double yaw;  
};

//充电坐标和方向
struct RechargePose
{
  double x;
  double y;
  double yaw;  
};

namespace recharge_controller_ns
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  class RechargeController
  {
  public:
    RechargeController();
    ~RechargeController();

  private:
    //机器人实时位置
    RobotPose robot_pose_;
    
    //充电位置
    RechargePose recharge_pose_;
    
    //底盘类型
    std::string chassis_type_;

    // 临时坐标点的距离
    double recharge_dist_;
    
    //脱桩距离
    double dis_recharge_dist_;

    // //回充失败重试次数
    // int recharge_repeat_;

    //后退时遇到障碍物累积时长计数
    int time_obs_taltol_;
    ros::Time back_has_obs_taltol_time_;

    //前进脱离充电桩时遇到障碍物累积时长计数
    int front_time_obs_taltol_;
    ros::Time front_has_obs_taltol_time_;

    //充电反馈成功后仍然后退时间
    int time_chage_done_taltol_;

    //控制频率
    int control_frequency_;

    //回充超时时间
    double recharge_time_out_;
    
    //线速度
    double linear_vel_;
    
    //角速度
    double angular_vel_;
    
    //横移角度
    double slipangle_;

    //角度误差
    double yaw_delta_;

    //距离误差
    double dist_delta_;

    //底盘角度反馈
    double angular_fb_;
    
    //机器人与充电点的距离,大于此值才会检测障碍物
    double check_obs_dist_in_back_;
    
    //机器人位置到充电路径的最短距离，做偏离异常检测
    double shortest_dist_y_;
    
    //充电点在机器人前面的距离，做偏离异常检测
    double shortest_dist_x_;
    
    //机器人当前方向与充电点方向差，做偏离异常检测
    double yaw_diff_err_;

    // 控制指令发布器
    ros::Publisher cmd_vel_pub_;
    
    // 回充是否成功标志
    bool is_charge_done_;
    
    //充电失败
    bool is_charge_failed_;

    //bms反馈充电成功标志
    bool bms_charge_done_;

    //取消标志位
    bool cancel_;

    // 回充点坐标更新回调函数
    bool rechargeCallback(yhs_msgs::Recharge::Request &req, yhs_msgs::Recharge::Response &res);
    bool disRechargeCallback(yhs_msgs::DisRecharge::Request &req, yhs_msgs::DisRecharge::Response &res);

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
    
    //获取机器人实时位置
    bool getRobotPose();
    
    //坐标转换
    bool transformPose(
    const std::string frame,
    const geometry_msgs::PoseStamped & in_pose,
    geometry_msgs::PoseStamped & out_pose);
    
    // 发布临时回充导航点
    bool goToTempPoint(const geometry_msgs::PoseStamped temp_point);

    //复位
    void reset();

    // //底盘数据回调函数
    // void chassisInfoCallback(const yhs_msgs::FwChassisInfoFb::ConstPtr& chassis_info_msg);
    
    //DGT底盘
    void DgtChassisCB(const yhs_msgs::DgtChassisInfoFb::ConstPtr& msg);
    //MK底盘
    void MkChassisCB(const yhs_msgs::MkChassisInfoFb::ConstPtr& msg);
    //FR底盘
    void FrChassisCB(const yhs_msgs::FrChassisInfoFb::ConstPtr& msg);
    //四转四驱底盘
    void FwChassisCB(const yhs_msgs::FwChassisInfoFb::ConstPtr& msg);

    //取消回调函数
    void rechargeCancelCallback(const std_msgs::Bool::ConstPtr& cancel_msg);
    
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
    
    //底盘信息订阅
    ros::Subscriber dgt_chassis_info_sub_;
    ros::Subscriber fw_chassis_info_sub_;
    ros::Subscriber fr_chassis_info_sub_;
    ros::Subscriber mk_chassis_info_sub_;

    ros::Subscriber angular_fb_sub_;
    ros::Publisher ctrl_cmd_pub_;
    ros::Publisher nav_type_pub_;//dxs
    
    //取消订阅器
    ros::Subscriber cancel_sub_;
    
    //Service
    ros::ServiceClient client_;
    ros::ServiceServer recharge_service_;
    ros::ServiceServer dis_recharge_service_;
  };
} // namespace recharge_controller_ns

#endif // RECHARGE_CONTROLLER_H
