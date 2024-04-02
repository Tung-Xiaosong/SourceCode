
#ifndef YUJIN_OCS_VELOCITY_SMOOTHER_HPP_
#define YUJIN_OCS_VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <geometry_msgs/TwistStamped.h>


#include "yhs_msgs/DgtChassisInfoFb.h" 
#include "yhs_msgs/FwChassisInfoFb.h" 
#include "yhs_msgs/MkChassisInfoFb.h" 
#include "yhs_msgs/FrChassisInfoFb.h" 


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother
{
public:
  VelocitySmoother(const std::string &name);

  ~VelocitySmoother()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

  bool init(ros::NodeHandle& nh);
  void spin();
  void shutdown() { shutdown_req = true; };
  std::mutex locker;

private:
  enum RobotFeedbackType
  {
    NONE,
    ODOMETRY,
    COMMANDS
  } robot_feedback;  /**< What source to use as robot velocity feedback */

  std::string name;
  bool quiet;        /**< Quieten some warnings that are unavoidable because of velocity multiplexing. **/
  double speed_lim_v, accel_lim_v, decel_lim_v;
  double speed_lim_w, accel_lim_w, decel_lim_w;
  double decel_factor;

  double frequency;

  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist  current_vel;
  geometry_msgs::Twist   target_vel;

  //遥控模式标志位
  bool                 is_joy_control;

  bool                 shutdown_req; /**< Shutdown requested by nodelet; kill worker thread */
  bool                 input_active;
  double                cb_avg_time;
  ros::Time            last_cb_time;
  std::vector<double> period_record; /**< Historic of latest periods between velocity commands */
  unsigned int             pr_next; /**< Next position to fill in the periods record buffer */

  //底盘信息订阅
  ros::Subscriber dgt_chassis_info_sub;
  ros::Subscriber fw_chassis_info_sub;
  ros::Subscriber fr_chassis_info_sub;
  ros::Subscriber mk_chassis_info_sub;
  
  //底盘底盘类型
  std::string chassis_type;

  ros::Subscriber odometry_sub;    /**< Current velocity from odometry */
  ros::Subscriber current_vel_sub; /**< Current velocity from commands sent to the robot, not necessarily by this node */
  ros::Subscriber raw_in_vel_sub;  /**< Incoming raw velocity commands */
  ros::Publisher  smooth_vel_pub;  /**< Outgoing smoothed velocity commands */

  void velocityCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);

  //DGT差速底盘
  void DgtChassisCB(const yhs_msgs::DgtChassisInfoFb::ConstPtr& msg);
  
  //MK底盘
  void MkChassisCB(const yhs_msgs::MkChassisInfoFb::ConstPtr& msg);
  
  //FW底盘
  void FwChassisCB(const yhs_msgs::FwChassisInfoFb::ConstPtr& msg);
  
  //FR底盘
  void FrChassisCB(const yhs_msgs::FrChassisInfoFb::ConstPtr& msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> values) {
    // Return the median element of an doubles vector
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  };

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<yocs_velocity_smoother::paramsConfig> *             dynamic_reconfigure_server;
  dynamic_reconfigure::Server<yocs_velocity_smoother::paramsConfig>::CallbackType dynamic_reconfigure_callback;
  void reconfigCB(yocs_velocity_smoother::paramsConfig &config, uint32_t unused_level);
  
};

} // yocs_namespace velocity_smoother

#endif /* YUJIN_OCS_VELOCITY_SMOOTHER_HPP_ */
