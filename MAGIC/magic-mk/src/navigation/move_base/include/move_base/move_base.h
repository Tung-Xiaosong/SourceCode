/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <mutex>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

//#include <move_base/SetCostmaps.h>
#include <move_base/PathIsValid.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"
#include "yhs_msgs/DynamicParams.h"
#include "move_base/teb_local_planner_ros.h"
#include "move_base/regulated_pure_pursuit_controller.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      bool checkPathIsValidService(yhs_msgs::PathIsValid::Request &req, yhs_msgs::PathIsValid::Response &resp);

      bool clearCostmaps();

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_,contorl_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      std::vector<std::string> recovery_behavior_names_;
      unsigned int recovery_index_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;

      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_,check_path_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      std::vector<geometry_msgs::PoseStamped>* last_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;
      boost::condition_variable_any planner_cond_;
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;

      //目标点订阅，手绘路径订阅 录制路径订阅 aap上设置参数订阅  导航类型 底盘速度和角度
      ros::Subscriber goal_sub_;
      ros::Subscriber path_sub_;
      ros::Subscriber dynamic_params_sub_;
      ros::Subscriber nav_type_sub_;
      ros::Subscriber in_slope_region_sub_;
      ros::Subscriber in_slow_region_sub_;
      ros::Subscriber in_narrow_region_sub_;
      ros::Subscriber angular_sub_;
      ros::Subscriber linear_sub_;

      nav_msgs::Path record_path_;
      std::vector<geometry_msgs::PoseStamped>* record_plan_;

      boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;

      ros::Publisher global_path_;

      //app上动态参数设置消息变量
      yhs_msgs::DynamicParams dynamic_params_;

      //导航类型消息变量
      uint8_t nav_type_;

      //底盘类型
      uint8_t car_type_;

      //四转四驱底盘轮子速度
      float chassis_angular_;
      //四转四驱底盘角度
      float chassis_linear_;

      //变量锁
      std::mutex mutex_;

      //手绘路径或者录制路径距离机器绕障时最近坐标点索引
      int nearest_index_;

      //机器人距离路径最近点的距离
      double distance_to_path_nearest_point_;

      //机器人距离路径最近点的距离大于0.5m置起位,小于0.2m才会被清0
      bool distance_to_path_nearest_point_big_than_zero_dot_five_;

      bool is_replan_;
      bool has_obs_in_path_;
      bool is_free_;

      //手绘或者录制路径是否处于绕障中
      bool is_in_avoid_obs_;

      //减速区域判断变量
      bool in_slope_region_;
      bool in_slow_region_;

      //窄道区域判断变量
      bool in_narrow_region_;

      //手绘路径 录制路径回调函数
      void receive_path_callback(const nav_msgs::Path::ConstPtr &msg);
      //app上动态配置函数
      void dynamic_params_callback(const yhs_msgs::DynamicParams::ConstPtr &msg);
      //导航类型回调函数
      void nav_type_callback(const std_msgs::UInt8::ConstPtr &msg);

      //斜坡位置回调函数
      void is_in_slope_region_callback(const std_msgs::Bool::ConstPtr &msg);
      //减速区域回调函数
      void is_in_slow_region_callback(const std_msgs::Bool::ConstPtr &msg);
      //窄道区域回调函数
      void is_in_narrow_region_callback(const std_msgs::Bool::ConstPtr &msg);

      //四转四驱底盘轮子角度回调函数
      void angularCB(const std_msgs::Float32::ConstPtr &msg);

      //四转四驱底盘轮子速度回调函数
      void linearCB(const std_msgs::Float32::ConstPtr &msg);

      //判断点周围是否有障碍物
      bool checkPointInObs(const geometry_msgs::Pose point);

      inline double norm2(const geometry_msgs::PoseStamped &from, geometry_msgs::PoseStamped &to) {
            double delta_x = from.pose.position.x - to.pose.position.x;
            double delta_y = from.pose.position.y - to.pose.position.y;
            return delta_x * delta_x + delta_y * delta_y;
      }

      
      inline int min_distance_index(const geometry_msgs::PoseStamped &from, nav_msgs::Path &path) {
      int index = -1;
      double min_value = std::numeric_limits<double>::max();
      double dis;
      for (size_t i = 0; i < path.poses.size(); i++) {
          dis = norm2(from, path.poses.at(i));
          if (dis < min_value) {
             min_value = dis;
             index = static_cast<int>(i);
          }
      }
            return index;
      }


  };
};
#endif

