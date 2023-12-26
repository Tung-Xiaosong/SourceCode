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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

namespace move_base {

  MoveBase::MoveBase(tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), 
    c_freq_change_(false), new_global_plan_(false),is_replan_(false),
    has_obs_in_path_(false),is_free_(true),nav_type_(0),
    in_slope_region_(false),in_slow_region_(false),in_narrow_region_(false),
    chassis_angular_(0.0),chassis_linear_(0.0) {

    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //规划失败后是否重规划
    private_nh.param("yhs_robot_params/strategy/is_replan", is_replan_, false);

    //手绘和录制路径是否绕障碍物
    bool graph_void_obstacle,record_void_obstacle;
    private_nh.param("yhs_robot_params/avoid_obstacle/graph_follow", graph_void_obstacle, false);
    private_nh.param("yhs_robot_params/avoid_obstacle/record_follow", record_void_obstacle, false);
    dynamic_params_.graph_void_obstacle = graph_void_obstacle;
    dynamic_params_.record_void_obstacle = record_void_obstacle;
    
    double goal_follow_max_speed,graph_follow_max_speed,record_follow_max_speed;
    //自由导航最大速度
    private_nh.param("yhs_robot_params/max_speed/goal_follow", goal_follow_max_speed, 0.6);
    //手绘路径最大速度
    private_nh.param("yhs_robot_params/max_speed/graph_follow", graph_follow_max_speed, 0.6);
    //录制路径最大速度
    private_nh.param("yhs_robot_params/max_speed/record_follow", record_follow_max_speed, 0.6);

    dynamic_params_.goal_follow_max_speed = goal_follow_max_speed;
    dynamic_params_.graph_follow_max_speed = graph_follow_max_speed;
    dynamic_params_.record_follow_max_speed =record_follow_max_speed;

    int car_type;
    //底盘类型
    private_nh.param("yhs_robot_params/chassis_size/type", car_type, 1);

    car_type_ = car_type;

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    record_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for comanding the base
    vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //录制路径 手绘路径
    path_sub_ = nh.subscribe<nav_msgs::Path>("recorded_path", 10, &MoveBase::receive_path_callback, this);

    //app上参数订阅
    dynamic_params_sub_ = nh.subscribe<yhs_msgs::DynamicParams>("dynamic_params", 10, &MoveBase::dynamic_params_callback, this);

    //导航类型订阅
    nav_type_sub_ = nh.subscribe<std_msgs::UInt8>("nav_type", 10, &MoveBase::nav_type_callback, this);

    //斜坡位置订阅
    in_slope_region_sub_ = nh.subscribe<std_msgs::Bool>("is_in_slope_region", 10, &MoveBase::is_in_slope_region_callback, this);

    //减速位置订阅
    in_slow_region_sub_ = nh.subscribe<std_msgs::Bool>("is_in_slow_region", 10, &MoveBase::is_in_slow_region_callback, this);

    //窄道位置订阅
    in_narrow_region_sub_ = nh.subscribe<std_msgs::Bool>("is_in_narrow_region", 10, &MoveBase::is_in_narrow_region_callback, this);

    //订阅四转四驱底盘轮子角度和速度
    angular_sub_ = nh.subscribe<std_msgs::Float32>("angular", 1,&MoveBase::angularCB, this);
    linear_sub_ = nh.subscribe<std_msgs::Float32>("linear", 1,&MoveBase::linearCB, this);


    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, false);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

//control start

    std::string control("regulated_pure_pursuit_controller/RegulatedPurePursuitController");
    //create a local planner
    try {
      contorl_ = blp_loader_.createInstance(control);
      ROS_INFO("Created control %s", control.c_str());
      contorl_->initialize(blp_loader_.getName(control), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

//control end

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*(planner_costmap_ros_->getLayeredCostmap()->getCostmap()));

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

//    set_costmaps_srv_ = private_nh.advertiseService("set_costmaps", &MoveBase::setCostmapsService, this);

    check_path_srv_ = private_nh.advertiseService("check_path", &MoveBase::checkPathIsValidService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);

  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose);

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

  bool MoveBase::checkPathIsValidService(yhs_msgs::PathIsValid::Request &req, yhs_msgs::PathIsValid::Response &resp){

    resp.result = 1;
    for(int i = 0; i < req.checked_path.poses.size(); i ++)
    {
      // tf::Quaternion RQ2;
      // double roll,pitch,yaw;
      // tf::quaternionMsgToTF(req.checked_path.poses[i].pose.orientation,RQ2);
      // tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
      // if( costmap_model_.get()->footprintCost(req.checked_path.poses[i].pose.position.x, req.checked_path.poses[i].pose.position.y, yaw, planner_costmap_ros_->getRobotFootprint(),
      // planner_costmap_ros_->getLayeredCostmap()->getInscribedRadius(), planner_costmap_ros_->getLayeredCostmap()->getCircumscribedRadius()) < 0 )
      // {
      //   resp.result = 0;
      //   break;
      // }
      if(checkPointInObs(req.checked_path.poses[i].pose))
      {
        ROS_WARN("checkPathIsValidService: invalid point (%.3f,%.3f) ",req.checked_path.poses[i].pose.position.x,\
        req.checked_path.poses[i].pose.position.y);
        resp.result = 0;
        break;
      }
    }
    return true;
  }

  bool MoveBase::clearCostmaps(){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

  //手绘 录制路径回调函数
  void MoveBase::receive_path_callback(const nav_msgs::Path::ConstPtr &msg) {

    ROS_INFO("Recive grah or record path.");

    record_path_ = *msg;

    record_plan_->clear();
    for(int i = 0 ; i < record_path_.poses.size(); i ++)
    {
      geometry_msgs::PoseStamped org = record_path_.poses.at(i);
      record_plan_->push_back(org);
    }
  }

  //app上动态参数回调函数
  void MoveBase::dynamic_params_callback(const yhs_msgs::DynamicParams::ConstPtr &msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    dynamic_params_ = *msg;

    ROS_INFO("Recive config.");
  }

  //导航类型回调函数
  void MoveBase::nav_type_callback(const std_msgs::UInt8::ConstPtr &msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    nav_type_ = msg->data;
  }

  //斜坡区域回调函数
  void MoveBase::is_in_slope_region_callback(const std_msgs::Bool::ConstPtr &msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    in_slope_region_ = msg->data;
  }

  //减速区域回调函数
  void MoveBase::is_in_slow_region_callback(const std_msgs::Bool::ConstPtr &msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    in_slow_region_ = msg->data;
  }

  //窄道区域回调函数
  void MoveBase::is_in_narrow_region_callback(const std_msgs::Bool::ConstPtr &msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    in_narrow_region_ = msg->data;
  }

  //四转四驱底盘轮子角度
  void MoveBase::angularCB(const std_msgs::Float32::ConstPtr &msg)
  {
    chassis_angular_ = msg->data;
  }

  //四转四驱底盘轮子速度
  void MoveBase::linearCB(const std_msgs::Float32::ConstPtr &msg)
  {
    chassis_linear_ = msg->data;
  }

  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        tf::Stamped<tf::Pose> global_pose;
        if(!planner_costmap_ros_->getRobotPose(global_pose)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        tf::poseStampedTFToMsg(global_pose, start);
    }
    else
    {
        start = req.start;
    }

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);


    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;
    delete record_plan_;

    planner_.reset();
    tc_.reset();
    contorl_.reset();
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) || q.w == 0){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  bool MoveBase::checkPointInObs(const geometry_msgs::Pose point)
  {
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(point.orientation,RQ2);
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);

    //非窄通道，录制路径和手绘路径模式下，将底盘左右尺寸增加20厘米，安全检测
    std::vector<geometry_msgs::Point> temp_foot_points;
    temp_foot_points = planner_costmap_ros_->getRobotFootprint();
    if( !in_narrow_region_ && ( (nav_type_ == 1 && !dynamic_params_.graph_void_obstacle) || (nav_type_ == 2 && !dynamic_params_.record_void_obstacle)) )
    {
      temp_foot_points[0].y += 0.2;
      temp_foot_points[1].y -= 0.2;
      temp_foot_points[2].y += 0.2;
      temp_foot_points[3].y -= 0.2;
    }

    if( costmap_model_.get()->footprintCost(point.position.x, point.position.y, yaw, temp_foot_points,
    planner_costmap_ros_->getLayeredCostmap()->getInscribedRadius(), planner_costmap_ros_->getLayeredCostmap()->getCircumscribedRadius()) < 0 )
    {
      return true;
    }    
    return false;
  }

  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();

//      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
      
      has_obs_in_path_ = false;
      is_free_ = true;

      int start_index = 0;
      nearest_index_ = -1;
      distance_to_path_nearest_point_ = 0;
      is_in_avoid_obs_ = false;

      //录制或者手绘路径
      if(nav_type_ != 0)
      {
        if( record_path_.poses.size() > 0 )
        {
          is_free_ = false;

          tf::Stamped<tf::Pose> global_pose;
          if(!planner_costmap_ros_->getRobotPose(global_pose)) 
          {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
          } 

          geometry_msgs::PoseStamped start;
          tf::poseStampedTFToMsg(global_pose, start);

          start_index = min_distance_index(start, record_path_);

          // 创建一个用于存储转换后结果的PoseStamped消息
          geometry_msgs::PoseStamped transformed_point;
          double start_index_point_angular = 0;
          static bool begin_add_start_index = false;
          try
          {
            // 设置输入点的header信息
            transformed_point.header = record_path_.poses[start_index].header;
            record_path_.poses[start_index].header.stamp = ros::Time(0);

            // 进行坐标变换，将输入点从map坐标系转换到base_link坐标系
            tf_.transformPose("base_link", record_path_.poses[start_index], transformed_point);

            start_index_point_angular = std::atan2(transformed_point.pose.position.y, transformed_point.pose.position.x);

          }
          catch(tf::TransformException& ex)
          {
            ROS_ERROR("Move_base failed to transform map point to base_link point!: %s", ex.what());
          }

          //机器人距离路径最近点的距离
          distance_to_path_nearest_point_ = std::hypot(transformed_point.pose.position.x, transformed_point.pose.position.y);

          //偏离路径后速度后，速度和前视距离的限制
          if(distance_to_path_nearest_point_ >= 0.23)
            is_in_avoid_obs_ = true;

          if(distance_to_path_nearest_point_ <= 0.15)
            distance_to_path_nearest_point_big_than_zero_dot_five_ = false;

          if( ( fabs(start_index_point_angular) > 0.5 && distance_to_path_nearest_point_ > 0.5 ) || begin_add_start_index )
          {
            begin_add_start_index = true;
            start_index += 8;
          }

          //重置
          if(std::hypot(transformed_point.pose.position.x, transformed_point.pose.position.y) < 0.15)
          {
            begin_add_start_index = false;
          }

          //保持最近点在机器人前方
          start_index += 1;
          if(start_index >= record_path_.poses.size()) start_index = record_path_.poses.size() - 1;

          nearest_index_ = start_index;

          double length = 0;
          for(int i = start_index + 1; i < record_path_.poses.size(); i ++)
          {
            //计算长度
            length += hypot(record_path_.poses[i].pose.position.x - record_path_.poses[i-1].pose.position.x,\
            record_path_.poses[i].pose.position.y - record_path_.poses[i-1].pose.position.y);

            //2米
            if(length > 2.0) break;

            // tf::Quaternion RQ2;
            // double roll,pitch,yaw;
            // tf::quaternionMsgToTF(record_path_.poses[i].pose.orientation,RQ2);
            // tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
            // if( costmap_model_.get()->footprintCost(record_path_.poses[i].pose.position.x, record_path_.poses[i].pose.position.y, yaw, planner_costmap_ros_->getRobotFootprint(),
            // planner_costmap_ros_->getLayeredCostmap()->getInscribedRadius(), planner_costmap_ros_->getLayeredCostmap()->getCircumscribedRadius()) < 0 )
            // {
            //   has_obs_in_path_ = true;
            //   break;
            // }
            if(checkPointInObs(record_path_.poses[i].pose))
            { 
              has_obs_in_path_ = true;
              break;
            }

          }

          for(int i = start_index; i < record_plan_->size(); i ++)
          {
            planner_plan_->push_back(record_plan_->at(i));
          } 

          //手绘路径,路径点裁剪
          if(nav_type_ == 1)
          {
            std::vector<geometry_msgs::PoseStamped> temp;
            temp.push_back(planner_plan_->front());

            for(int i = 2; i < planner_plan_->size() - 1; i += 2)
            {
              temp.push_back(planner_plan_->at(i));
            }

            temp.push_back(planner_plan_->back());

            planner_plan_->clear();
            for(int i = 0; i < temp.size(); i ++)
            {
              planner_plan_->push_back(temp[i]);
            }
          }

          //将机器人当前位置插入到路径最前面
          planner_plan_->insert(planner_plan_->begin(),start);
        }
        else
        {
        }

      }
    
      bool gotPlan = true;

      //自由导航，默认为false
//      gotPlan = is_free_ ? false : true;

      //自由导航
      if(is_free_)
      {
        gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
      }
      //录制或者手绘路径，绕障
      else if((has_obs_in_path_ && dynamic_params_.graph_void_obstacle) || (has_obs_in_path_ && dynamic_params_.record_void_obstacle))
      {
        planner_plan_->clear();

        double length = 0;

        //如果前方有障碍物，则取路径上2米远处的一个点作为临时目标点规划路径
        int i = start_index + 1;
        for(; i < record_path_.poses.size(); i ++)
        {
          length += hypot(record_path_.poses[i].pose.position.x - record_path_.poses[i-1].pose.position.x,\
          record_path_.poses[i].pose.position.y - record_path_.poses[i-1].pose.position.y);

          if(length > 2.0) 
          {
            temp_goal = record_path_.poses[i];
            std::vector<geometry_msgs::Point> temp_foot_points;
            //判断该点有没有障碍物，有则向后移
            bool in_obs = checkPointInObs(record_path_.poses[i].pose);
            if( costmap_model_.get()->footprintCost(record_path_.poses[i].pose.position,temp_foot_points, 0, 0) >= 0 && !in_obs)
              break;
          }
        }

        gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
        is_in_avoid_obs_ = true;
      }
      //已经处于绕障情况下，而且距离机器人最近的路径点上没有障碍物
      else if(distance_to_path_nearest_point_ > 0.3 || distance_to_path_nearest_point_big_than_zero_dot_five_)
      {
//        ROS_WARN("distance_to_path_nearest_point_   %f    %d",distance_to_path_nearest_point_,distance_to_path_nearest_point_big_than_zero_dot_five_);
        distance_to_path_nearest_point_big_than_zero_dot_five_ = true;
        if( nearest_index_ + 8 >= record_path_.poses.size() )
        {
          temp_goal = record_path_.poses[record_path_.poses.size() -1];
        }
        else
        {
          temp_goal = record_path_.poses[nearest_index_ + 8];
        }
        gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
        is_in_avoid_obs_ = true;
      }
      else
      //录制或者手绘路径，停障
      {
        if(has_obs_in_path_)
          gotPlan = false;
      }

      if(gotPlan){
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_ == PLANNING || state_ == CONTROLLING) {
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        lock.lock();

        state_ = PLANNING;
//        runPlanner_ = true;
//        planner_cond_.notify_one();
        publishZeroVelocity();

        ROS_WARN("front has obs or not get global plan, stop!");

        planning_retries_++;
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end && ( is_free_ || (nav_type_ == 1 && dynamic_params_.graph_void_obstacle) ||\
           (nav_type_ == 2 && dynamic_params_.record_void_obstacle) ) ) ) {
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    auto c = dynamic_cast<teb_local_planner::TebLocalPlannerROS*>(tc_.get());
    if(c)
    {
      c->setNewGoal();
    }

    auto contorl_private = dynamic_cast<regulated_pure_pursuit_controller::RegulatedPurePursuitController*>(contorl_.get());
    if(contorl_private)
    {
      contorl_private->setNewGoal();
    }

    ros::NodeHandle n;
    while(n.ok())
    {
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){

            resetState();
            as_->setPreempted(move_base_msgs::MoveBaseResult(), "goal was canceled");
            //取消
            ROS_INFO("Goal cancel!");
//            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted(move_base_msgs::MoveBaseResult(), "goal was canceled");

          //取消
          ROS_INFO("Goal cancel!");
//          record_path_.poses.clear();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //取消
    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
//    record_path_.poses.clear();
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::TwistStamped cmd_vel_pub;

    //update feedback to correspond to our curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index 
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if( new_global_plan_ /*|| ((has_obs_in_path_ && !dynamic_params_.graph_void_obstacle) || \
    (has_obs_in_path_ && !dynamic_params_.record_void_obstacle))*/ ){
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      if(!tc_->setPlan(*controller_plan_) || !contorl_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        //异常
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
//        record_path_.poses.clear();
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        if(contorl_->isGoalReached()/*tc_->isGoalReached()*/){
          tc_->isGoalReached();
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
//          record_path_.poses.clear();
          return true;
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        
        {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        
        if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
//          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;


          //轨迹跟踪控制
          std::vector<geometry_msgs::PoseStamped> control_plan;

          auto c = dynamic_cast<teb_local_planner::TebLocalPlannerROS*>(tc_.get());
          if(c)
          {
            c->getPath(control_plan);
            c->setFlag(in_narrow_region_);
          }

          //将teb规划出来的路径转化为局部路径
          //bool need_set_teb_plan = false;
          // for(int i = 0; i < control_plan.size(); i++) 
          // {
          //   try
          //   {
          //     // 创建一个用于存储转换后结果的PoseStamped消息
          //     geometry_msgs::PoseStamped transformed_point;

          //     // 设置输入点的header信息
          //     transformed_point.header = control_plan[i].header;

          //     // 进行坐标变换，将输入点从map坐标系转换到base_link坐标系
          //     tf_.transformPose("base_link", control_plan[i], transformed_point);

          //     if( transformed_point.pose.position.x < -0.06 && nearest_index_ > 0)
          //     {
          //       record_path_.poses[nearest_index_].header = control_plan[i].header;
          //       tf_.transformPose("base_link", record_path_.poses[nearest_index_], transformed_point);

          //       if( std::hypot(transformed_point.pose.position.x,transformed_point.pose.position.y) > 0.5)
          //       {
          //         need_set_teb_plan = true;
          //         break;
          //       }
          //     }

          //     if(i > 10) break;
          //   }
          //   catch(tf::TransformException& ex)
          //   {
          //     ROS_ERROR("Move_base failed to transform path to base_link: %s", ex.what());
          //   }
          // }

          //自由导航 录制路径 手绘路径
          if(!control_plan.empty())
          {
            control_plan.pop_back();
            control_plan.push_back(controller_plan_->back());

            if(is_free_ || (has_obs_in_path_ && dynamic_params_.graph_void_obstacle) || (has_obs_in_path_ && dynamic_params_.record_void_obstacle) \
            || distance_to_path_nearest_point_big_than_zero_dot_five_)
              contorl_->setPlan(control_plan);
          }

          std::lock_guard<std::mutex> lock(mutex_);
          //设置最大速度 导航类型 斜坡减速区域
          auto contorl_private = dynamic_cast<regulated_pure_pursuit_controller::RegulatedPurePursuitController*>(contorl_.get());
          if(contorl_private)
          {
            contorl_private->setMaxSpeed(dynamic_params_.goal_follow_max_speed,dynamic_params_.graph_follow_max_speed,\
            dynamic_params_.record_follow_max_speed);

            contorl_private->setRegion(in_slope_region_,in_slow_region_,in_narrow_region_);

            contorl_private->setNavType(nav_type_,car_type_,chassis_angular_,chassis_linear_,is_in_avoid_obs_);
          }

          if(!contorl_->computeVelocityCommands(cmd_vel))
          {
            cmd_vel.linear.x = 0; 
            cmd_vel.angular.z = 0;
          }
          
          cmd_vel_pub.header.stamp = ros::Time::now();
          cmd_vel_pub.twist = cmd_vel;
          vel_pub_.publish(cmd_vel_pub);
 
        }
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          recovery_status_pub_.publish(msg);

          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
//            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
//            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
//            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          //等待5s后，选择继续等待或者直接复位
          if(!is_replan_)
          {
//            record_path_.poses.clear();
            ROS_ERROR("cannot reach the goal");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "The robot is encountering an obstacle , cannot reach the goal");
            resetState();
            return true;
          }
          else
          {
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            return false;
          }
             
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
//        record_path_.poses.clear();
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_){
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }
};
