#include "recharge_controller/recharge_controller.h"

namespace recharge_controller_ns
{
  RechargeController::RechargeController()
    : robot_x_(0.0),
      robot_y_(0.0),
      robot_yaw_(0.0),
      recharge_x_(0.0),
      recharge_y_(0.0),
      recharge_yaw_(0.0),
      is_running_(false),
      continue_(false),
      is_charge_done_(false),
      bms_charge_done_(false),
      is_charge_failed_(false),
      cancel_(false),
      yaw_delta_(0.07),
      angular_fb_(0.0),
      dist_delta_(0.005),
      time_obs_taltol_(0),
      time_chage_done_taltol_(0)
  {

    ros::param::param<double>("recharge_dist", recharge_dist_, 0.45);
    ros::param::param<int>("recharge_repeat", recharge_repeat_, 3);
    ros::param::param<double>("recharge_time_out", time_out_, 5.0);
    ros::param::param<int>("recharge_control_frequency", control_frequency_, 33);

    // 创建控制指令发布器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

    ctrl_cmd_pub_ = nh_.advertise<yhs_msgs::FwCtrlCmd>("fw_ctrl_cmd", 1);
    
    //导航类型发布
    nav_type_pub_ = nh_.advertise<std_msgs::UInt8>("nav_type", 1);//dxs

    // 订阅机器人位置信息
    odom_sub_ = nh_.subscribe("odom", 1, &RechargeController::odomCallback, this);

    // 订阅bms信息
    bms_flag_sub_ = nh_.subscribe("chassis_info_fb", 1, &RechargeController::chassisInfoCallback, this);

    //订阅取消命令
    cancel_sub_ = nh_.subscribe("recharge_cancel", 1, &RechargeController::rechargeCancelCallback, this);

    //订阅底盘轮子角度反馈
    angular_fb_sub_ = nh_.subscribe("angular", 1, &RechargeController::angularCallback, this);
    
    // 创建服务客户端对象
    client_ = nh_.serviceClient<yhs_msgs::PathIsValid>("/move_base/check_path");
    
    //创建回充服务端
    recharge_service_ = nh_.advertiseService("recharge", &RechargeController::rechargeCallback,this);

    //创建脱离充电桩服务端
    dis_recharge_service_ = nh_.advertiseService("dis_recharge", &RechargeController::disRechargeCallback,this);
    
  }

  RechargeController::~RechargeController()
  {
  }

  void RechargeController::start()
  {
  }

  void RechargeController::stop()
  {
  }

  bool RechargeController::goToTempPoint() {
  
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = temp_point_.pose.position.x;
    goal.target_pose.pose.position.y = temp_point_.pose.position.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = temp_point_.pose.orientation;

    ROS_INFO("Start Recharge...");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return true;
    }   
    
    return false;
  }

  //取消充电回调函数
  void RechargeController::rechargeCancelCallback(const std_msgs::Bool::ConstPtr& cancel_msg)
  {  
    cancel_ = cancel_msg->data;
  }

  //角度订阅
  void RechargeController::angularCallback(const std_msgs::Float32::ConstPtr& angular_msg)
  {  
    angular_fb_ = angular_msg->data;
  }

  bool RechargeController::disRechargeCallback(yhs_msgs::DisRecharge::Request &req, yhs_msgs::DisRecharge::Response &res)
  {
    static bool no_done = false;

    res.result = 0;

    if( (!bms_charge_done_ || (recharge_x_ == 0 && recharge_y_ == 0)) &&  !no_done ) return true;

    no_done = true;

    //检查脱离充电桩过程在是否有障碍物 
    yhs_msgs::PathIsValid front_path;
    for(int i = 2; i < 13; i ++) {
      
      geometry_msgs::PoseStamped transformed_point_in,transformed_point_out;
      transformed_point_in.header.stamp = ros::Time();
      transformed_point_in.header.frame_id = "base_link";
      
      transformed_point_in.pose.position.x = 0.05 * i;
      transformed_point_in.pose.orientation.w = 1;
      
      tf_listener_.transformPose("map", transformed_point_in, transformed_point_out);
      
      front_path.request.checked_path.poses.push_back(transformed_point_out);
    }

    front_has_obs_taltol_time_ = ros::Time::now();
    ros::Rate loop_rate(control_frequency_);

    double dist_delta = 0.0;
    while(dist_delta < 0.5 && ros::ok()) {

      // 获取机器人当前坐标
      tf::StampedTransform transform;
      try {
        tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10));
        tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return true;
      }

      double robot_x = transform.getOrigin().x();
      double robot_y = transform.getOrigin().y();

      double dx = robot_x - recharge_x_;
      double dy = robot_y - recharge_y_;
      dist_delta = std::sqrt(dx * dx + dy * dy);

      bool front_is_safe = true;

      double linear = 0.1;

      if (client_.call(front_path)) {
        if(front_path.response.result == 0)
        {

          linear = 0.0;

          //有障碍物，超时
          if( (ros::Time::now() - front_has_obs_taltol_time_).toSec() > time_out_ ) {
          
            front_is_safe = false;
          }

        } 
        else front_has_obs_taltol_time_ = ros::Time::now();;
      } 
      else {
        ROS_ERROR("Failed to call service %s", client_.getService().c_str());
        front_is_safe = false;
      }

      if(!front_is_safe){

        ROS_WARN("disChage front has obs, disChage failed!!!");
        return true;
      }

      publishCmdVel(linear, 0.0);

      loop_rate.sleep();
    }

    publishCmdVel(0.0, 0.0);

    res.result = 1;
    no_done = false;
    return true;
  }    
  
  bool RechargeController::rechargeCallback(yhs_msgs::Recharge::Request &req, yhs_msgs::Recharge::Response &res)
  {    
    if(bms_charge_done_) {
      res.result = 1;
      return true;
    }

    // 更新回充点坐标
    recharge_x_ = req.recharge_goal.pose.position.x;
    recharge_y_ = req.recharge_goal.pose.position.y;
    tf::Quaternion q(
      req.recharge_goal.pose.orientation.x,
      req.recharge_goal.pose.orientation.y,
      req.recharge_goal.pose.orientation.z,
      req.recharge_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    recharge_yaw_ = yaw;
    
    temp_point_.pose.orientation = req.recharge_goal.pose.orientation;

    ROS_INFO("recharge point is (%.3f  %.3f)",recharge_x_,recharge_y_);
    
    // 计算开始回充坐标点的位置，距离充点电正前方1.5m处，方向一致
    temp_point_.pose.position.x = recharge_x_ + std::cos(recharge_yaw_) * recharge_dist_;
    temp_point_.pose.position.y = recharge_y_ + std::sin(recharge_yaw_) * recharge_dist_;
    temp_point_.pose.position.z = 0;
    //取消标志位置0
    cancel_ = false;

    std_msgs::UInt8 type;
    type.data = 0;
    nav_type_pub_.publish(type);//dxs

    bool to_temp_point = goToTempPoint();

    if(to_temp_point)
    {
      publishCmdVel(0, 0);
      sleep(2);
      // 设置continue_标志变量为true，并通知条件变量，以便唤醒等待该变量的线程
      {
        std::lock_guard<std::mutex> lock(mutex_);
        continue_ = true;
      }
      
      back_has_obs_taltol_time_ = ros::Time::now();
      ros::Rate loop_rate(control_frequency_);
      //回充失败后重试 recharge_repeat_ 次
      int repeat = 0;
      while(/*repeat < recharge_repeat_ &&*/ ros::ok()) {// TODO

        //取消
        if(cancel_)
        {
          res.result = 0;
          cancel_ = 0;
          break;
        }
      
        //执行回充
        rechargeControl();
        
        //充电成功
        if(is_charge_done_){
          // 停止机器人运动
          publishCmdVel(0.0, 0.0);

          reset();      

          ROS_INFO("recharge done!!!");

          res.result = 1;
          break;
        }
        
        //充电失败
        if(is_charge_failed_) {
        
          reset(); 

          ROS_WARN("recharge failed!!!");

          loop_rate.sleep()
          goToTempPoint();
          //res.result = 0;
          repeat ++;
          break;
        }
        
        loop_rate.sleep();
      }

    }

    return true; 
  }

  void RechargeController::chassisInfoCallback(const yhs_msgs::FwChassisInfoFb::ConstPtr& chassis_info_msg) {

    std::lock_guard<std::mutex> lock(mutex_);
    bms_charge_done_ = chassis_info_msg->io_fb.io_fb_charge_state;
  }

  void RechargeController::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
  }

  void RechargeController::publishCmdVel(const double linear_vel, const double angular_vel) {
    // 发布控制指令
    geometry_msgs::TwistStamped cmd_vel_pub;
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;

    cmd_vel_pub.header.stamp = ros::Time::now();
    cmd_vel_pub.twist = cmd_vel;
    cmd_vel_pub_.publish(cmd_vel_pub);
  }

  void RechargeController::publishCtrlCmd(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle) {
    // 发布控制指令

    yhs_msgs::FwCtrlCmd msg;

    msg.ctrl_cmd_linear = linear_vel;
    msg.ctrl_cmd_angular = angular_vel;
    msg.ctrl_cmd_gear = gear;
    msg.ctrl_cmd_slipangle = slipangle;

    ctrl_cmd_pub_.publish(msg);
  }

  double RechargeController::distanceToLine(Vector2d A, Vector2d dir, Vector2d B) {
    Vector2d AB = B - A;
    Vector2d u = dir.normalized();
    double AP = AB.dot(u);
    double dist = sqrt(AB.dot(AB) - AP * AP);
    return dist;
  }

  void RechargeController::rechargeControl() {

    // 使用互斥锁保护访问类成员变量的线程安全
    std::lock_guard<std::mutex> lock(mutex_);

    // 获取回充点相对于map的变换关系
    tf::StampedTransform transform;
    try {
      tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10));
      tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    // 获取机器人当前坐标
    robot_x_ = transform.getOrigin().x();
    robot_y_ = transform.getOrigin().y();
    robot_yaw_ = tf::getYaw(transform.getRotation());

    geometry_msgs::PointStamped recharge_point;
    recharge_point.header.frame_id = "map";
    recharge_point.point.x = recharge_x_;
    recharge_point.point.y = recharge_y_;
    recharge_point.point.z = 0.0;

    // 将回充点的坐标从地图坐标系下转换为机器人坐标系下的坐标
    geometry_msgs::PointStamped recharge_point_to_base_link_point;
    try{
        tf_listener_.transformPoint("base_link", recharge_point, recharge_point_to_base_link_point);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    // 计算机器人到回充点的距离和方向
    double dx = robot_x_ - recharge_x_;
    double dy = robot_y_ - recharge_y_;
    double temp_dist = std::sqrt(dx * dx + dy * dy);

    double yaw_diff = tf::getYaw(tf::Quaternion(0, 0, sin((robot_yaw_ - recharge_yaw_) / 2), cos((robot_yaw_ - recharge_yaw_) / 2)));

    Vector2d A(recharge_x_, recharge_y_);
    double angle = recharge_yaw_;
    Vector2d dir(cos(angle), sin(angle));
    Vector2d B(robot_x_, robot_y_);

    double shortest_dist = distanceToLine(A, dir, B);

    // 计算机器人需要的线速度和角速度
    double linear_vel = -0.1;
    double angular_vel = 0.0;
    unsigned char gear = 7;
    double slipangle = 0.0;

    if (!bms_charge_done_) {

      double angular_back = std::atan2(recharge_point_to_base_link_point.point.y, recharge_point_to_base_link_point.point.x);

      if(fabs(yaw_diff) >= yaw_delta_)
      {
        yaw_delta_ = 0.05;
        angular_vel = yaw_diff > 0 ? -0.08 : 0.08;
        linear_vel = 0.0;
        gear = 6;
      }
      
      if(fabs(yaw_diff) < yaw_delta_)
      {
        yaw_delta_ = 0.18;
      }

      //往左偏了
      if( recharge_point_to_base_link_point.point.y > 0 && shortest_dist > dist_delta_ )
      {
        linear_vel = -0.1; 
        angular_vel = 0.0;
        slipangle = -9;

        dist_delta_ = 0.005;
        gear = 7;
      }

      //往右偏了
      if( recharge_point_to_base_link_point.point.y < 0 && shortest_dist > dist_delta_ )
      {
        linear_vel = -0.1;
        angular_vel = 0.0;
        slipangle = 9;

        dist_delta_ = 0.005;
        gear = 7;
      }

      if(shortest_dist <= dist_delta_)
      {
        dist_delta_ = 0.01;//0.015
      }
    }
    else{

      if(bms_charge_done_) {

        is_charge_done_ = true;
        return;
      }
	
    }
    
    //检查后退过程在是否有障碍物 
    yhs_msgs::PathIsValid back_path;
    for(int i = 0; i < 5; i ++) {
      
      geometry_msgs::PoseStamped transformed_point_in,transformed_point_out;
      transformed_point_in.header.stamp = ros::Time();
      transformed_point_in.header.frame_id = "base_link";
      
      transformed_point_in.pose.position.x = -0.05 * i;
      transformed_point_in.pose.orientation.w = 1;
      
      tf_listener_.transformPose("map", transformed_point_in, transformed_point_out);
      
      transformed_point_out.pose.orientation.z = transform.getRotation().z();
      transformed_point_out.pose.orientation.w = transform.getRotation().w();
      transformed_point_out.pose.orientation.x = transform.getRotation().x();
      transformed_point_out.pose.orientation.y = transform.getRotation().y();
      
      back_path.request.checked_path.poses.push_back(transformed_point_out);
    }


    bool back_is_safe = true;
    if (client_.call(back_path)) {
      if(back_path.response.result == 0 && temp_dist > 0.33)
      {
        ROS_WARN("chage back has obs!!!");
        back_is_safe = false;
      } 
      else back_has_obs_taltol_time_ = ros::Time::now();
    } 
    else {
      ROS_ERROR("Failed to call service %s", client_.getService().c_str());
      back_is_safe = false;
    }
    
    if(!back_is_safe)
    {
      linear_vel = 0.0;
      angular_vel = 0.0;
      
      //有障碍物，超时
      if( (ros::Time::now() - back_has_obs_taltol_time_).toSec() > time_out_ ) {
      
        is_charge_failed_ = true;
        return;
      }
    }
    
    //异常处理，偏离充电桩
    if( shortest_dist > 0.15 || recharge_point_to_base_link_point.point.x > 0.08 || fabs(yaw_diff) > 0.15){
    
      linear_vel = 0.0;
      angular_vel = 0.0;
    
      is_charge_failed_ = true;

      ROS_WARN("chage robot position except!!!");
    }

    if( fabs(angular_fb_) > 12 && ( fabs(linear_vel) > 0.01 || fabs(slipangle) > 3 ) )
    {
      linear_vel = 0.0;
      angular_vel = 0.0;
    }
    
    // 发布控制指令
    if(gear == 6)
      publishCmdVel(linear_vel, angular_vel);
    else publishCtrlCmd(linear_vel, angular_vel, gear, slipangle);

  }

  void RechargeController::reset() 
  {
    std::lock_guard<std::mutex> lock(mutex_);

    is_charge_failed_ = false;
    continue_ = false;
    bms_charge_done_ = false;
    is_charge_done_ = false;

    yaw_delta_ = 0.07;    
    dist_delta_ = 0.005;

    time_obs_taltol_ = 0;
    time_chage_done_taltol_ = 0;
  }
} // namespace recharge_controller_ns


