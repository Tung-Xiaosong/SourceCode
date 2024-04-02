#include "recharge_controller/recharge_controller.h"

namespace recharge_controller_ns
{
  RechargeController::RechargeController()
    : robot_pose_({0,0,0}),
      recharge_pose_({0,0,0}),
      is_running_(false),
      continue_(false),
      is_charge_done_(false),
      is_charge_failed_(false),
      cancel_(false),
      yaw_delta_(0.07),
      angular_fb_(0.0),
      dist_delta_(0.005),
      time_obs_taltol_(0),
      time_chage_done_taltol_(0)
  {
  
    ros::NodeHandle private_nh("~");
    
    //底盘类型
    private_nh.param("/common/chassis_type", chassis_type_, std::string("FW"));
    
    //距离充电点距离
    private_nh.param("/recharge_controller/recharge_dist", recharge_dist_, 0.45);
    
    //距离充电点距离
    private_nh.param("/recharge_controller/dis_recharge_dist", dis_recharge_dist_, 0.4);
    
    //障碍物超时时间
    private_nh.param("/recharge_controller/recharge_time_out", recharge_time_out_, 5.0);
    
    //控制频率
    private_nh.param("/recharge_controller/control_frequency", control_frequency_, 33);
    
    //线速度
    private_nh.param("/recharge_controller/linear_vel", linear_vel_, 0.1);
    
    //角速度
    private_nh.param("/recharge_controller/angular_vel", angular_vel_, 0.1);
    
    //横移角度
    private_nh.param("/recharge_controller/slipangle", slipangle_, 10.0);
    
    //机器人当前角度与充电点角度差
    private_nh.param("/recharge_controller/yaw_delta", yaw_delta_, 0.07);
    
    //机器人与充电点距离误差
    private_nh.param("/recharge_controller/dist_delta", dist_delta_, 0.005);
    
    //机器人与充电点的距离,大于此值才会检测障碍物
    private_nh.param("/recharge_controller/check_obs_dist_in_back", check_obs_dist_in_back_, 0.33);
    
    //机器人位置到充电路径的最短距离，做偏离异常检测
    private_nh.param("recharge_controller/shortest_dist_y", shortest_dist_y_, 0.15);
  
    //充电点在机器人前面的距离，做偏离异常检测
    private_nh.param("/recharge_controller/shortest_dist_x", shortest_dist_x_, 0.08);
    
    //充电点在机器人前面的距离，做偏离异常检测
    private_nh.param("/recharge_controller/yaw_diff_err", yaw_diff_err_, 0.15);

    // 创建控制指令发布器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

    ctrl_cmd_pub_ = nh_.advertise<yhs_msgs::FwCtrlCmd>("fw_ctrl_cmd", 1);
    
    //导航类型发布
    nav_type_pub_ = nh_.advertise<std_msgs::UInt8>("nav_type", 1);//dxs

    // 订阅机器人位置信息
    odom_sub_ = nh_.subscribe("odom", 1, &RechargeController::odomCallback, this);

    // // 订阅bms信息
    // bms_flag_sub_ = nh_.subscribe("chassis_info_fb", 1, &RechargeController::chassisInfoCallback, this);
    
    //订阅底盘数据
    private_nh.param("/common/chassis_type", chassis_type_, std::string("FW"));
    if(chassis_type_ == "DGT")
    {
      dgt_chassis_info_sub_ = nh_.subscribe<yhs_msgs::DgtChassisInfoFb>("chassis_info_fb", 10, &RechargeController::DgtChassisCB,this);  
    }
    else if(chassis_type_ == "FW")
    {
      fw_chassis_info_sub_ = nh_.subscribe<yhs_msgs::FwChassisInfoFb>("chassis_info_fb", 10, &RechargeController::FwChassisCB,this); 
    }
    else if(chassis_type_ == "MK")
    {
      mk_chassis_info_sub_ = nh_.subscribe<yhs_msgs::MkChassisInfoFb>("chassis_info_fb", 10, &RechargeController::MkChassisCB,this); 
    }
    else if(chassis_type_ == "FR")
    {
      fr_chassis_info_sub_ = nh_.subscribe<yhs_msgs::FrChassisInfoFb>("chassis_info_fb", 10, &RechargeController::FrChassisCB,this); 
    }

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

  bool RechargeController::goToTempPoint(const geometry_msgs::PoseStamped temp_point) {
  
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = temp_point.pose.position.x;
    goal.target_pose.pose.position.y = temp_point.pose.position.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = temp_point.pose.orientation;

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
  
  bool RechargeController::getRobotPose()
  {
    // 获取机器人当前坐标
    tf::StampedTransform transform;
    try {
      tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10));
      tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    
    robot_pose_.x = transform.getOrigin().x();
    robot_pose_.y = transform.getOrigin().y();
    robot_pose_.yaw = tf::getYaw(transform.getRotation());
    
    return true;
  }
  
  bool RechargeController::transformPose(
    const std::string frame,
    const geometry_msgs::PoseStamped & in_pose,
    geometry_msgs::PoseStamped & out_pose)
  {
    if (in_pose.header.frame_id == frame) 
    {
      out_pose = in_pose;
      return true;
    }
  
    try 
    {
      tf_listener_.transformPose(frame, in_pose, out_pose);
      out_pose.header.frame_id = frame;
      return true;
    } 
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return false;
    }
  }
  
  //脱离充电桩
  bool RechargeController::disRechargeCallback(yhs_msgs::DisRecharge::Request &req, yhs_msgs::DisRecharge::Response &res)
  {
    // 获取机器人当前坐标
    if( !getRobotPose() )
    {
      res.result = 0;
      return true;
    }

    // double dx = robot_pose_.x - recharge_pose_.x;
    // double dy = robot_pose_.y - recharge_pose_.y;
    // double dist_delta = std::sqrt(dx * dx + dy * dy);
    
    // //已经开始，遇到障碍物未完成标志
    // static bool no_done = false;

    // //首先判断是否要脱桩
    // if( dist_delta > 0.5 &&  !no_done )
    // {
    //   res.result = 1;
    //   return true;
    // }

    //检查脱离充电桩过程在是否有障碍物 
    yhs_msgs::PathIsValid front_path;
    for(int i = 2; i < 13; i ++) {
      
      geometry_msgs::PoseStamped transformed_point_in,transformed_point_out;
      transformed_point_in.header.stamp = ros::Time();
      transformed_point_in.header.frame_id = "base_link";
      
      transformed_point_in.pose.position.x = 0.05 * i;
      transformed_point_in.pose.orientation.w = 1;
      
      //如果转换失败
      if( !transformPose("map", transformed_point_in, transformed_point_out) )
      {
        res.result = 0;
        return true;
      }
      
      front_path.request.checked_path.poses.push_back(transformed_point_out);
    }

    front_has_obs_taltol_time_ = ros::Time::now();
    ros::Rate loop_rate(control_frequency_);

    double dist_delta = 0.0;
    uint8_t result = 1;
    while(dist_delta < dis_recharge_dist_ && ros::ok()) {

      // 获取机器人当前坐标
      if( !getRobotPose() )
      {
        result = 0;
        break;
      }

      double dx = robot_pose_.x - recharge_pose_.x;
      double dy = robot_pose_.y - recharge_pose_.y;
      dist_delta = std::sqrt(dx * dx + dy * dy);

      double linear = 0.1;

      //调用成功
      if (client_.call(front_path)) {
        if(front_path.response.result == 0)
        {

          linear = 0.0;

          //有障碍物，超时
          if( (ros::Time::now() - front_has_obs_taltol_time_).toSec() > recharge_time_out_ ) {   
            result = 0;
            break;
          }

        } 
        else front_has_obs_taltol_time_ = ros::Time::now();;
      } 
      else {
        ROS_ERROR("Failed to call service %s", client_.getService().c_str());
        result = 0;
        break;
      }
      publishCmdVel(linear, 0.0);
      loop_rate.sleep();
    }

    publishCmdVel(0.0, 0.0);

    if(result == 0)
    {
      ROS_WARN("disChage front has obs, disChage failed!!!");
    }
    
    res.result = result;
    return true;
  }    
  
  bool RechargeController::rechargeCallback(yhs_msgs::Recharge::Request &req, yhs_msgs::Recharge::Response &res)
  {    
    if(is_charge_done_) {
      res.result = 1;
      return true;
    }
    
    ROS_INFO("recharge point is (%.3f  %.3f)",req.recharge_goal.pose.position.x,req.recharge_goal.pose.position.y);

    // 更新回充点坐标
    recharge_pose_.x = req.recharge_goal.pose.position.x;
    recharge_pose_.y = req.recharge_goal.pose.position.y;
    tf::Quaternion q(
      req.recharge_goal.pose.orientation.x,
      req.recharge_goal.pose.orientation.y,
      req.recharge_goal.pose.orientation.z,
      req.recharge_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    recharge_pose_.yaw = yaw;
    
    //临时坐标点
    geometry_msgs::PoseStamped temp_point;
    temp_point.pose.orientation = req.recharge_goal.pose.orientation;
    
    // 计算开始回充坐标点的位置，距离充点电正前方recharge_dist_m处，方向一致
    temp_point.pose.position.x = recharge_pose_.x + std::cos(recharge_pose_.yaw) * recharge_dist_;
    temp_point.pose.position.y = recharge_pose_.y + std::sin(recharge_pose_.yaw) * recharge_dist_;
    temp_point.pose.position.z = 0;
    
    //取消标志位置0
    cancel_ = false;

    std_msgs::UInt8 type;
    type.data = 0;
    nav_type_pub_.publish(type);//dxs

    bool to_temp_point = goToTempPoint(temp_point);

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

      //循环
      while(ros::ok()) {

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
              
          ROS_INFO("recharge done!!!");
          res.result = 1;
          break;
          
        }
        
        //充电失败
        if(is_charge_failed_) {
        
          ROS_WARN("recharge failed!!!");
          res.result = 0;
          break;
          
        }
        
        loop_rate.sleep();
      }
      // 停止机器人运动
      publishCmdVel(0.0, 0.0);
      reset(); 
    }
    return true; 
  }


  //DGT底盘信息
  void RechargeController::DgtChassisCB(const yhs_msgs::DgtChassisInfoFb::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_charge_done_ = msg->io_fb.io_fb_charge_state;
  }
  
  //FW底盘信息
  void RechargeController::FwChassisCB(const yhs_msgs::FwChassisInfoFb::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_charge_done_ = msg->io_fb.io_fb_charge_state;
  }
  
  //MK底盘信息
  void RechargeController::MkChassisCB(const yhs_msgs::MkChassisInfoFb::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_charge_done_ = msg->bms_flag_info_fb.bms_flag_info_charge_flag;
  }
  
  //FR底盘信息
  void RechargeController::FrChassisCB(const yhs_msgs::FrChassisInfoFb::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_charge_done_ = msg->bms_flag_info_fb.bms_flag_info_charge_flag;
  }

  // void RechargeController::chassisInfoCallback(const yhs_msgs::FwChassisInfoFb::ConstPtr& chassis_info_msg) {

  //   std::lock_guard<std::mutex> lock(mutex_);
  //   is_charge_done_ = chassis_info_msg->io_fb.io_fb_charge_state;
  // }

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

    // 获取机器人当前坐标
    if( !getRobotPose() )
    {
      is_charge_failed_ = true;
      return;
    }

    // 将回充点的坐标从地图坐标系下转换为机器人坐标系下的坐标
    geometry_msgs::PoseStamped r_p,r_p_t_b_l_p;
    r_p.header.frame_id = "map";
    r_p.pose.position.x = recharge_pose_.x;
    r_p.pose.position.y = recharge_pose_.y;
    r_p.pose.position.z = 0.0;
    if( !transformPose("base_link", r_p, r_p_t_b_l_p) )
    {
      is_charge_failed_ = true;
      return;
    }

    // 计算机器人到回充点的距离和方向
    double dx = robot_pose_.x - recharge_pose_.x;
    double dy = robot_pose_.y - recharge_pose_.y;
    double temp_dist = std::sqrt(dx * dx + dy * dy);

    double yaw_diff = tf::getYaw(tf::Quaternion(0, 0, sin((robot_pose_.yaw - recharge_pose_.yaw) / 2),\
    cos((robot_pose_.yaw - recharge_pose_.yaw) / 2)));

    Vector2d A(recharge_pose_.x, recharge_pose_.y);
    double angle = recharge_pose_.yaw;
    Vector2d dir(cos(angle), sin(angle));
    Vector2d B(robot_pose_.x, recharge_pose_.y);

    double shortest_dist = distanceToLine(A, dir, B);

    // 计算机器人需要的线速度和角速度
    double linear_vel = -linear_vel_;
    double angular_vel = 0.0;
    unsigned char gear = 7;
    double slipangle = 0.0;
    
    double yaw_delta = yaw_delta_;
    double dist_delta = dist_delta_;
    
    if(chassis_type_ == "DGT") gear = 3;

    if (!is_charge_done_) {

      double angular_back = std::atan2(r_p_t_b_l_p.pose.position.y, r_p_t_b_l_p.pose.position.x);

      //如果大于yaw_delta，停下来原地旋转
      if(fabs(yaw_diff) >= yaw_delta)
      {
        yaw_delta = yaw_delta_ - 0.02;
        angular_vel = yaw_diff > 0 ? -angular_vel_ : angular_vel_;
        linear_vel = 0.0;
        
        if(chassis_type_ == "FW") gear = 6;

      }
      
      //让底盘不反复调整
      if(fabs(yaw_diff) < yaw_delta)
      {
        yaw_delta = yaw_delta_ + 0.1;
      }

      //往左偏了
      if( r_p_t_b_l_p.pose.position.y > 0 && shortest_dist > dist_delta )
      {
        linear_vel = -linear_vel_; 
        angular_vel = angular_vel_;
        slipangle = -slipangle_;

        dist_delta = dist_delta_;
      }

      //往右偏了
      if( r_p_t_b_l_p.pose.position.y < 0 && shortest_dist > dist_delta )
      {
        linear_vel = -linear_vel_;
        angular_vel = -angular_vel_;
        slipangle = slipangle_;

        dist_delta = dist_delta_;
      }

      //让底盘不反复调整
      if(shortest_dist <= dist_delta)
      {
        dist_delta = dist_delta_ + 0.01;//0.015
      }
    }
    else
    {
      return;
    }
    
    //检查后退过程在是否有障碍物 
    yhs_msgs::PathIsValid back_path;
    for(int i = 0; i < 5; i ++) {
      
      geometry_msgs::PoseStamped transformed_point_in,transformed_point_out;
      transformed_point_in.header.stamp = ros::Time();
      transformed_point_in.header.frame_id = "base_link";
      
      transformed_point_in.pose.position.x = -0.05 * i;
      transformed_point_in.pose.orientation.w = 1;
      
      //如果转换失败
      if( !transformPose("map", transformed_point_in, transformed_point_out) )
      {
        is_charge_failed_ = true;
        return;
      }
      
      back_path.request.checked_path.poses.push_back(transformed_point_out);
    }


    bool back_is_safe = true;
    if (client_.call(back_path)) {
      if(back_path.response.result == 0 && temp_dist > check_obs_dist_in_back_)
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
      if( (ros::Time::now() - back_has_obs_taltol_time_).toSec() > recharge_time_out_ ) {
      
        is_charge_failed_ = true;
        ROS_WARN("obs except!!!");        
        return;
        
      }
    }
    
    //异常处理，偏离充电桩
    if( shortest_dist > shortest_dist_y_ || r_p_t_b_l_p.pose.position.x > shortest_dist_x_ || fabs(yaw_diff) > yaw_diff_err_){
    
      linear_vel = 0.0;
      angular_vel = 0.0;
      is_charge_failed_ = true;
      ROS_WARN("robot position except!!!");
      return;
      
    }

    if( fabs(angular_fb_) > 12 && ( fabs(linear_vel) > 0.01 || fabs(slipangle) > 3 ) )
    {
      linear_vel = 0.0;
      angular_vel = 0.0;
    }
    
    // 发布控制指令
    if(gear != 7)
      publishCmdVel(linear_vel, angular_vel);
    else publishCtrlCmd(linear_vel, angular_vel, gear, slipangle);

  }

  void RechargeController::reset() 
  {
    std::lock_guard<std::mutex> lock(mutex_);

    is_charge_failed_ = false;
    continue_ = false;
    is_charge_done_ = false;

    // yaw_delta_ = 0.07;    
    // dist_delta_ = 0.005;

    time_obs_taltol_ = 0;
    time_chage_done_taltol_ = 0;
  }
} // namespace recharge_controller_ns


