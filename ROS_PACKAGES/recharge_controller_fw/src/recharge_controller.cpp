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
      yaw_delta_(0.05),
      angular_fb_(0.0),
      dist_delta_(0.005),
      time_obs_taltol_(0),
      time_chage_done_taltol_(0),
      front_time_obs_taltol_(0),
      charge_time_out_(0),//dxscpp
      flag(true)
  {

    ros::param::param<double>("recharge_dist", recharge_dist_, 0.8);
    ros::param::param<int>("recharge_repeat", recharge_repeat_, 3);
    ros::param::param<double>("recharge_time_out", time_out_, 5.0);
    ros::param::param<int>("recharge_control_frequency", control_frequency_, 33);

    // 创建控制指令发布器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ctrl_cmd_pub_ = nh_.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 1);

    // 订阅机器人位置信息
    odom_sub_ = nh_.subscribe("odom", 1, &RechargeController::odomCallback, this);

    // 订阅bms信息
    //bms_flag_sub_ = nh_.subscribe("chassis_info_fb", 1, &RechargeController::chassisInfoCallback, this);
    bms_flag_sub_ = nh_.subscribe("io_fb", 1, &RechargeController::chassisInfoCallback, this);//dxs bms_flag_fb

    //订阅底盘轮子角度反馈
    angular_fb_sub_ = nh_.subscribe("angular", 1, &RechargeController::angularCallback, this);
    
    // 创建服务客户端对象
    client_ = nh_.serviceClient<yhs_can_msgs::PathIsValid>("/move_base/check_path");
    
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

// 发布倒退回充导航点
  bool RechargeController::goToTempPoint() {
  
    MoveBaseClient ac("move_base", true); //使用了MoveBaseClient类来创建一个名为ac的客户端对象,将其初始化为与名为"move_base"的MoveBase服务器通信

    while(!ac.waitForServer(ros::Duration(5.0)))  // 5s内等待与move_base服务器建立连接。该函数会一直阻塞，直到成功连接到服务器或超时。
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = temp_point_.pose.position.x; //temp_point_倒退点
    goal.target_pose.pose.position.y = temp_point_.pose.position.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = temp_point_.pose.orientation;


    ROS_INFO("Start Recharge...");
    ac.sendGoal(goal);  //向move_base服务器发送目标位置
    ac.waitForResult(); //等待move_base服务器处理目标位置并返回结果。该方法会一直阻塞程序直到收到结果为止。

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)  //使用ac.getState()函数获取导航任务的执行状态前提是先调用ac.waitForResult()函数等待导航任务的执行结果
    {                                                                 //ac.getState()函数返回一个actionlib::SimpleClientGoalState类型的对象，表示导航任务的状态信息。
      return true;                                                    //可以使用这个对象来判断导航任务是否成功完成、正在执行中还是出现了错误。
    }   
    
    return false; //机器人未成功到达目标位置，或者在等待结果时出现错误，返回false
  }

  // 角度反馈回调函数
  void RechargeController::angularCallback(const std_msgs::Float32::ConstPtr& angular_msg)
  {  
    angular_fb_ = angular_msg->data;
  }

  //  脱离回充
  bool RechargeController::disRechargeCallback(yhs_can_msgs::DisRecharge::Request &req, yhs_can_msgs::DisRecharge::Response &res)
  {
    static bool no_done = false;

    res.result = 0;

    if( (!bms_charge_done_ || (recharge_x_ == 0 && recharge_y_ == 0)) &&  !no_done ) return true;

    no_done = true;

    //检查脱离充电桩过程在是否有障碍物 dxs
    yhs_can_msgs::PathIsValid front_path;
    for(int i = 2; i < 13; i ++) {
      
      geometry_msgs::PoseStamped transformed_point_in,transformed_point_out;
      transformed_point_in.header.stamp = ros::Time();
      transformed_point_in.header.frame_id = "base_link";
      
      transformed_point_in.pose.position.x = 0.05 * i;  //60cm
      transformed_point_in.pose.orientation.w = 1;
      
      tf_listener_.transformPose("map", transformed_point_in, transformed_point_out);
      
      front_path.request.checked_path.poses.push_back(transformed_point_out);
    }

    ros::Rate loop_rate(control_frequency_);

    double dist_delta = 0.0;  //距离误差
    while(dist_delta < 0.5 && ros::ok()) {//50cm

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

      double linear = 0.1;//前进的线速度

      //障碍物检测
      if (client_.call(front_path)) { //通过client_客户端对象调用名为/move_base/check_path的服务，并传递front_path作为请求参数。
        if(front_path.response.result == 0)
        {

          linear = 0.0;
          front_time_obs_taltol_ ++;  //前进脱离充电桩时遇到障碍物累积时长计数

          //有障碍物，超时
          if( static_cast<double>(front_time_obs_taltol_ / control_frequency_) > time_out_ ) {
          
            front_is_safe = false;
          }

        } 
        else front_time_obs_taltol_ = 0;
      } 
      else {
        ROS_ERROR("Failed to call service %s", client_.getService().c_str());
        front_is_safe = false;
      }

      if(!front_is_safe){
        front_time_obs_taltol_ = 0;

        ROS_WARN("disChage front has obs, disChage failed!!!");
        return true;
      }

      publishCmdVel(linear, 0.0);

      loop_rate.sleep();
    }

    publishCmdVel(0.0, 0.0);

    res.result = 1;

    front_time_obs_taltol_ = 0;
    no_done = false;
    return true;
  }    
  
  //回充
  bool RechargeController::rechargeCallback(yhs_can_msgs::Recharge::Request &req, yhs_can_msgs::Recharge::Response &res)
  {    
    if(bms_charge_done_) {
      res.result = 1;
      return true;  //return 1
    }
    // 更新回充点坐标
    recharge_x_ = req.recharge_goal.pose.position.x;  //回充点赋值
    recharge_y_ = req.recharge_goal.pose.position.y;
    tf::Quaternion q( //创建四元数对象q
      req.recharge_goal.pose.orientation.x,
      req.recharge_goal.pose.orientation.y,
      req.recharge_goal.pose.orientation.z,
      req.recharge_goal.pose.orientation.w);
    tf::Matrix3x3 m(q); //创建一个旋转矩阵对象m
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    recharge_yaw_ = yaw;
    
    temp_point_.pose.orientation = req.recharge_goal.pose.orientation;  //recharge_x_回充点   temp_point_倒退点
    
    // 计算开始回充时坐标点的位置，距离充电点正前方1.5m处，方向一致
    temp_point_.pose.position.x = recharge_x_ + std::cos(recharge_yaw_) * recharge_dist_; // recharge_dist_临时坐标点距充电点的距离0.8
    temp_point_.pose.position.y = recharge_y_ + std::sin(recharge_yaw_) * recharge_dist_;
    temp_point_.pose.position.z = 0;
    
    bool to_temp_point = goToTempPoint();//发布倒退回充导航电,并导航前往，到达后，返回true

    if(to_temp_point)
    {
      publishCmdVel(0, 0);  //发布线速度角速度0
      sleep(2);
      // 设置continue_标志变量为true，并通知条件变量，以便唤醒等待该变量的线程
      {
        std::lock_guard<std::mutex> lock(mutex_);
        continue_ = true;
      }
      
      ros::Rate loop_rate(control_frequency_);//ros::Rate 是 ROS 提供的一个用于控制循环频率的类。它允许程序以特定的频率运行，用于实现周期性的任务或控制循环。创建了一个 ros::Rate 对象 loop_rate
      //回充失败后重试 recharge_repeat_ 次
      int repeat = 0;
      while(/*repeat < recharge_repeat_ &&*/ ros::ok()) {
      
//        bool result = execute();
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

          res.result = 0;
          break;
        }
        
        // if(result) {
        //   res.result = 1;
        //   return true;
        // }
        
        // else {
        //   repeat ++;
        // }
        loop_rate.sleep();
      }

    }

    return true; 
  }

// 订阅bms信息反馈
  //void RechargeController::chassisInfoCallback(const yhs_can_msgs::ChassisInfoFb::ConstPtr& chassis_info_msg) {
  void RechargeController::chassisInfoCallback(const yhs_can_msgs::io_fb::ConstPtr& chassis_info_msg) { //dxs
    std::lock_guard<std::mutex> lock(mutex_);
    //bms_charge_done_ = chassis_info_msg->io_fb.io_fb_charge_state;
    bms_charge_done_ = chassis_info_msg -> io_fb_charge_state;
  }

  void RechargeController::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
  }

// 发布控制指令
  void RechargeController::publishCmdVel(const double linear_vel, const double angular_vel) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    cmd_vel_pub_.publish(cmd_vel);
  }

// 发布控制指令
  void RechargeController::publishCtrlCmd(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle) {

    yhs_can_msgs::ctrl_cmd msg;

    msg.ctrl_cmd_linear = linear_vel;
    msg.ctrl_cmd_angular = angular_vel;
    msg.ctrl_cmd_gear = gear;
    msg.ctrl_cmd_slipangle = slipangle;

    ctrl_cmd_pub_.publish(msg);
  }

//计算最短路径
  double RechargeController::distanceToLine(Vector2d A, Vector2d dir, Vector2d B) {
    Vector2d AB = B - A;
    Vector2d u = dir.normalized();
    double AP = AB.dot(u);
    double dist = sqrt(AB.dot(AB) - AP * AP);
    return dist;
  }

//执行回充
  void RechargeController::rechargeControl() {

    // 使用互斥锁保护访问类成员变量的线程安全
    std::lock_guard<std::mutex> lock(mutex_);

    // 使用了tf库来获取机器人当前的坐标信息。
    tf::StampedTransform transform;
    try {                         //源坐标系、目标坐标系
      tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10)); //通过tf::TransformListener类的waitForTransform()函数等待名为"map"和"base_link"之间的坐标变换关系。
                                                                                          //ros::Time(0)表示最新的变换关系
      tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);  //使用lookupTransform()函数从tf监听器tf_listener_中获取"map"到"base_link"之间的坐标变换关系，
                                                                                  //并将结果保存在tf::StampedTransform对象transform中
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
        tf_listener_.transformPoint("base_link", recharge_point, recharge_point_to_base_link_point);//通过tf_listener_ 将 recharge_point 从 "map" 坐标系转换到 "base_link" 坐标系(目标坐标系、源坐标、存储转换后的坐标)
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    // 计算机器人到回充点的距离和方向
    double dx = robot_x_ - recharge_x_;
    double dy = robot_y_ - recharge_y_;
    double temp_dist = std::sqrt(dx * dx + dy * dy);

    double yaw_diff = tf::getYaw(tf::Quaternion(0, 0, sin((robot_yaw_ - recharge_yaw_) / 2), cos((robot_yaw_ - recharge_yaw_) / 2)));

    Vector2d A(recharge_x_, recharge_y_); //使用Vector2d类创建了A和B两个二维向量对象，分别表示充电桩的位置(recharge_x_, recharge_y_)和机器人的位置(robot_x_, robot_y_)。
    double angle = recharge_yaw_;
    Vector2d dir(cos(angle), sin(angle)); //将充电桩的朝向recharge_yaw_通过cos()和sin()函数分别计算出方向向量dir的x和y分量。
    Vector2d B(robot_x_, robot_y_);

    double shortest_dist = distanceToLine(A, dir, B); //计算了机器人当前位置与充电桩之间的最短距离。（垂直于充电桩的最短距离）
    
    // 计算机器人需要的线速度和角速度
    double linear_vel = -0.08;//dxs_change-0.07（速度太小，可能怼不上充电桩）
    double angular_vel = 0.0;
    unsigned char gear = 7;
    double slipangle = 0.0;//目标车体侧偏角°

    if (!bms_charge_done_) {

      double angular_back = std::atan2(recharge_point_to_base_link_point.point.y, recharge_point_to_base_link_point.point.x);

      if(fabs(yaw_diff) >= yaw_delta_)//0.05
      {
        yaw_delta_ = 0.05;
        angular_vel = yaw_diff > 0 ? -0.08 : 0.08;
        linear_vel = 0.0;
        gear = 6;
      }
      
      if(fabs(yaw_diff) < yaw_delta_)
      {
        yaw_delta_ = 0.1;
      }

      //往左偏了
      if( recharge_point_to_base_link_point.point.y > 0 && shortest_dist > dist_delta_ )
      {
        linear_vel = -0.08; 
        angular_vel = 0.0;
        slipangle = -7;

        dist_delta_ = 0.005;
        gear = 7;
      }

      //往右偏了
      if( recharge_point_to_base_link_point.point.y < 0 && shortest_dist > dist_delta_ )
      {
        linear_vel = -0.08;
        angular_vel = 0.0;
        slipangle = 7;

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
/*********************障碍物检测*********************************/    
    //检查后退过程在是否有障碍物 dxs
    yhs_can_msgs::PathIsValid back_path;
    for(int i = 0; i < 4; i ++) {//dxs_change5
      
      geometry_msgs::PoseStamped transformed_point_in,transformed_point_out;
      transformed_point_in.header.stamp = ros::Time();
      transformed_point_in.header.frame_id = "base_link";
      
      transformed_point_in.pose.position.x = -0.05 * i;//机器人后15cm
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
      if(back_path.response.result == 0 && temp_dist > 0.30)//dxs0.25
      {
        ROS_WARN("chage back has obs!!!");
        back_is_safe = false;
      } 
      else time_obs_taltol_ = 0;
    } 
    else {
      ROS_ERROR("Failed to call service %s", client_.getService().c_str());
      back_is_safe = false;
    }
    if(!back_is_safe)
    {
      linear_vel = 0.0;
      angular_vel = 0.0;
      
      time_obs_taltol_ ++;
      
      //有障碍物，超时
      if( static_cast<double>(time_obs_taltol_ / control_frequency_) > time_out_ ) {//static_cast是一个类型转换运算符，用于进行静态类型转换。它可以将一个表达式或变量从一种类型转换为另一种类型
      
        is_charge_failed_ = true;
        return;
      }
    }
/******************************************************/

    // //dxscpp超时异常处理
    // if(fabs(dx) < 0.30 )
    // {
    //   ROS_INFO("dx is %f\n",dx);
    //   static ros::Time last_time = ros::Time::now();
    //   ros::Time current_time = ros::Time::now();
    //   double delta_t = (current_time - last_time).toSec();
    //   if (delta_t > 10 && !bms_charge_done_)  // 如果超过10秒且没有收到充电成功的反馈
    //   {
    //     ROS_WARN("charging timeout!!! time is %f \n",delta_t);
    //     is_charge_failed_ = true;
    //     return;
    //   }
    // }
//*******************************
    // bool static reset_last_time = true;//是否重置last_time
    // static ros::Time last_time;
    // if(fabs(dx) < 0.30)
    // {
    //   ROS_INFO("dx is %f\n", dx);
    //   if(reset_last_time)
    //   {
    //     last_time = ros::Time::now();
    //     reset_last_time = false;
    //   }
    //   ros::Time current_time = ros::Time::now();
    //   double delta_t = (current_time - last_time).toSec();
    //   if(delta_t > 4)
    //   {
    //     last_time = ros::Time::now();
    //     current_time = ros::Time::now();
    //     delta_t = (current_time - last_time).toSec();
    //   }
    //   if (delta_t > 3.95 && !bms_charge_done_)  // 如果超过10秒且没有收到充电成功的反馈
    //   {
    //     ROS_WARN("charging timeout!!! time is %f \n",delta_t);
    //     is_charge_failed_ = true;
    //     reset_last_time = true;
    //     return;
    //   }
    // }

    if(fabs(temp_dist) < 0.30)
    {
      ROS_INFO("temp_dist is %f\n", temp_dist);
      charge_time_out_ ++ ;
      if( static_cast<double>(charge_time_out_ / control_frequency_) > time_out_ && !bms_charge_done_) //5s
      {
        ROS_WARN("charging timeout!!!");
        is_charge_failed_ = true;
        charge_time_out_ = 0;
        return;
      }
    }
    // if(fabs(dx) < 0.30)
    // {
    //   ROS_INFO("dx is %f\n", dx);
    //   if(/*flag &&*/ charge_time_out_ < 10)
    //   {
    //     charge_time_out_ ++;
    //     ROS_INFO("%d ",charge_time_out_);
    //     if( static_cast<double>(charge_time_out_ / control_frequency_) > time_out_ && !bms_charge_done_) 
    //     {
    //       ROS_WARN("charging timeout!!! ");
    //       is_charge_failed_ = true;
    //       charge_time_out_ = 0;
    //       return;
    //     }
    //   }
    //   else{
    //     charge_time_out_ = 0;
    //     //flag = true;
    //   }
    // }
//*****************************

    //异常处理，偏离充电桩
    if( shortest_dist > 0.15 || recharge_point_to_base_link_point.point.x > 0.08 || fabs(yaw_diff) > 0.15){
    
      linear_vel = 0.0;
      angular_vel = 0.0;
    
      is_charge_failed_ = true;

      ROS_WARN("chage robot position except!!!");
    }

    ROS_INFO("info   %f  %f  %d ",shortest_dist, yaw_diff/3.14*180, gear);

    if( fabs(angular_fb_) > 10 && ( fabs(linear_vel) > 0.01 || fabs(slipangle) > 3 ) )
    {
      linear_vel = 0.0;
      angular_vel = 0.0;
    }
    
    // 发布控制指令
    if(gear == 6)
      publishCmdVel(linear_vel, angular_vel);
    else publishCtrlCmd(linear_vel, angular_vel, gear, slipangle);

  }

//复位
  void RechargeController::reset() 
  {
    std::lock_guard<std::mutex> lock(mutex_);

    is_charge_failed_ = false;
    continue_ = false;
    bms_charge_done_ = false;
    is_charge_done_ = false;

    yaw_delta_ = 0.05;    
    dist_delta_ = 0.005;

    time_obs_taltol_ = 0;
    time_chage_done_taltol_ = 0;
  }
} // namespace recharge_controller_ns


