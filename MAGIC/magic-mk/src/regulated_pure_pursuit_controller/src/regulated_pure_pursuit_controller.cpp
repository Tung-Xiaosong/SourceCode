

#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav_core::BaseLocalPlanner)

namespace regulated_pure_pursuit_controller
{

RegulatedPurePursuitController::RegulatedPurePursuitController() : initialized_(false), odom_helper_("odom"), goal_reached_(false),\
in_slope_region_(false),in_slow_region_(false),nav_type_(0),goal_max_speed_(0.0),graph_max_speed_(0.0),record_max_speed_(0.0),\
base_link_to_car_front_dist_(0.28),slow_speed_max_(0.3),car_type_(0),chassis_linear_(0.0),chassis_angular_(0.0)
{ }    

// base
void RegulatedPurePursuitController::initialize(std::string name, tf::TransformListener* tf,
                                      costmap_2d::Costmap2DROS *costmap_ros)
{
  if (!isInitialized() ){
    ros::NodeHandle pnh_("~/" + name);

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    
    initParams(pnh_);// 初始化参数
    initPubSubSrv(pnh_);// 初始化发布器

    odom_helper_.setOdomTopic( odom_topic_ );

    initialized_ = true;
    ROS_INFO("RegulatedPurePursuitController Initialized");
  }
  else {
    ROS_WARN("RegulatedPurePursuitController has already been initialized, doing nothing.");
  }


}

void RegulatedPurePursuitController::initPubSubSrv(ros::NodeHandle& nh){
  global_path_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
  carrot_pub_ = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = nh.advertise<nav_msgs::Path>("lookahead_collision_arc", 1);


}

void RegulatedPurePursuitController::initParams(ros::NodeHandle& nh){
  nh.param<std::string>("odom_topic", odom_topic_, "odom");

  nh.param<double>("max_robot_pose_search_dist", max_robot_pose_search_dist_, getCostmapMaxExtent());

  nh.param<int>("min_global_plan_complete_size", min_global_plan_complete_size_, 20);
  nh.param<double>("global_plan_prune_distance", global_plan_prune_distance_, 1.0);

  //Lookahead
  nh.param<double>("lookahead_time", lookahead_time_, 1.5);
  nh.param<double>("lookahead_dist", lookahead_dist_, 1.0);
  nh.param<bool>("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_, false);
  nh.param<double>("min_lookahead_dist", min_lookahead_dist_, 0.1);
  nh.param<double>("max_lookahead_dist", max_lookahead_dist_, 5.0);

  //角度相差太大，原地旋转
  //Rotate to heading param
  nh.param<bool>("use_rotate_to_heading", use_rotate_to_heading_, true);
  nh.param<double>("rotate_to_heading_min_angle", rotate_to_heading_min_angle_, 0.5);
  nh.param<double>("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_, 0.25);
  nh.param<double>("max_angular_accel", max_angular_accel_, 0.8);

  //Reversing
  nh.param<bool>("allow_reversing", allow_reversing_, true);
  if (use_rotate_to_heading_ && allow_reversing_) {
    ROS_WARN("Disabling reversing. Both use_rotate_to_heading and allow_reversing "
    "parameter cannot be set to true. By default setting use_rotate_to_heading true");
//    allow_reversing_ = false;
  }
  nh.param<double>("reversing_dist", reversing_dist_, -0.05);

  //最大速度
  nh.param<double>("desired_linear_vel", max_seed_limit_, 1.5);

  //最大角速度
  nh.param<double>("max_angular_vel", max_angular_vel_, 0.6);

  //减速到的最小速度
  nh.param<double>("min_approach_linear_velocity", min_approach_linear_velocity_, 0.2);

  //剩下路径的长度，减速因子
  nh.param<double>("approach_velocity_scaling_dist", approach_velocity_scaling_dist_, 1.0);
  nh.param<double>("approach_velocity_scaling_long_dist", approach_velocity_scaling_long_dist_, 1.5);

  //Regulated linear velocity scaling
  nh.param<bool>("use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_, true);

  //最小转弯半径
  nh.param<double>("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_, 0.8);

  nh.param<double>("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_, 0.25);
  
  //Inflation cost scaling (Limit velocity by proximity to obstacles)
  nh.param<bool>("use_cost_regulated_linear_velocity_scaling", use_cost_regulated_linear_velocity_scaling_, false);
  nh.param<double>("inflation_cost_scaling_factor", inflation_cost_scaling_factor_, 3.0);
  nh.param<double>("cost_scaling_dist", cost_scaling_dist_, 0.6);
  nh.param<double>("cost_scaling_gain", cost_scaling_gain_, 1.0);
  if (inflation_cost_scaling_factor_ <= 0.0){
    ROS_WARN("The value inflation_cost_scaling_factor is incorrectly set, "
        "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  //Collision avoidance
  nh.param<double>("max_allowed_time_to_collision_up_to_carrot", max_allowed_time_to_collision_up_to_carrot_, 1.0);
  nh.param<bool>("use_collision_detection", use_collision_detection_, true);
  
  //目标点距离参数  
  nh.param<double>("goal_dist_tol", goal_dist_tol_, 0.05);
  nh.param<double>("goal_dist_tol_temp", goal_dist_tol_temp_, goal_dist_tol_);
  nh.param<double>("goal_dist_tol_error", goal_dist_tol_error_, goal_dist_tol_ + 0.2);

  nh.param<double>("control_frequency", control_frequency_, 20);
  control_duration_ = 1.0 / control_frequency_;

  double transform_tolerance;
  nh.param<double>("transform_tolerance", transform_tolerance, 0.1);
  transform_tolerance_ = ros::Duration(transform_tolerance);

  //Ddynamic Reconfigure

  ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(nh));
  ddr_->registerVariable<double>("lookahead_time", &this->lookahead_time_, "", 0.0, 20.0);
  ddr_->registerVariable<double>("lookahead_dist", &this->lookahead_dist_, "", 0.0, 20.0);

  ddr_->registerVariable<bool>("use_velocity_scaled_lookahead_dist", &this->use_velocity_scaled_lookahead_dist_);
  ddr_->registerVariable<double>("min_lookahead_dist", &this->min_lookahead_dist_, "", 0.0, 5.0);
  ddr_->registerVariable<double>("max_lookahead_dist", &this->max_lookahead_dist_, "", 0.0, 10.0);

  //reversing
  ddr_->registerVariable<double>("reversing_dist", &this->reversing_dist_, "", -0.2, 0.1);

  //Rotate to heading param
  ddr_->registerVariable<bool>("use_rotate_to_heading", &this->use_rotate_to_heading_);
  ddr_->registerVariable<double>("rotate_to_heading_min_angle", &this->rotate_to_heading_min_angle_, "", 0.0, 15.0);
  ddr_->registerVariable<double>("rotate_to_heading_angular_vel", &this->rotate_to_heading_angular_vel_, "", 0.0, 15.0);
  ddr_->registerVariable<double>("max_angular_accel", &this->max_angular_accel_, "", 0.0, 15.0);

  //Speed
  ddr_->registerVariable<double>("max_seed_limit", &this->max_seed_limit_, "", 0.0, 10.0);
  ddr_->registerVariable<double>("max_angular_vel", &this->max_angular_vel_, "", 0.0, 10.0);
  ddr_->registerVariable<double>("min_approach_linear_velocity", &this->min_approach_linear_velocity_, "", 0.0, 10.0);
  ddr_->registerVariable<double>("approach_velocity_scaling_dist", &this->approach_velocity_scaling_dist_, "", 0.0, 10.0);

  //Regulated linear velocity scaling
  ddr_->registerVariable<bool>("use_regulated_linear_velocity_scaling", &this->use_regulated_linear_velocity_scaling_);
  ddr_->registerVariable<double>("regulated_linear_scaling_min_radius", &this->regulated_linear_scaling_min_radius_, "", 0.0, 5.0);
  ddr_->registerVariable<double>("regulated_linear_scaling_min_speed", &this->regulated_linear_scaling_min_speed_, "", 0.0, 5.0);
  
  //Inflation cost scaling (Limit velocity by proximity to obstacles)
  ddr_->registerVariable<bool>("use_cost_regulated_linear_velocity_scaling", &this->use_cost_regulated_linear_velocity_scaling_);
  ddr_->registerVariable<double>("inflation_cost_scaling_factor", &this->inflation_cost_scaling_factor_, "", 0.0, 10.0);
  ddr_->registerVariable<double>("cost_scaling_dist", &this->cost_scaling_dist_, "", 0.0, 10.0);
  ddr_->registerVariable<double>("cost_scaling_gain", &this->cost_scaling_gain_, "", 0.0, 10.0);

  //Collision avoidance
  ddr_->registerVariable<double>("max_allowed_time_to_collision_up_to_carrot", &this->max_allowed_time_to_collision_up_to_carrot_, "", 0.0, 10.0);
  ddr_->registerVariable<double>("goal_dist_tol", &this->goal_dist_tol_, "", 0.0, 4.0);
  ddr_->registerVariable<double>("goal_dist_tol_error", &this->goal_dist_tol_error_, "", 0.0, 4.0);
  ddr_->registerVariable<bool>("use_collision_detection", &this->use_collision_detection_);

  ddr_->publishServicesTopics();
  
}

//给全局路径设置方向
void RegulatedPurePursuitController::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path)
{
  // if(nav_type_ == 1)
  // {
  //   std::vector<geometry_msgs::PoseStamped> temp;
  //   temp.push_back(path.front());

  //   for(int i = 4; i < path.size() - 1; i += 4)
  //   {
  //     temp.push_back(path[i]);
  //   }

  //   temp.push_back(path.back());

  //   path = temp;
  // }

  for(int i = 0; i < path.size() - 1; i ++)
  {
    double x0 = path[i].pose.position.x,
          y0 = path[i].pose.position.y,
          x1 = path[i + 1].pose.position.x,
          y1 = path[i + 1].pose.position.y;
          
    double angle = atan2(y1-y0,x1-x0);


    path[i].pose.orientation = tf::createQuaternionMsgFromYaw(angle);

    if(path[i].pose.orientation.w > 2  || path[i].pose.orientation.w < 0 || path[i].pose.orientation.w == 0)
    {
      ROS_WARN("  %.3f  %.3f  %.3f   %.3f   %.3f  %.3f",path[i].pose.orientation.w,angle,x0,y0,x1,y1);
    }
//    set_angle(&path[index], angle);
  }
}

// base 获得全局路径global_plan_
bool RegulatedPurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if(!initialized_)
  {
    ROS_ERROR("RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  goal_reached_ = false;

  return true;
}

//设置最大速度
bool RegulatedPurePursuitController::setMaxSpeed(const double goal_max_speed, const double graph_max_speed, const double record_max_speed)
{
  goal_max_speed_ = goal_max_speed;
  graph_max_speed_ = graph_max_speed;
  record_max_speed_ = record_max_speed;
  return true;
}

//设置斜坡和减速区域标志
bool RegulatedPurePursuitController::setRegion(const bool slope_region, const bool slow_region, const bool narrow_region)
{
  is_in_slope_region_ = slope_region;
  is_in_slow_region_ = slow_region;
  is_in_narrow_region_ = narrow_region;
  return true;
}

//设置导航类型和底盘类型 底盘轮子角度 底盘轮子速度 手绘路径和录制路径是否处于绕障中
bool RegulatedPurePursuitController::setNavType(const uint8_t nav_type,const uint8_t car_type,\
const float chassis_angular,const float chassis_linear,const bool is_in_avoid_obs)
{
  nav_type_ = nav_type;
  car_type_ = car_type;
  chassis_angular_ = chassis_angular;
  chassis_linear_ = chassis_linear;
  is_in_avoid_obs_ = is_in_avoid_obs;
  return true;
}

// base
uint32_t RegulatedPurePursuitController::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                  const geometry_msgs::TwistStamped& velocity,
                                                  geometry_msgs::TwistStamped &cmd_vel,
                                                  std::string &message)
{
  if(!initialized_)
  {
    ROS_ERROR("RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
    return false;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

  goal_reached_ = false;  

  double linear_vel, angular_vel;
  double lookahead_dist = lookahead_dist_;
  self_rotate_angular_ = 0.75;

  //Get current pose of robot
  tf::Stamped<tf::Pose> tf_robot_pose;
  costmap_ros_->getRobotPose(tf_robot_pose);
  
  geometry_msgs::PoseStamped robot_pose;  
  tf::poseStampedTFToMsg(tf_robot_pose, robot_pose);

  // Get robot velocity
  geometry_msgs::Twist robot_cur_speed; 
  getRobotVel(robot_cur_speed);

  pruneGlobalPlan(*tf_, tf_robot_pose, global_plan_, 1.0);
  
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  tf::StampedTransform tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, tf_robot_pose, *costmap_, global_frame_, max_lookahead_dist_, 
                          transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }
  
  //给手绘路径路径点赋予方向
  if(nav_type_ == 1)
    setAngleBasedOnPositionDerivative(global_plan_);

  //将转换后的路径的全局坐标转换为base_link坐标
  std::vector<geometry_msgs::PoseStamped> transformed_local_plan;

  for (const auto& pose : transformed_plan) {

    try
    {
      // 创建一个用于存储转换后结果的PoseStamped消息
      geometry_msgs::PoseStamped transformed_point;

      // 设置输入点的header信息
      transformed_point.header = pose.header;

      // 进行坐标变换，将输入点从map坐标系转换到base_link坐标系
      tf_->transformPose("base_link", pose, transformed_point);

      transformed_local_plan.push_back(transformed_point);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Failed to transform path: %s", ex.what());
    }
  }

  // check if global goal is reached
  tf::Stamped<tf::Pose> global_goal;
  tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
  global_goal.setData( tf_plan_to_global * global_goal );
  double dx = global_goal.getOrigin().getX() - tf_robot_pose.getOrigin().getX();
  double dy = global_goal.getOrigin().getY() - tf_robot_pose.getOrigin().getY();
  const double robot_to_goal_dist = std::sqrt(dx*dx+dy*dy);

  // 将点转换为tf::Transform类型
  tf::Transform transform1(global_goal.getRotation(), global_goal.getOrigin());
  tf::Transform transform2(tf_robot_pose.getRotation(), tf_robot_pose.getOrigin());

  // 计算姿态之间的角度差
  tf::Quaternion q_diff = transform1.getRotation() * transform2.getRotation().inverse();
  double delta_orient = tf::getYaw(q_diff);

  // 判断是否到达目标点 
  if(robot_to_goal_dist < goal_dist_tol_ && car_type_ > 2 && fabs(delta_orient) > 1.57)//dxs
  {
    goal_reached_ = true;
    return true;
  }
  else if((robot_to_goal_dist < goal_dist_tol_ && fabs(delta_orient) < 0.1) || \
  (car_type_ > 2 && fabs(transformed_local_plan.back().pose.position.x) < goal_dist_tol_ && fabs(delta_orient) < 0.1 \
  && fabs(transformed_local_plan.back().pose.position.y) < 0.3))
  {
    goal_reached_ = true;
    return true;
  }

  nav_msgs::Path local_path;
  local_path.header.stamp = ros::Time::now();
  local_path.header.frame_id = "base_link";
  for (const auto& pose : transformed_local_plan) {

    local_path.poses.push_back(pose);
  }

  local_plan_pub_.publish(local_path);


  //根据导航类型，赋最大的速度
  if(nav_type_ == 0)
  {
    linear_vel = goal_max_speed_;
  }
  else if(nav_type_ == 1)
  {
    linear_vel = graph_max_speed_;
  }
  else
  {
    linear_vel = record_max_speed_;
  }

  //计算弯曲程度
  double direction_difference_sum = 0.0;
  double front_path_length = 0.0;
  int value_num = 0;

  if(global_plan_.size() > 5)
  {
    for(int i = 2; i < global_plan_.size() - 3; i ++)
    {
  //    double delta_angular = fabs(std::atan2(global_plan_[i].pose.position.y, global_plan_[i].pose.position.x));

      if(global_plan_[i].pose.orientation.w > 2 || global_plan_[i + 1].pose.orientation.w > 2 || global_plan_[i].pose.orientation.w == 0)
      {
        ROS_WARN("global_plan_[%lu].pose.orientation error!!!",global_plan_.size());
        std::cout << global_plan_[i].pose.orientation << std::endl;
        continue;
      }

      double yaw1 = tf::getYaw(global_plan_[2].pose.orientation);
      double yaw2 = tf::getYaw(global_plan_[i + 1].pose.orientation);
      double yaw_diff = fabs(std::atan2(std::sin(yaw1 - yaw2),std::cos(yaw1 - yaw2)));

//      ROS_INFO("global_plan_.size   yaw1   yaw2   %d  %.3f   %.3f  %.3f",global_plan_.size(),yaw1,yaw2,yaw_diff);

  //    front_path_length += sqrt(global_plan_[i].pose.position.y * global_plan_[i].pose.position.y + \
  //                        global_plan_[i].pose.position.x * global_plan_[i].pose.position.x);

      double dx = global_plan_[i].pose.position.x - tf_robot_pose.getOrigin().getX();
      double dy = global_plan_[i].pose.position.y - tf_robot_pose.getOrigin().getY();

      double angular_del = 0.05;
//      if(nav_type_ == 1) angular_del = 0.1;
      if( yaw_diff >= angular_del && std::sqrt(dx*dx+dy*dy) > 0.5 )
      {
        direction_difference_sum += yaw_diff;
        value_num ++;
      }
    }
    if(value_num > 0) direction_difference_sum /= value_num;
  }

  //路径弯曲程度对速度的影响
  static bool direction_difference_sum_is_big_than_zero_dot_five = false;
  static ros::Time is_small_than_zero_dot_five_time = ros::Time::now();
  static ros::Time is_big_than_zero_dot_five_time = ros::Time::now();
  {
    if(direction_difference_sum <= 0.2)
    {
      is_small_than_zero_dot_five_time = ros::Time::now();
    } 

    if( direction_difference_sum >= 0.25)
    {
      is_big_than_zero_dot_five_time = ros::Time::now();
    }

    //direction_difference_sum大于keep_time秒后限制速度
    double keep_time = 0.2;
    if(linear_vel >= 1.0) 
      keep_time = 0.1;
    if( (ros::Time::now() - is_small_than_zero_dot_five_time).toSec() > keep_time || direction_difference_sum_is_big_than_zero_dot_five )
    {
//      ROS_WARN("direction_difference_sum > 0.25");
      if(linear_vel > 0.5)
        linear_vel = 0.5;
      direction_difference_sum_is_big_than_zero_dot_five = true;
    }

    //direction_difference_sum小于2s后恢复速度
    if( (ros::Time::now() - is_big_than_zero_dot_five_time).toSec() > 2 )
    {
      direction_difference_sum_is_big_than_zero_dot_five = false;
    }
  }

//  if(direction_difference_sum >= 0.25) 
//    ROS_WARN("direction_difference_sum : %f",direction_difference_sum);

  //路径周围障碍物对速度的影响
  static bool obs_time_is_big_than_two_second = false;
  static ros::Time obs_in_robot_left_right_40cm_time = ros::Time::now();
  static ros::Time no_obs_in_robot_left_right_40cm_time = ros::Time::now();
  {
    double length = 0;
    for(int i = 1; i < global_plan_.size(); i ++)
    {

      length += hypot(global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x,\
      global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y);

      if(length > 2.0) break;

      tf::Quaternion RQ2;
      double roll,pitch,yaw;
      tf::quaternionMsgToTF(global_plan_[i].pose.orientation,RQ2);
      tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);

      //将底盘左右尺寸增加60厘米，障碍物判断
      std::vector<geometry_msgs::Point> temp_foot_points;
      temp_foot_points = footprint_spec_;

      temp_foot_points[0].y += 0.4;
      temp_foot_points[1].y -= 0.4;
      temp_foot_points[2].y += 0.4;
      temp_foot_points[3].y -= 0.4;

      double footprint_cost = costmap_model_->footprintCost(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, yaw,\
       temp_foot_points, robot_inscribed_radius_, robot_circumscribed_radius);
      if( footprint_cost < 0 )
      {
        obs_in_robot_left_right_40cm_time = ros::Time::now();   
      }
      else
      {
        no_obs_in_robot_left_right_40cm_time = ros::Time::now();
      }
    }

    //左右两边有障碍物超过keep_time秒
    double keep_time = 2;
    if(linear_vel >= 0.5 && linear_vel < 1.0) 
      keep_time = 1.5;
    if(linear_vel >= 1.0) 
      keep_time = 1;
    if( (ros::Time::now() - no_obs_in_robot_left_right_40cm_time).toSec() > keep_time || obs_time_is_big_than_two_second )
    {
      obs_time_is_big_than_two_second = true;
      if(linear_vel > 0.5) linear_vel = 0.5;
      ROS_WARN("left and right obs > 0.3 ");
    } 

    //左右两边没有障碍物超过2s
    if( (ros::Time::now() - obs_in_robot_left_right_40cm_time).toSec() > 2 )
    {
      obs_time_is_big_than_two_second = false;
    } 
  }

  //机器人距离目标点的距离和当前的速度对目标速度的影响
  if( std::sqrt(dx*dx+dy*dy) <= 3.0 && linear_vel >= 0.5 && robot_cur_speed.linear.x < 0.55 )
  {
    linear_vel = 0.5;
  }

  //距离目标点的最大速度限制
  if( robot_to_goal_dist <= 1.0 && linear_vel > 0.5 )
  {
//    linear_vel = 0.5;
  }

  //斜坡或者减速区域对目标速度的影响
  if(is_in_slope_region_ || is_in_slow_region_)
  {
    linear_vel = std::min(slow_speed_max_,linear_vel);
  }

  //路径弯曲程度对前视距离点的影响
  if( !direction_difference_sum_is_big_than_zero_dot_five /*direction_difference_sum >= 0.5*/)
  {
    lookahead_dist = 3.0;
  }
  if( direction_difference_sum >= 0.5)
  {
    lookahead_dist = 0.7;
  }

  //手绘路径或者录制路径是否处于绕障中对速度和前视距离的影响
  if(is_in_avoid_obs_)
  {
    if(linear_vel > 0.5)
      linear_vel = 0.5;
    lookahead_dist = 0.7;
  }

  //窄通道对速度和前视距离点的影响
  if(is_in_narrow_region_)
  {
    if(linear_vel > 0.3) linear_vel = 0.3;
    lookahead_dist = 0.3;
  }

  //获取前视距离点
  geometry_msgs::PoseStamped carrot_pose;
  {
    carrot_pose = getLookAheadPoint(lookahead_dist, transformed_local_plan);
    geometry_msgs::PointStamped carrot_msg ;
    carrot_msg.header = carrot_pose.header;
    carrot_msg.point.x = carrot_pose.pose.position.x;
    carrot_msg.point.y = carrot_pose.pose.position.y;
    carrot_msg.point.z = 0.01;  // publish right over map to stand out
    carrot_pub_.publish(carrot_msg);

    // const double carrot_dist2 =
    // (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    // (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

    // double curvature = 0.0;
    // if (carrot_dist2 > 0.001) {
    //   curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
    // }
  };

  //针对DGT FW底盘
  if(car_type_ <= 2)
  {
    //检查是否要后退
    if( frontHasObs(robot_pose) && shouldReversing(transformed_local_plan) && \
    std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y) < 2.0 ) 
    {
      linear_vel = -0.1;
      angular_vel = 0;
    }
  
    //不后退，正常行驶
    else
    {
      //原地调节方向到达目标点方向 
      if (shouldRotateToGoalHeading(carrot_pose))
      {
        double angle_to_goal = tf::getYaw(transformed_local_plan.back().pose.orientation);
        rotateToHeading(linear_vel, angular_vel, angle_to_goal);
      }
  
      else
      {
        double angular_front = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  
        //设置线速度  
        applyConstraints(local_path, linear_vel,robot_cur_speed);
        
        linear_vel = clamp(fabs(linear_vel), 0.0, max_seed_limit_);
  
        //FW车型
        if(car_type_ == 2)
        {
          //设置最小角速度 0.3m/s=>0.03  0.5m/s=>0.044   0.8m/s=>0.07  1m/s=>0.09  1.2m/s=>0.105  1.5m/s=>0.131 
          static double angular_vel_factor = 1.0;
          if( fabs(robot_cur_speed.angular.z) < 0.01 && fabs(angular_vel) > 0.01 && robot_to_goal_dist > 2)
          {
            angular_front *= angular_vel_factor;
            angular_vel_factor += 0.3;
          }
          else
          {
            angular_vel_factor = 1.0;
          }
  
          //设置最小角速度 0.3m/s=>0.03  0.5m/s=>0.044   0.8m/s=>0.07  1m/s=>0.09  1.2m/s=>0.105  1.5m/s=>0.131 
          if(angular_front != 0 && robot_cur_speed.linear.x > 0.05 && robot_to_goal_dist < 2)
          {
            if( robot_cur_speed.linear.x / fabs(angular_front) > 11)
            {
              double multiple = robot_cur_speed.linear.x / 11.5;
              angular_front = angular_front > 0 ? multiple : -multiple;
            }
          }
        }
  
        //角速度
        angular_vel = clamp(angular_front, -max_angular_vel_, max_angular_vel_);
  
        static bool self_rote = false;
        if(fabs(angular_front) > self_rotate_angular_ || self_rote)
        {
          self_rote = true;
          geometry_msgs::PoseStamped carrot_pose_near = getLookAheadPoint(0.2, transformed_local_plan);
  
          double angular_near = std::atan2(carrot_pose_near.pose.position.y, carrot_pose_near.pose.position.x);
  
          if(fabs(angular_near) < 0.15)
          {
            self_rote = false;
            ROS_INFO("self_rote done !!!");
          }
          else
          {
            int sign = angular_near > 0 ? 1 : -1;
  
            if( fabs(angular_near) > 3.0)
              sign = 1;
  
            angular_vel = sign * rotate_to_heading_angular_vel_;
  
            linear_vel = 0;
          }
        }
      }
    }
  }
  
  // TODO ackman底盘 -----------------------------------
  else
  {
    double dist_to_goal = std::hypot(transformed_local_plan.back().pose.position.x, transformed_local_plan.back().pose.position.y);
    double carrot_pose_angle = 0.0;
    
    geometry_msgs::PoseStamped carrot_pose_ackman = getLookAheadPoint(0.23, transformed_local_plan);

    //距离目标点1.0米内
    if( dist_to_goal <= 0.5 /*|| \
    (dist_to_goal <= 2.0 && fabs(tf::getYaw(transformed_local_plan.back().pose.orientation)) > 1.57)*/ )//dxs change <= 1.0
    {
      ROS_INFO("dist to goal < 1.0");
      carrot_pose_ackman = getLookAheadPoint(0.05, transformed_local_plan);
      
      double sign = carrot_pose_ackman.pose.position.x < 0 ? -1 : 1;
      
      carrot_pose_angle = tf::getYaw(carrot_pose_ackman.pose.orientation) * 10;//dxs change 3
      linear_vel = sign * 0.2;

      if(sign < 0)
      {
        carrot_pose_angle = - carrot_pose_angle;
      }
      
    }
    //是否要后退
    else if( carrot_pose_ackman.pose.position.x <= -0.15)
    {
      ROS_INFO("backing ...");
      linear_vel = -0.2;
//      carrot_pose_angle = carrot_pose_ackman.pose.position.y > 0 ? 0.44 : -0.44;
      carrot_pose_angle = - tf::getYaw(carrot_pose_ackman.pose.orientation) * 10;
    }
    
    else
    {
      ROS_INFO("going !");
      carrot_pose_angle = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
      
      //设置线速度  
      applyConstraints(local_path, linear_vel,robot_cur_speed);
      linear_vel = clamp(fabs(linear_vel), 0.0, max_seed_limit_);
    }
    angular_vel = clamp(carrot_pose_angle, -max_angular_vel_, max_angular_vel_);
    ROS_WARN("angular_vel is %f", angular_vel);
  } // ------------------------


  bool isCollision = isCollisionImminent( robot_pose, linear_vel, angular_vel, robot_cur_speed,robot_to_goal_dist);
  if ( isCollision ) 
  {
    ROS_WARN("Detected collision ahead!  linear_vel = %f",linear_vel);
    linear_vel = 0;
//    angular_vel = 0;

    //当前不能前进时，按照原来的角速度旋转
    isCollision = isCollisionImminent( robot_pose, linear_vel, angular_vel, robot_cur_speed,robot_to_goal_dist);  
  }

  if(isCollision) {
    ROS_WARN("Final detected collision ahead!");
    linear_vel = 0;
    angular_vel = 0;
  }

  //四转四驱车型轮子回零
  if(fabs(chassis_angular_) > 17 && car_type_ == 2)
  {
    if(fabs(chassis_linear_) < 0.05 && fabs(linear_vel) >= 0.02)
    {
      linear_vel = 0;
      angular_vel = 0;
    }
  }

  // populate and return message
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return true;
}

bool RegulatedPurePursuitController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome;
}

bool RegulatedPurePursuitController::setNewGoal(const int car_type)
{
  goal_dist_tol_ = goal_dist_tol_temp_;
}

// base
bool RegulatedPurePursuitController::isGoalReached()
{
  if (goal_reached_){
    ROS_INFO("[RegulatedPurePursuitController] Goal Reached!");

    goal_dist_tol_ = goal_dist_tol_temp_;
    return true;
  }
  return false;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

//检测正/侧前方10～20厘米处是否有障碍物
bool RegulatedPurePursuitController::frontHasObs( const geometry_msgs::PoseStamped robot_pose)
{

  nav_msgs::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = "map";
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  std::vector<geometry_msgs::PoseStamped> front_path;

  geometry_msgs::PoseStamped pose_in;
  pose_in.header.frame_id = "/base_link";
  pose_in.header.stamp = ros::Time(0);
  pose_in.pose.position.y = 0.0;
  pose_in.pose.position.z = 0.0;
  pose_in.pose.orientation.w = 1;//robot_pose.pose.orientation; 

  for(int i = 0; i < 8; i ++) {

    pose_in.pose.position.x = i * 0.05;
    front_path.push_back(pose_in);
  }


  for(int i = 0; i < 6; i ++) {

    pose_in.pose.position.y = -i * 0.025;
    pose_in.pose.position.x = -pose_in.pose.position.y;

//    pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(0.785);

    front_path.push_back(pose_in);
  }

  for(int i = 0; i < 6; i ++) {

    pose_in.pose.position.y = i * 0.025;
    pose_in.pose.position.x = pose_in.pose.position.y;

//    pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(-0.785);

    front_path.push_back(pose_in);
  }  

  unsigned char obs_position_num = 0;
  for (int i = 0; i < front_path.size(); i ++/*const auto& pose : front_path*/) 
  {

    geometry_msgs::PoseStamped global_pose;

    try
    {
      tf_->transformPose("/map", front_path[i], global_pose);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Failed to transform path: %s", ex.what());
      return false;
    }

    global_pose.header.frame_id = "/map";

    arc_pts_msg.poses.push_back(global_pose);

    if( inCollision(global_pose.pose.position.x, global_pose.pose.position.y, tf::getYaw(global_pose.pose.orientation)) ) 
    {
//      return true;
      if(i <= 7) obs_position_num |= 1;
      else if(i > 7 && i < 14) obs_position_num |= 2;
      else obs_position_num |= 4;
    }
  }

//  carrot_arc_pub_.publish(arc_pts_msg);
  if(obs_position_num > 1)
  {
    self_rotate_angular_ = 0.5;
  } 

  return obs_position_num == 0 ? false : true;
}

//是否要后退
bool RegulatedPurePursuitController::shouldReversing( const std::vector<geometry_msgs::PoseStamped>& transformed_plan_local)
{
  //已经开始后退变量
  static bool begin_done = false;

  //检查目标点是否在机器人后方
  bool goal_is_at_robot_front = false;
  for (const auto& pose : transformed_plan_local) {
    if( pose.pose.position.x > 0.1 ) {
      goal_is_at_robot_front = true;
      break;
    } 
  }

  if(!goal_is_at_robot_front) return false;

  if(begin_done) {
    //量程回冲
    for (const auto& pose : transformed_plan_local) {
      if( pose.pose.position.x < -0.005 ) {
        begin_done = true;
        return true;
      } 
    }
    begin_done = false;
    return false;
  }

  for (const auto& pose : transformed_plan_local) {

    if( (pose.pose.position.x < -0.06 && transformed_plan_local.back().pose.position.x > 0.1) || begin_done ) {
      begin_done = true;

      geometry_msgs::PoseStamped pose_in, pose_back;
      pose_in.header.frame_id = "/base_link";
      pose_in.header.stamp = ros::Time(0);
      pose_in.pose.position.x = -0.1;
      pose_in.pose.position.y = 0.0;
      pose_in.pose.position.z = 0.0;
      pose_in.pose.orientation = pose.pose.orientation; 

      tf_->transformPose("/map", pose_in, pose_back);
      pose_back.header.frame_id = "/map";

      if( !inCollision(pose_back.pose.position.x, pose_back.pose.position.y, tf::getYaw(pose_back.pose.orientation)) )
      {
        return true;
      }       
    }
  }

  return false;
}

//到达目的地进行原地旋转调整方向
bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);

  if(use_rotate_to_heading_ && (dist_to_goal < goal_dist_tol_))
  {
    goal_dist_tol_ = goal_dist_tol_error_;
    return true;
  }

  return false;
//  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

//原地旋转速度和方向
void RegulatedPurePursuitController::rotateToHeading(double & linear_vel, double & angular_vel,const double & angle_to_path)
{
  linear_vel = 0.0;
  double sign = angle_to_path > 0.0 ? 1.0 : -1.0;

  if( fabs(angle_to_path) > 3.0)
    sign = 1;

  angular_vel = sign * 0.25;
}


//减速处理
double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::Path & transformed_path,
  const geometry_msgs::Twist robot_cur_speed
) const
{

  static double last_velocity_scaling = 1.0;
  static bool big_than_zero_dot_eight = false;
  double approach_velocity_scaling_dist_tem = approach_velocity_scaling_dist_;

  if(robot_cur_speed.linear.x >= 0.95 || big_than_zero_dot_eight) approach_velocity_scaling_dist_tem = approach_velocity_scaling_long_dist_;

  if(robot_cur_speed.linear.x <= 0.1) 
  {
    big_than_zero_dot_eight = false;
    last_velocity_scaling = 1.0;
  }

  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_tem) {

    if(approach_velocity_scaling_dist_tem == approach_velocity_scaling_long_dist_)
    {
      big_than_zero_dot_eight = true;
    }

    auto & last = transformed_path.poses.back();

    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    double return_value = (distance_to_last_pose / approach_velocity_scaling_dist_tem + last_velocity_scaling) / 2;
    last_velocity_scaling = return_value;
    return return_value;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const nav_msgs::Path & path,
  double & linear_vel,
  const geometry_msgs::Twist robot_cur_speed
) const
{
  static double last_linear_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path,robot_cur_speed);

  //如果减速处理后的速度比当前速度要大
  if( linear_vel > last_linear_vel && velocity_scaling < 1 /*&& robot_cur_speed.linear.x >= 0.1*/)
    linear_vel = last_linear_vel;

  //如果减速幅度比较大
  if( last_linear_vel - linear_vel >= 0.1 && velocity_scaling < 1 )
    linear_vel = last_linear_vel - 0.05;

  double approach_vel = linear_vel;

  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

//  ROS_WARN("linear:::  %.3f  %.3f  %.3f",linear_vel,velocity_scaling,approach_vel);

  last_linear_vel = linear_vel;

  linear_vel = std::min(linear_vel, approach_vel);
}

//曲率减速处理
void RegulatedPurePursuitController::applyConstraints(const nav_msgs::Path & path, double & linear_vel, const geometry_msgs::Twist robot_cur_speed)
{
  //计算路径的长度，到达设定距离后进行减速处理
  applyApproachVelocityScaling(path, linear_vel,robot_cur_speed);
}


//Get lookahead point on the global plan
geometry_msgs::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist, const std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.begin(), transformed_plan.end(), [&](const auto & ps) 
  {
    return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
  });

  // If the number of poses is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.end()) {
    goal_pose_it = std::prev(transformed_plan.end());
  }

  return *goal_pose_it;
}


double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}


bool RegulatedPurePursuitController::transformGlobalPlan(
  const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
  const tf::Stamped<tf::Pose>& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, tf::StampedTransform* tf_plan_to_global)
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    tf::StampedTransform plan_to_global_transform;
    tf.waitForTransform(global_frame, ros::Time::now(),
    plan_pose.header.frame_id, plan_pose.header.stamp,
    plan_pose.header.frame_id, ros::Duration(0.5));
    tf.lookupTransform(global_frame, ros::Time(),
    plan_pose.header.frame_id, plan_pose.header.stamp, 
    plan_pose.header.frame_id, plan_to_global_transform);

    //let's get the pose of the robot in the frame of the plan
    tf::Stamped<tf::Pose> robot_pose;
    tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                    costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                          // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.getOrigin().x() - global_plan[j].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
      }
    }
    
    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && /*sq_dist <= sq_dist_threshold &&*/ (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf::poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += std::sqrt( std::pow(global_plan[i].pose.position.x - global_plan[i-1].pose.position.x,2) + 
                                      std::pow(global_plan[i].pose.position.y - global_plan[i-1].pose.position.y,2) );

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
      tf_pose.setData(plan_to_global_transform * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);
      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}


bool RegulatedPurePursuitController::pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
      return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    tf::StampedTransform global_to_plan_transform;
    tf.lookupTransform(global_plan.front().header.frame_id, global_pose.frame_id_, ros::Time(0), global_to_plan_transform);
    tf::Stamped<tf::Pose> robot;
    robot.setData( global_to_plan_transform * global_pose );
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.getOrigin().x() - it->pose.position.x;
      double dy = robot.getOrigin().y() - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}



double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    ROS_ERROR("RegulatedPurePursuitController: Dimensions of the costmap are too small "
                  "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

bool RegulatedPurePursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    ROS_WARN_THROTTLE(1.0, "The dimensions of the costmap is too small to successfully check for "
    "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
    "increase your costmap size.");
    return true;
  }

  double footprint_cost = costmap_model_->footprintCost(
      x, y, theta, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

  return footprint_cost < 0;
}

//安全检测
bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const geometry_msgs::Twist robot_cur_speed,
  const double robot_to_goal_dist)
{
  // check current point is OK
  if (inCollision(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }

  // visualization messages
  nav_msgs::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = "map";
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;
  

  geometry_msgs::Pose2D curr_pose;
  curr_pose.theta = tf::getYaw(robot_pose.pose.orientation);

  //最小距离检测
  for(int i = 0; i < 13 && robot_cur_speed.linear.x > 0.55 && robot_to_goal_dist > 1.0; i ++) {

    curr_pose.x = robot_pose.pose.position.x + 0.025 * i * cos(curr_pose.theta);
    curr_pose.y = robot_pose.pose.position.y + 0.025 * i * sin(curr_pose.theta);
    curr_pose.theta = tf::getYaw(robot_pose.pose.orientation);// + (0.0 * i) * angular_vel;

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {

      ROS_WARN("linear  > 0.5   front  has  obs  stop !!!");
      return true;
    }
  }

  //control_duration_ control_frequency
  int count = 8;
//  if(linear_vel <= 0.5) count = 8;
  for(int i = 0; i < count && linear_vel != 0; i ++) {

    curr_pose.x = robot_pose.pose.position.x + (0.1 * i) * linear_vel * cos(curr_pose.theta);
    
    if(car_type_ <= 2)
    {
      curr_pose.y = robot_pose.pose.position.y + (0.1 * i) * linear_vel * sin(curr_pose.theta);
      curr_pose.theta = tf::getYaw(robot_pose.pose.orientation) + (0.1 * i) * angular_vel;//robot_cur_speed.angular.z;
    }
      
    else
    {
      curr_pose.y = robot_pose.pose.position.y + (0.1 * i) * linear_vel * sin(curr_pose.theta);
      curr_pose.theta = tf::getYaw(robot_pose.pose.orientation) + (0.1 * i) * linear_vel * tan(angular_vel) / 0.6;;
    }

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      carrot_arc_pub_.publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_.publish(arc_pts_msg);

  return false;
}


/**
 * Helper methods
 */

void RegulatedPurePursuitController::createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path){

  path.header = plan[0].header;
  for (int i = 0; i < plan.size(); i++){
    path.poses.push_back(plan[i]);
  }
}

geometry_msgs::PointStamped RegulatedPurePursuitController::createCarrotMsg( const geometry_msgs::PoseStamped & carrot_pose)
{
  geometry_msgs::PointStamped carrot_msg ;
  carrot_msg.header = carrot_pose.header;
  carrot_msg.point.x = carrot_pose.pose.position.x;
  carrot_msg.point.y = carrot_pose.pose.position.y;
  carrot_msg.point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

//Get the size of the costmap
double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersX());

  return max_costmap_dim_meters / 2.0;
}

void RegulatedPurePursuitController::getRobotVel(geometry_msgs::Twist& speed){
  nav_msgs::Odometry robot_odom;

  odom_helper_.getOdom(robot_odom);

  speed.linear.x = robot_odom.twist.twist.linear.x;
  speed.angular.z = robot_odom.twist.twist.angular.z;
}


}
