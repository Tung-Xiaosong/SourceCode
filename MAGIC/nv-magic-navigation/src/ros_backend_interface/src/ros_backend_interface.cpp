#include <ros_backend_interface/ros_backend_interface.h>


namespace ros_backend_interface {

  RosBackendInterface::RosBackendInterface(tf::TransformListener& tf) :
  tf_(tf),
  map_init_(false),
  running_(true),
  init_pose_is_success_(false),
  ac_(NULL),
  cmd_vel_is_pause_(false),
  fw_ctrl_cmd_is_pause_(false),
  resolution_(0){

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    //订阅scan数据
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &RosBackendInterface::scan_CB,this);

    //订阅地图数据
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &RosBackendInterface::map_CB,this);

    //订阅底盘数据
    private_nh.param("/common/chassis_type", chassis_type_, std::string("FW"));
    if(chassis_type_ == "DGT")
    {
      dgt_chassis_info_sub_ = nh.subscribe<yhs_msgs::DgtChassisInfoFb>("chassis_info_fb", 10, &RosBackendInterface::DgtChassisCB,this);  
    }
    else if(chassis_type_ == "FW")
    {
      fw_chassis_info_sub_ = nh.subscribe<yhs_msgs::FwChassisInfoFb>("chassis_info_fb", 10, &RosBackendInterface::FwChassisCB,this); 
      fw_ctrl_cmd_sub_ = nh.subscribe<yhs_msgs::FwCtrlCmd>("fw_ctrl_cmd", 10, &RosBackendInterface::fw_ctrl_cmd_CB,this);
      fw_ctrl_cmd_pub_ = nh.advertise<yhs_msgs::FwCtrlCmd>("ctrl_cmd", 10);
    }
    else if(chassis_type_ == "MK")
    {
      mk_chassis_info_sub_ = nh.subscribe<yhs_msgs::MkChassisInfoFb>("chassis_info_fb", 10, &RosBackendInterface::MkChassisCB,this); 
    }
    else if(chassis_type_ == "FR")
    {
      fr_chassis_info_sub_ = nh.subscribe<yhs_msgs::FrChassisInfoFb>("chassis_info_fb", 10, &RosBackendInterface::FrChassisCB,this); 
    }
    else
    {
    }

    //订阅cartographer路径来判断定位状态
    marker_array_sub_ = nh.subscribe("constraint_list", 10, &RosBackendInterface::markerArray_CB,this);

    //订阅里程计，判断是否正常
    odom_sub_ = nh.subscribe("odom", 10, &RosBackendInterface::odom_CB,this);

    //订阅IMU，判断是否正常
    imu_sub_ = nh.subscribe("imu_data", 10, &RosBackendInterface::imu_CB,this);

    //订阅超声波，判断是否正常
    ultrasonic_sub_ = nh.subscribe("ultrasonic", 10, &RosBackendInterface::ultrasonic_CB,this);

    //局部路径
    path_nav_grid_sub_ = nh.subscribe("move_base/TebLocalPlannerROS/local_plan", 1,  &RosBackendInterface::pathNav_CB,this);

    //导航速度
    cmd_vel_nav_sub_ = nh.subscribe("cmd_vel_nav", 10, &RosBackendInterface::cmd_vel_nav_CB,this);

    //APP速度
    cmd_vel_app_sub_ = nh.subscribe("cmd_vel_app", 10, &RosBackendInterface::cmd_vel_app_CB,this);

    //斜坡区域
    set_region_sub_ = nh.subscribe("set_region", 1, &RosBackendInterface::set_region_CB, this);

    //栅格化激光数据
    scan_to_grid_pub_ = nh.advertise<geometry_msgs::PoseArray>("grid_scan", 1);

    //机器人实时位置
    robot_position_pub_ = nh.advertise<yhs_msgs::RobotPosition>("robot_position", 1);

    //机器人硬件状态
    robot_hardwre_status_pub_ = nh.advertise<yhs_msgs::RobotHardwreStatus>("robot_hardwre_status", 1);

    //版本
    version_pub_ = nh.advertise<yhs_msgs::Version>("version", 1);

    //定位状态
    robot_location_status_pub_ = nh.advertise<std_msgs::Bool>("robot_location_status", 1);

    //导航类型
    nav_tye_pub_ = nh.advertise<std_msgs::UInt8>("nav_type", 1);

    //手绘或者录制路径
    record_path_pub_ = nh.advertise<nav_msgs::Path>("recorded_path", 1);

    //app控制速度
    cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

    //是否处于斜坡区域
    in_slope_region_pub_ = nh.advertise<std_msgs::Bool>("is_in_slope_region", 1);

    //是否处于减速区域
    in_slow_region_pub_ = nh.advertise<std_msgs::Bool>("is_in_slow_region", 1);

    //是否处于窄道区域
    in_narrow_region_pub_ = nh.advertise<std_msgs::Bool>("is_in_narrow_region", 1);

    //初始化点
    init_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    //栅格化局部路径
    grid_nav_path_pub_ = nh.advertise<geometry_msgs::PoseArray>("grid_nav_path", 1);

    //取消回充
    cancel_pub_ = nh.advertise<std_msgs::Bool>("recharge_cancel", 1);

    //与后端对接目标点服务
    goal_ser_ = nh.advertiseService("move_base_simple/goal", &RosBackendInterface::goal_CB,this);

    //与后端对接初始化服务
    initpose_ser_ = nh.advertiseService("initialpose", &RosBackendInterface::initialPose_CB,this);

    //与后端对接暂停服务
    goal_pause_ser_ = nh.advertiseService("move_base/pause", &RosBackendInterface::Pause_CB,this);

    //与后端对接取消服务
    goal_cancel_ser_ = nh.advertiseService("move_base/cancel", &RosBackendInterface::Cancel_CB,this);

    //创建检查路径客户端对象
    client_ = nh.serviceClient<yhs_msgs::PathIsValid>("/move_base/check_path");

    ac_ = new MoveBaseClient("move_base", true);

    //thread
    run_thread_ = std::thread(&RosBackendInterface::dataTrans, this);

    }

    //stop
    RosBackendInterface::~RosBackendInterface(){
      stop();
    }

    //停止
    void RosBackendInterface::stop() {
      if (running_) {
        running_ = false;
        run_thread_.join();
      }
      if(ac_ != NULL) delete ac_;
    }


    void RosBackendInterface::fw_ctrl_cmd_CB(const yhs_msgs::FwCtrlCmd::ConstPtr& fw_ctrl_cmd__msg)
    {
      yhs_msgs::FwCtrlCmd msg;
      if(!fw_ctrl_cmd_is_pause_)
        msg = *fw_ctrl_cmd__msg;

      fw_ctrl_cmd_pub_.publish(msg);     
    }


    // 里程计回调函数
    void RosBackendInterface::odom_CB(const nav_msgs::Odometry::ConstPtr& odm_msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_data_last_time_ = ros::Time::now();
    }

    // IMU回调函数
    void RosBackendInterface::imu_CB(const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      imu_data_last_time_ = ros::Time::now();
    }

    // 超声波回调函数
    void RosBackendInterface::ultrasonic_CB(const yhs_msgs::Ultrasonic::ConstPtr& ultrasonic_msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ultrasonic_data_last_time_ = ros::Time::now();
    }

    // 判断点是否在多边形内部
    inline bool RosBackendInterface::pointInPolygon(const geometry_msgs::PointStamped& point, const geometry_msgs::PoseArray& polygon)
    {
      int n = polygon.poses.size();
      bool inside = false;
      for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon.poses[i].position.y <= point.point.y) && (point.point.y < polygon.poses[j].position.y)) ||
            ((polygon.poses[j].position.y <= point.point.y) && (point.point.y < polygon.poses[i].position.y))) {
          if (point.point.x < (polygon.poses[j].position.x - polygon.poses[i].position.x) * (point.point.y - polygon.poses[i].position.y) /
                              (polygon.poses[j].position.y - polygon.poses[i].position.y) + polygon.poses[i].position.x) {
            inside = !inside;
          }
        }
      }
      return inside;
    }

    //斜坡区域回调函数
    void RosBackendInterface::set_region_CB(const yhs_msgs::SetRegionConstPtr &region_msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);

      for(int i = 0; i < region_msg->slope_area.size(); i++)
      {
        slope_region_.push_back(region_msg->slope_area[i]);
      }

      for(int i = 0; i < region_msg->slow_area.size(); i++)
      {
        slow_region_.push_back(region_msg->slow_area[i]);
      }

      for(int i = 0; i < region_msg->narrow_area.size(); i++)
      {
        narrow_region_.push_back(region_msg->narrow_area[i]);
      }
    }

    //栅格化路径
    void RosBackendInterface::pathNav_CB(const nav_msgs::Path::ConstPtr& path_msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);

      geometry_msgs::PoseArray pose_array;
      pose_array.header = path_msg->header;

      // 将路径消息转换为位姿数组
      for (size_t i = 0; i < path_msg->poses.size(); ++i)
      {
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header = path_msg->header;
        pose_in.pose = path_msg->poses[i].pose;

        double wx = path_msg->poses[i].pose.position.x;
        double wy = path_msg->poses[i].pose.position.y;

        int x = std::floor((wx - origin_position_.x) / resolution_);
        int y = std::floor((wy - origin_position_.y) / resolution_);

        // 将转换后的栅格地图坐标封装成位姿并添加到位姿数组中
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation = path_msg->poses[i].pose.orientation;

        pose_array.poses.push_back(pose);
      }

      grid_nav_path_pub_.publish(pose_array);
    }

    //数据传输
    void RosBackendInterface::dataTrans(){

      tf::TransformListener listener;
      int mx,my;
      double yaw;

      while (running_ && ros::ok()) {

        if(map_init_){

          tf::StampedTransform transform;
          try
          {
            std::lock_guard<std::mutex> lock(mutex_);

            listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(5));

            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

            mx = std::floor((transform.getOrigin().x() - origin_position_.x) / resolution_);
            my = std::floor((transform.getOrigin().y() - origin_position_.y) / resolution_);

            robot_position_.pose.position.x = transform.getOrigin().x();
            robot_position_.pose.position.y = transform.getOrigin().y();
            
            robot_position_.pose.orientation.z = transform.getRotation().z();
            robot_position_.pose.orientation.w = transform.getRotation().w();
            robot_position_.pose.orientation.x = transform.getRotation().x();
            robot_position_.pose.orientation.y = transform.getRotation().y();

            yaw = tf::getYaw(transform.getRotation());
          }
          catch (tf::TransformException& ex)
          {
            ROS_ERROR_STREAM("Failed to get robot pose: " << ex.what());
          }

          // 判断点是否在斜坡或者减速多边形内部
          std_msgs::Bool slope_region_msg;
          slope_region_msg.data = false;

          std_msgs::Bool slow_region_msg;
          slow_region_msg.data = false;

          std_msgs::Bool narrow_region_msg;
          narrow_region_msg.data = false;

          for(int i = 0; i < slope_region_.size(); i++)
          {
            geometry_msgs::PointStamped map_point;
            map_point.point.x = transform.getOrigin().x();
            map_point.point.y = transform.getOrigin().y();

            bool inside = pointInPolygon(map_point, slope_region_[i]);
            if(inside)
            {
              slope_region_msg.data = true;
              break;
            }
          }

          for(int i = 0; i < slow_region_.size(); i++)
          {
            geometry_msgs::PointStamped map_point;
            map_point.point.x = transform.getOrigin().x();
            map_point.point.y = transform.getOrigin().y();

            bool inside = pointInPolygon(map_point, slow_region_[i]);
            if(inside)
            {
              slow_region_msg.data = true;
              break;
            }
          }

          for(int i = 0; i < narrow_region_.size(); i++)
          {
            geometry_msgs::PointStamped map_point;
            map_point.point.x = transform.getOrigin().x();
            map_point.point.y = transform.getOrigin().y();

            bool inside = pointInPolygon(map_point, narrow_region_[i]);
            if(inside)
            {
              narrow_region_msg.data = true;
              break;
            }
          }

          in_slope_region_pub_.publish(slope_region_msg);
          in_slow_region_pub_.publish(slow_region_msg);
          in_narrow_region_pub_.publish(narrow_region_msg);
          
          //发布世界坐标和栅格坐标
          yhs_msgs::RobotPosition robot_position_msg;
          robot_position_msg.header.stamp = ros::Time::now();

          robot_position_msg.world_position.x = transform.getOrigin().x();
          robot_position_msg.world_position.y = transform.getOrigin().y();
          robot_position_msg.world_position.theta = yaw;

          robot_position_msg.grid_position.x = mx;
          robot_position_msg.grid_position.y = my;
          robot_position_msg.grid_position.theta = yaw;

          robot_position_pub_.publish(robot_position_msg);

          std_msgs::Bool robot_status_msg;
          robot_status_msg.data = init_pose_is_success_;

          //发布定位状态
          robot_location_status_pub_.publish(robot_status_msg);

          //发布版本
          yhs_msgs::Version version_msg;
          version_msg.type = 1;
          version_msg.chassis_version = "1.0.0";
          version_msg.navigation_version = "DGT-1.0.1";
          version_pub_.publish(version_msg);

          //判断定位丢失
          static float world_position_x_last = robot_position_msg.world_position.x;
          static float world_position_y_last = robot_position_msg.world_position.y;

          float dx = robot_position_msg.world_position.x - world_position_x_last;
          float dy = robot_position_msg.world_position.y - world_position_y_last;

          if(sqrt(dx*dx + dy*dy) > 1.0) init_pose_is_success_ = false;

          world_position_x_last = robot_position_msg.world_position.x;
          world_position_y_last = robot_position_msg.world_position.y;
        }
        ros::Duration(0.1).sleep();

        //发布底盘信息
        yhs_msgs::RobotHardwreStatus pub_msg;

        //判断数据是否有效
        ros::Duration timeout(1);
        std::lock_guard<std::mutex> lock(mutex_);
        if( ros::Time::now() - odom_data_last_time_ <= timeout)
        {
          pub_msg.odom_status = true;
        }

        if( ros::Time::now() - imu_data_last_time_ <= timeout)
        {
          pub_msg.imu_status = true;
        }

        if( ros::Time::now() - lidar_data_last_time_ <= timeout)
        {
          pub_msg.lidar_status = true;
        }

        if( ros::Time::now() - ultrasonic_data_last_time_ <= timeout)
        {
          pub_msg.ultrasonic_status = true;
        }

        pub_msg.type = 1;
        pub_msg.lidar_num = 1;
        pub_msg.camera_num = 1;
        pub_msg.ultrasonic_num = 0;
        pub_msg.gps_num = 0;

        pub_msg.emergency_stop_status = chassis_msg_.emergency_stop_status;
        pub_msg.battery_percentage = chassis_msg_.battery_percentage;
        pub_msg.charge_status = chassis_msg_.charge_status;
  
        pub_msg.anticollision_status = chassis_msg_.anticollision_status;
        pub_msg.error_code = 0;
        pub_msg.io_status = 0;

        robot_hardwre_status_pub_.publish(pub_msg); 
      }
    }

    //DGT底盘信息
    void RosBackendInterface::DgtChassisCB(const yhs_msgs::DgtChassisInfoFb::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      chassis_msg_.emergency_stop_status = msg->io_fb.io_fb_estop;
      chassis_msg_.battery_percentage = msg->bms_flag_fb.bms_flag_fb_soc;
      chassis_msg_.charge_status = msg->io_fb.io_fb_charge_state;
      
      chassis_msg_.anticollision_status = msg->io_fb.io_fb_fm_impact_sensor;
    }
    
    //FW底盘信息
    void RosBackendInterface::FwChassisCB(const yhs_msgs::FwChassisInfoFb::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      chassis_msg_.emergency_stop_status = msg->io_fb.io_fb_estop;
      chassis_msg_.battery_percentage = msg->bms_flag_fb.bms_flag_fb_soc;
      chassis_msg_.charge_status = msg->io_fb.io_fb_charge_state;
      
      chassis_msg_.anticollision_status = msg->io_fb.io_fb_fm_impact_sensor;
    }
    
    //MK底盘信息
    void RosBackendInterface::MkChassisCB(const yhs_msgs::MkChassisInfoFb::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      chassis_msg_.emergency_stop_status = msg->veh_diag_fb.veh_fb_aux_scram;
      chassis_msg_.battery_percentage = msg->bms_flag_info_fb.bms_flag_info_soc;
      chassis_msg_.charge_status = msg->bms_flag_info_fb.bms_flag_info_charge_flag;
      
      chassis_msg_.anticollision_status = msg->io_fb.io_fb_fm_impact_sensor;
    }
    
    //FR底盘信息
    void RosBackendInterface::FrChassisCB(const yhs_msgs::FrChassisInfoFb::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      chassis_msg_.emergency_stop_status = msg->veh_diag_fb.veh_fb_aux_scram;
      chassis_msg_.battery_percentage = msg->bms_flag_info_fb.bms_flag_info_soc;
      chassis_msg_.charge_status = msg->bms_flag_info_fb.bms_flag_info_charge_flag;
      
      chassis_msg_.anticollision_status = msg->io_fb.io_fb_fm_impact_sensor;
    }

    //激光数据
    void RosBackendInterface::scan_CB(const sensor_msgs::LaserScan::ConstPtr& msg){

      std::lock_guard<std::mutex> lock(mutex_);
      lidar_data_last_time_ = ros::Time::now();

      if(!map_init_) return;

      sensor_msgs::PointCloud2 cloud;

      projector_.transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud, tf_);

      cloud.header.stamp = ros::Time();

      pcl::PointCloud<pcl::PointXYZ> rawCloud, global_frame_cloud;
      pcl::fromROSMsg(cloud, rawCloud);


      pcl_ros::transformPointCloud("map", rawCloud, global_frame_cloud, tf_);

      geometry_msgs::PoseArray pose_array;
      pose_array.header.stamp = ros::Time::now();
      pose_array.header.frame_id = "map";

      for (unsigned int i = 0; i < global_frame_cloud.points.size(); ++i) {

        double wx = global_frame_cloud.points[i].x;
        double wy = global_frame_cloud.points[i].y;

        int x = std::floor((wx - origin_position_.x) / resolution_);
        int y = std::floor((wy - origin_position_.y) / resolution_);

        // 将转换后的栅格地图坐标封装成位姿并添加到位姿数组中
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        pose_array.poses.push_back(pose);

      }
      scan_to_grid_pub_.publish(pose_array);
    }

    //地图数据
    void RosBackendInterface::map_CB(const nav_msgs::OccupancyGrid::ConstPtr& msg){

      std::lock_guard<std::mutex> lock(mutex_);
      map_init_ = true;
      running_ = true;
      resolution_ = msg->info.resolution;
      origin_position_ = msg->info.origin.position;
    }

    void RosBackendInterface::markerArray_CB(const visualization_msgs::MarkerArray::ConstPtr& msg){

      std::lock_guard<std::mutex> lock(mutex_);
      if(msg->markers[4].points.size() > 0)
      {
        init_pose_is_success_ = true;
        for(int i = 0; i < msg->markers[4].points.size(); i ++)
        {
//        ROS_INFO_STREAM("\t(" << msg->markers[4].points[i].x << ", " << msg->markers[4].points[i].y << ", " << msg->markers[4].points[i].z << ")");
        }
      }

    }

    //初始化点数据
    bool RosBackendInterface::initialPose_CB(yhs_msgs::InitialPose::Request &req,  yhs_msgs::InitialPose::Response &resp){

      resp.result = 1;
      if(init_pose_is_success_) return true;

      geometry_msgs::Twist cmd;
      cmd.angular.z = 0.3;

      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";

      pose.pose = req.goal.pose;
      init_pose_pub_.publish(pose);
      
      //计算时间
      ros::Time rotate_start = ros::Time::now();
      ros::Time has_obs_start = ros::Time::now();
      while(!init_pose_is_success_)
      {
        //动态初始化
        if(req.type == 1)
        {
          yhs_msgs::PathIsValid path;
          std::lock_guard<std::mutex> lock(mutex_);
          for(int i = -2; i < 3; i ++)
          {
            geometry_msgs::PoseStamped point;
            point.pose.orientation = robot_position_.pose.orientation;
            point.pose.position.x = robot_position_.pose.position.x + 0.05 * i;
            point.pose.position.y = robot_position_.pose.position.y + 0.05 * i;
            path.request.checked_path.poses.push_back(point);
          }
          if (client_.call(path))
          {
            //没有障碍物
            if(path.response.result == 1 )
            {
              cmd.angular.z = 0.3;
              if(chassis_type_ == "FR" || chassis_type_ == "MK")
              {
                cmd.angular.z = 0.5;
                cmd.linear.x = 0.1;
              }
              has_obs_start = ros::Time::now();
            }
            //有障碍物
            else
            {
              cmd.angular.z = 0.0;
            }
          } 
          else 
          {
            ROS_ERROR("Failed to call service %s", client_.getService().c_str());
          }

          double time_length = (ros::Time::now() - has_obs_start).toSec();
          if(time_length > 5) 
          {
            break;
          }

          geometry_msgs::TwistStamped cmd_vel_pub;
          cmd_vel_pub.header.stamp = ros::Time::now();
          cmd_vel_pub.twist = cmd;
          cmd_vel_pub_.publish(cmd_vel_pub);

//          cmd_vel_pub_.publish(cmd);
        }
          
        usleep(1000 * 33);
        double time_length = (ros::Time::now() - rotate_start).toSec();

        if(time_length > 20) break;
      }

      resp.result = init_pose_is_success_;
      return true;
    }

    //导航点和路径点数据
    bool RosBackendInterface::goal_CB(yhs_msgs::Goal::Request &req,  yhs_msgs::Goal::Response &resp){

      const int8_t type = req.type;
      geometry_msgs::PoseStamped goal = req.goal;
      nav_msgs::Path graph_path = req.graph_path;
      nav_msgs::Path record_path = req.record_path;

      while(!ac_->waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      std_msgs::UInt8 nav_type_msg;
      nav_type_msg.data = type;

      //发布导航类型
      nav_tye_pub_.publish(nav_type_msg);


      //自由导航
      if(type == 0)
      {
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose = goal.pose;
        
        ac_->sendGoal(goal_msg);
        ac_->waitForResult();

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          resp.result = 3;
        if(ac_->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
          resp.result = 2;
        if(ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
          resp.result = 4;
      }

      //手绘路径
      else if (type == 1)
      {
        if(graph_path.poses.size() < 2)
        {
          resp.result = 4;
          return true;
        }

        //导航到路径最后一个点
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";

        graph_path.header.frame_id = "map";
        graph_path.header.stamp = ros::Time::now();
        record_path_pub_.publish(graph_path);

        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose =graph_path.poses[graph_path.poses.size() -1].pose;
        
        ac_->sendGoal(goal_msg);
        ac_->waitForResult();

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          resp.result = 3;
        if(ac_->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
          resp.result = 2;
        if(ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
          resp.result = 4;
      }

      //录制路径
      else if (type == 2)
      {
        if(record_path.poses.size() < 2)
        {
          resp.result = 4;
          return true;
        }

        //导航到路径最后一个点        
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";

        record_path.header.frame_id = "map";
        record_path.header.stamp = ros::Time::now();
        record_path_pub_.publish(record_path);

        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose =record_path.poses[record_path.poses.size() -1].pose;
        
        ac_->sendGoal(goal_msg);
        ac_->waitForResult();

        //1：导航中  2：导航取消 3：到达 4：不能到达

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          resp.result = 3;
        if(ac_->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
          resp.result = 2;
        if(ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
          resp.result = 4;
      }
      else
      {
      }
      
      return true;
    }

    //暂停 恢复
    bool RosBackendInterface::Pause_CB(yhs_msgs::Pause::Request &req,  yhs_msgs::Pause::Response &resp){

      //暂停
      if(req.pause == 1)
      {
        cmd_vel_is_pause_ = true;
        fw_ctrl_cmd_is_pause_ = true;
      }
      
      //恢复
      else 
      {
        cmd_vel_is_pause_ = false;
        fw_ctrl_cmd_is_pause_ = false;
      }

      resp.result = 1;
      return true;
    }

    //取消
    bool RosBackendInterface::Cancel_CB(yhs_msgs::Cancel::Request &req,  yhs_msgs::Cancel::Response &resp){

      ROS_WARN("navigation to goal cancel!");

      if(req.cancel != 1) return true;

      //取消后，暂停位全部清除
      cmd_vel_is_pause_ = false;
      fw_ctrl_cmd_is_pause_ = false;

      //给充电节点发送取消数据
      std_msgs::Bool msg;
      msg.data = true;
      cancel_pub_.publish(msg);

      while(!ac_->waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal_msg;
      ac_->sendGoal(goal_msg);

//      ac_->cancelGoal();
      ac_->waitForResult();

      if(ac_->getState() == actionlib::SimpleClientGoalState::PREEMPTED || ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
        resp.result = 2;

      return true;
    }

    //导航速度
    void RosBackendInterface::cmd_vel_nav_CB(const geometry_msgs::TwistStamped::ConstPtr& msg){

      cmd_vel_nav_ = msg->twist;

      if(cmd_vel_is_pause_) 
      {
        cmd_vel_nav_.linear.x = 0.0;
        cmd_vel_nav_.angular.z = 0.0;
      }

      geometry_msgs::TwistStamped cmd_vel_pub;
      cmd_vel_pub.header.stamp = ros::Time::now();
      cmd_vel_pub.twist = cmd_vel_nav_;
      cmd_vel_pub_.publish(cmd_vel_pub);

//      cmd_vel_pub_.publish(cmd_vel_nav_);
    }

    //app速度
    void RosBackendInterface::cmd_vel_app_CB(const geometry_msgs::Twist::ConstPtr& msg){
      
      cmd_vel_app_ = *msg;

      geometry_msgs::TwistStamped cmd_vel_pub;
      cmd_vel_pub.header.stamp = ros::Time::now();
      cmd_vel_pub.twist = cmd_vel_app_;
      cmd_vel_pub_.publish(cmd_vel_pub);

//      cmd_vel_pub_.publish(cmd_vel_app_);
    }

};
