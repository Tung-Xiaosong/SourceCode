
#ifndef ROS_BACKEND_INTERFACE_H
#define ROS_BACKEND_INTERFACE_H

#include <vector>
#include <string>
#include <thread>
#include <mutex>


//system
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#define CAR_TYPE 4   //1: DGt  2: FW  3: FR  4: MK

#if CAR_TYPE == 1
  #include "yhs_msgs/DgtChassisInfoFb.h" 
  #define chassis_msg_type  yhs_msgs::DgtChassisInfoFb
#elif CAR_TYPE == 2
  #include "yhs_msgs/FwChassisInfoFb.h" 
  #include "yhs_msgs/FwCtrlCmd.h"
  #define chassis_msg_type  yhs_msgs::FwChassisInfoFb
#elif CAR_TYPE == 3
  #include "yhs_msgs/FrChassisInfoFb.h" 
  #define chassis_msg_type  yhs_msgs::FrChassisInfoFb
#else
  #include "yhs_msgs/MkChassisInfoFb.h" 
  #define chassis_msg_type  yhs_msgs::MkChassisInfoFb
#endif

//custom
#include "yhs_msgs/RobotPosition.h"
#include "yhs_msgs/RobotHardwreStatus.h"
#include "yhs_msgs/Version.h"
#include "yhs_msgs/Goal.h"
#include "yhs_msgs/InitialPose.h"
#include "yhs_msgs/Pause.h"
#include "yhs_msgs/Cancel.h"
#include "yhs_msgs/SetRegion.h"
#include "yhs_msgs/Ultrasonic.h"
#include "yhs_msgs/PathIsValid.h"


namespace ros_backend_interface {
  
  class RosBackendInterface {

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    public:
     
      RosBackendInterface(tf::TransformListener& tf);

      virtual ~RosBackendInterface();

    private:

      void chassis_CB(const chassis_msg_type::ConstPtr& chassis_info_fb_msg);
      void scan_CB(const sensor_msgs::LaserScan::ConstPtr& msg);
      void map_CB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
      void initialpose_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
      void markerArray_CB(const visualization_msgs::MarkerArray::ConstPtr& msg);
      void cmd_vel_nav_CB(const geometry_msgs::TwistStamped::ConstPtr& msg);
      void cmd_vel_app_CB(const geometry_msgs::Twist::ConstPtr& msg);

      void set_region_CB(const yhs_msgs::SetRegionConstPtr &region_msg);

      bool goal_CB(yhs_msgs::Goal::Request &req,  yhs_msgs::Goal::Response &resp);
      bool initialPose_CB(yhs_msgs::InitialPose::Request &req,  yhs_msgs::InitialPose::Response &resp);
      bool Pause_CB(yhs_msgs::Pause::Request &req,  yhs_msgs::Pause::Response &resp);
      bool Cancel_CB(yhs_msgs::Cancel::Request &req,  yhs_msgs::Cancel::Response &resp);

      void pathNav_CB(const nav_msgs::Path::ConstPtr& path_msg);

      inline bool pointInPolygon(const geometry_msgs::PointStamped& point, const geometry_msgs::PoseArray& polygon);

      void odom_CB(const nav_msgs::Odometry::ConstPtr& odm_msg);
      void imu_CB(const sensor_msgs::Imu::ConstPtr& imu_msg);
      void ultrasonic_CB(const yhs_msgs::Ultrasonic::ConstPtr& ultrasonic_msg);

 
      tf::TransformListener& tf_;
      laser_geometry::LaserProjection projector_;

      ros::ServiceServer goal_ser_,initpose_ser_,goal_pause_ser_,goal_cancel_ser_;

      ros::Publisher chassis_pub_, scan_to_grid_pub_,robot_position_pub_, robot_hardwre_status_pub_,\
      version_pub_,robot_location_status_pub_,record_path_pub_,cmd_vel_pub_,init_pose_pub_,grid_nav_path_pub_,\
      in_slope_region_pub_,in_slow_region_pub_,nav_tye_pub_,cancel_pub_,in_narrow_region_pub_;

      ros::Subscriber scan_sub_,map_sub_,chassis_sub_,init_pose_sub_,goal_sub_,\
      marker_array_sub_,cmd_vel_nav_sub_,cmd_vel_app_sub_,path_nav_grid_sub_,odom_sub_,\
      set_region_sub_,imu_sub_,ultrasonic_sub_;

      //map infomation
      bool map_init_;
      float resolution_;
      geometry_msgs::Point origin_position_;

      //thread
      void stop();
      void dataTrans();

      //mutex
      std::mutex mutex_;

      std::thread run_thread_;
      bool running_;

      bool init_pose_is_success_;
      bool cmd_vel_is_pause_;

      MoveBaseClient *ac_;

      geometry_msgs::Twist cmd_vel_app_,cmd_vel_nav_;

      //机器人实时位置
      geometry_msgs::PoseStamped robot_position_;

      //路径检查客户端对象
      ros::ServiceClient client_;

      //斜坡区域
      std::vector<geometry_msgs::PoseArray> slope_region_;

      //减速区域
      std::vector<geometry_msgs::PoseArray> slow_region_;

      //窄道区域
      std::vector<geometry_msgs::PoseArray> narrow_region_;

      //状态判断
      ros::Time odom_data_last_time_;
      ros::Time imu_data_last_time_;
      ros::Time lidar_data_last_time_;
      ros::Time cameras_data_last_time_;
      ros::Time ultrasonic_data_last_time_;

      //底盘msg数据
      chassis_msg_type sub_msg_;

      //FW车型回充暂停标志位
      bool fw_ctrl_cmd_is_pause_;


      //FW车型
      #if CAR_TYPE == 2
        ros::Subscriber fw_ctrl_cmd_sub_;
        ros::Publisher fw_ctrl_cmd_pub_;

        void fw_ctrl_cmd_CB(const yhs_msgs::FwCtrlCmd::ConstPtr& fw_ctrl_cmd__msg);
      #endif

  };
};



#endif // ROS_BACKEND_INTERFACE_H


