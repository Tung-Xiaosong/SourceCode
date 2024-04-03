#ifndef REGULATED_PURE_PURSUIT_CONTROLLER_H
#define REGULATED_PURE_PURSUIT_CONTROLLER_H

#include <algorithm>

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <tf2/utils.h>
// transforms
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

// #include <regulated_pure_pursuit_controller/geometry_utils.h>
#include <geometry_msgs/Pose2D.h>

#include <mbf_costmap_core/costmap_controller.h>
#include <mbf_msgs/ExePathResult.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include "regulated_pure_pursuit_controller/geometry_utils.h"


namespace regulated_pure_pursuit_controller{
class RegulatedPurePursuitController : public nav_core::BaseLocalPlanner/*, public mbf_costmap_core::CostmapController*/{

public:

  RegulatedPurePursuitController();

  ~RegulatedPurePursuitController(){
    delete costmap_model_;
  };

  /**
   * Initialization methods
   */

  void initParams(ros::NodeHandle& nh);

  void initPubSubSrv(ros::NodeHandle& nh);


  void initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * Inherited virtual methods from BaseLocalPlanner
   */

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                      const geometry_msgs::TwistStamped& velocity,
                                                      geometry_msgs::TwistStamped &cmd_vel,
                                                      std::string &message);

  bool pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot);

  /**
      * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
      * 
      * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h 
      * such that the index of the current goal pose is returned as well as 
      * the transformation between the global plan and the planning frame.
      * @param tf A reference to a tf buffer
      * @param global_plan The plan to be transformed
      * @param global_pose The global pose of the robot
      * @param costmap A reference to the costmap being used so the window size for transforming can be computed
      * @param global_frame The frame to transform the plan to
      * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
      * @param[out] transformed_plan Populated with the transformed plan
      * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
      * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
      * @return \c true if the global plan is transformed, \c false otherwise
      */
  bool transformGlobalPlan(
      const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const tf::Stamped<tf::Pose>& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, tf::StampedTransform* tf_plan_to_global);

  bool isGoalReached();

  /**
    * @brief Dummy version to satisfy MBF API
    */
  bool isGoalReached(double xy_tolerance, double yaw_tolerance) { 
    return isGoalReached(); 
  }

  const bool& isInitialized(){
    return initialized_;
  }


  bool shouldRotateToPath( const geometry_msgs::PoseStamped & carrot_pose, 
                            double & angle_to_path);

  bool shouldRotateToGoalHeading( const geometry_msgs::PoseStamped & carrot_pose);

  bool shouldReversing( const std::vector<geometry_msgs::PoseStamped>& transformed_plan_local);

  void rotateToHeading(double & linear_vel, double & angular_vel,const double & angle_to_path);

  double approachVelocityScalingFactor(const nav_msgs::Path & transformed_path,const geometry_msgs::Twist robot_cur_speed) const;

  void applyApproachVelocityScaling(const nav_msgs::Path & path,double & linear_vel,const geometry_msgs::Twist robot_cur_speed) const;

  geometry_msgs::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const std::vector<geometry_msgs::PoseStamped>& transformed_plan);
  
  double getLookAheadDistance( const geometry_msgs::Twist & speed);

  nav_msgs::Path transformGlobalPlan(const geometry_msgs::PoseStamped & pose);

  bool transformPose( const std::string frame, const geometry_msgs::PoseStamped & in_pose,
                      geometry_msgs::PoseStamped & out_pose) const;

  double costAtPose(const double & x, const double & y);

  bool inCollision( const double & x, const double & y,
                    const double & theta);

  bool isCollisionImminent(
    const geometry_msgs::PoseStamped & robot_pose,
    const double & linear_vel, const double & angular_vel,
    const geometry_msgs::Twist robot_cur_speed,
    const double robot_to_goal_dist);

  //新增
  bool frontHasObs( const geometry_msgs::PoseStamped robot_pose);
  bool frontHasObs( const geometry_msgs::PoseStamped robot_pose, const double angular_vel);
  void applyConstraints(const nav_msgs::Path & path, double & linear_vel,const geometry_msgs::Twist robot_cur_speed);


  /**
   * @brief Get the maximum extent of costmap (from costmap center along its edges to the edge)
   * 
   * @return double 
   */
  double getCostmapMaxExtent() const;

  double euclidean_distance(const geometry_msgs::PoseStamped& p1,
                            const geometry_msgs::PoseStamped& p2);

  void createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path);

  
  geometry_msgs::PointStamped createCarrotMsg( const geometry_msgs::PoseStamped & carrot_pose);

  void getRobotVel(geometry_msgs::Twist& speed);
      
  template<typename T>
  inline const T& clamp(const T& value, const T& low, const T& high) {
      return (value < low) ? low : ((value > high) ? high : value);
  }

  //设置最大速度
  bool setMaxSpeed(const double goal_max_speed, const double graph_max_speed, const double record_max_speed);

  //设置是否处于斜坡或者减速区域
  bool setRegion(const bool slope_region, const bool slow_region, const bool narrow_region);

  //设置导航类型和底盘类型 底盘轮子角度 底盘轮子速度 手绘路径和录制路径是否处于绕障中
  bool setNavType(const uint8_t nav_type,\
  const float chassis_angular,const float chassis_linear,const bool is_in_avoid_obs,const bool is_slow_speed);

  //新导航点设置
  bool setNewGoal(const geometry_msgs::PoseStamped goal_global);
  
  //设置teb路径，供选择
  bool setTebPlan(const std::vector<geometry_msgs::PoseStamped>& teb_plan);

  private:
    bool initialized_{false}; //indication of whether program has initialized

    /**
     * User-defined params
     */
    std::string odom_topic_{"odom"};

    double max_robot_pose_search_dist_;
    double global_plan_prune_distance_{1.0};


    //前视距离相关
    bool use_velocity_scaled_lookahead_dist_;
    double lookahead_time_;
    double lookahead_dist_;
    double min_lookahead_dist_, max_lookahead_dist_;

    //原地旋转相关
    bool use_rotate_to_heading_;
    double rotate_to_heading_min_angle_;
    double rotate_to_heading_angular_vel_;

    //速度相关
    bool use_regulated_linear_velocity_scaling_;
    double goal_max_speed_;
    double graph_max_speed_;
    double record_max_speed_;
    double max_seed_limit_;
    double max_angular_vel_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double approach_velocity_scaling_long_dist_;  

    //区域减速相关
    bool is_in_slope_region_;
    bool is_in_slow_region_;
    bool is_in_narrow_region_;
    double slow_speed_max_;

    //导航类型相关
    uint8_t nav_type_;

    //误差相关
    double goal_dist_tol_;
    double goal_yaw_tol_;
    double goal_ackman_dist_tol_;
    double goal_ackman_yaw_tol_;
    double goal_dist_tol_temp_;
    double goal_dist_tol_error_;
    
    //ackman底盘距离目标点距离
    double to_goal_dist_;
    
    //减速设置
    double direction_difference_sum_down_;
    double direction_difference_sum_up_;
    double direction_difference_sum_down_not_free_nav_;
    double direction_difference_sum_up_not_free_nav_;
    double direction_difference_sum_keep_time_;

    int min_global_plan_complete_size_;
    ros::Duration transform_tolerance_;
    //Control frequency
    double control_duration_,control_frequency_;

    //base_link到车前边缘的距离
    double base_link_to_car_front_dist_;

    /**
     * Pointer to other ROS Objects
     */
    tf::TransformListener *tf_;
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    base_local_planner::CostmapModel* costmap_model_; //For retrieving robot footprint cost
    base_local_planner::OdometryHelperRos odom_helper_;

    // for visualisation, publishers of global and local plan
    /**
     * @brief 
     * 
     */
    ros::Publisher global_path_pub_, local_plan_pub_;
    ros::Publisher carrot_pub_;
    ros::Publisher carrot_arc_pub_;

    bool in_slope_region_,in_slow_region_,in_narrow_region_;

    /**
     * Configs
     */
    std::string global_frame_{"map"};
    std::string robot_base_frame_{"base_link"};

    /**
     * Run-time variables
     */
    std::vector<geometry_msgs::PoseStamped> global_plan_; //Stores the current global plan
    bool goal_reached_;
    
    //全局目标点
    geometry_msgs::PoseStamped goal_global_;

    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;


    //新增
    std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot 
    double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
    double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot

    //底盘类型
    std::string chassis_type_;

    //四转四驱轮子角度和速度
    float chassis_angular_,chassis_linear_;

    //给全局路径设置方向
    void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path);
    
    //计算曲率
    inline double computeCurvature(const geometry_msgs::PoseStamped& p1,
                        const geometry_msgs::PoseStamped& p2,
                        const geometry_msgs::PoseStamped& p3);

    //手绘路径或者录制路径是否处于绕障中
    bool is_in_avoid_obs_;
    
    //偏离减速
    bool is_slow_speed_;

    //原地旋转阈值
    double self_rotate_angular_;
    
    //阿克曼减速角度阈值
    double slow_speed_angular_;
    
    //放置teb路径
    std::vector<geometry_msgs::PoseStamped> teb_plan_;

};
};

#endif //REGULATED_PURE_PURSUIT_CONTROLLER_H




