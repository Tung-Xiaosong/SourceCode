
#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf/transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"
#include "yhs_msgs/SetRegion.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <std_msgs/Bool.h>//dxs add

namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;

  class PointCloudToLaserScanNodelet : public nodelet::Nodelet
  {

  public:
    PointCloudToLaserScanNodelet();

  private:
    virtual void onInit();

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
        tf2_ros::filter_failure_reasons::FilterFailureReason reason);

    void connectCb();

    void disconnectCb();

    void setRegionCb(const yhs_msgs::SetRegionConstPtr &region_msg);
    bool pointInPolygon(const geometry_msgs::PointStamped& point, const geometry_msgs::PoseArray& polygon);

    void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);

    void is_in_narrow_region_callback(const std_msgs::Bool::ConstPtr &msg);//dxs add

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher pub_;

    ros::Subscriber set_region_sub_,imu_sub_;

    ros::Subscriber in_narrow_region_sub_;//dxs add

    boost::mutex connect_mutex_;
    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
    boost::shared_ptr<MessageFilter> message_filter_;

    // ROS Parameters
    unsigned int input_queue_size_;
    std::string target_frame_;
    double tolerance_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
    double inf_epsilon_;
    //窄道区域判断变量
    bool in_narrow_region_;//dxs add
    //斜坡区域
    std::vector<geometry_msgs::PoseArray> slope_region_;

    //窄道dxs
    //std::vector<geometry_msgs::PoseArray> narrow_region_;

    //IMU角度
    double roll_, pitch_,yaw_;

    //斜坡裁剪距离
    double cut_dist_;
  };

}  // pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
