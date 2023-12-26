#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // 将点云转换为 pcl::PointCloud 类型
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 创建旋转矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float theta = M_PI / 6;  // 绕 Y 轴旋转 30 度
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

    // 应用旋转矩阵到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *rotatedCloud, transform);

    // 将旋转后的点云转换回 sensor_msgs::PointCloud2 类型，并发布
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*rotatedCloud, output);
    output.header = msg->header;
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_rotation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud_topic", 10, pointCloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("rotated_point_cloud_topic", 10);

    ros::spin();

    return 0;
}











#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_node");
    ros::NodeHandle nh;

    sensor_msgs::PointCloud2 cloud;

    // 在这里给 cloud 赋值

    // 创建一个指向常量的共享指针
    boost::shared_ptr<const sensor_msgs::PointCloud2> cloudPtr = boost::make_shared<const sensor_msgs::PointCloud2>(cloud);

    // 将共享指针转换为 sensor_msgs::PointCloud2::ConstPtr 类型
    sensor_msgs::PointCloud2::ConstPtr constCloudPtr = boost::const_pointer_cast<const sensor_msgs::PointCloud2>(cloudPtr);

    // 使用 constCloudPtr 进行其他操作，如发布到 ROS 话题
    // ...

    return 0;
}
