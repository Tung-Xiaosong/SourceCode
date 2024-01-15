#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 定义多边形的顶点坐标
const geometry_msgs::PoseArray polygon1 = {
  { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 1.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } }
};

const geometry_msgs::PoseArray polygon2 = {
  { { 2.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 3.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 3.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 2.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } }
};

const geometry_msgs::PoseArray polygon3 = {
  { { 0.5, 2.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 1.5, 2.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } },
  { { 1.0, 3.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } }
};

// 判断点是否在多边形内部
bool pointInPolygon(const geometry_msgs::PointStamped& point, const geometry_msgs::PoseArray& polygon)
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

// 订阅激光数据并处理
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const tf2_ros::Buffer& tfBuffer)
{
  // 获取激光雷达坐标系到地图坐标系的变换
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", msg->header.frame_id, msg->header.stamp);
  }
  catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // 遍历激光雷达数据，将每个激光点变换到地图坐标系，并判断是否在多边形内部
  for (int i = 0; i < msg->ranges.size(); i++) {
    // 转换激光点到地图坐标系
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = msg->header.frame_id;
    laser_point.header.stamp = msg->header.stamp;
    laser_point.point.x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment);
    laser_point.point.y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);
    laser_point.point.z = 0.0;

    geometry_msgs::PointStamped map_point;
    try {
      tfBuffer.transform(laser_point, map_point, "map");
    }
    catch (tf2::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      continue;
    }

    // 判断点是否在多边形内部
    bool inside = pointInPolygon(map_point, polygon1) || pointInPolygon(map_point, polygon2) || pointInPolygon(map_point, polygon3);

    // 输出处理结果
    ROS_INFO("Laser point (%f, %f) is%s in the polygons.", map_point.point.x, map_point.point.y, inside ? "" : " not");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_in_polygon");
  ros::NodeHandle nh;

  // 发布多边形的顶点坐标信息
  ros::Publisher polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("polygons", 1);
  geometry_msgs::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = "map";
  polygon_msg.header.stamp = ros::Time::now();
  polygon_msg.polygon.points.resize(12);
  for (int i = 0; i < 4; i++) {
    polygon_msg.polygon.points[i].x = polygon1.poses[i].position.x;
    polygon_msg.polygon.points[i].y = polygon1.poses[i].position.y;
  }
  for (int i = 0; i < 4; i++) {
    polygon_msg.polygon.points[i+4].x = polygon2.poses[i].position.x;
    polygon_msg.polygon.points[i+4].y = polygon2.poses[i].position.y;
  }
  for (int i = 0; i < 3; i++) {
    polygon_msg.polygon.points[i+8].x = polygon3.poses[i].position.x;
    polygon_msg.polygon.points[i+8].y = polygon3.poses[i].position.y;
  }
  polygon_pub.publish(polygon_msg);

  // 订阅激光数据并处理
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(laserCallback, _1, std::ref(tfBuffer)));

  ros::spin();

  return 0;
}
