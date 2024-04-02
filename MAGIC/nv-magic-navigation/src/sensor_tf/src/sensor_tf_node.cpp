#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>

struct SensorParams {
  std::string name;
  std::vector<double> tf;
  std::string parent;
  std::string child;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_tf_node");
  ros::NodeHandle nh;

  // 读取参数服务器中的sensor配置
  std::vector<SensorParams> sensors;
  XmlRpc::XmlRpcValue sensor_params;
  if (nh.getParam("sensors_tf", sensor_params)) 
  {
    for (auto it = sensor_params.begin(); it != sensor_params.end(); ++it) 
    {
      SensorParams sensor;
      sensor.name = it->first;
      sensor.tf.resize(6);
      for (int i = 0; i < 6; ++i) 
      {
        if (it->second["tf"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble) 
        {
          sensor.tf[i] = static_cast<double>(it->second["tf"][i]);
        } 
        else 
        {
          ROS_WARN("Invalid tf value for sensor %s, using default value (0.0)", sensor.name.c_str());
          sensor.tf[i] = 0.0;
        }
      }
      if (it->second["parent"].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        sensor.parent = static_cast<std::string>(it->second["parent"]);
      } 
      else 
      {
        ROS_WARN("Invalid parent value for sensor %s, using default value ('world')", sensor.name.c_str());
        sensor.parent = "world";
      }
      if (it->second["child"].getType() == XmlRpc::XmlRpcValue::TypeString) 
      {
        sensor.child = static_cast<std::string>(it->second["child"]);
      } 
      else 
      {
        ROS_WARN("Invalid child value for sensor %s, using default value ('%s')", sensor.name.c_str(), sensor.name.c_str());
        sensor.child = sensor.name;
      }
      sensors.push_back(sensor);
    }
  } 
  else 
  {
    ROS_ERROR("Failed to load sensors parameters");
    return 1;
  }

  // 创建tf广播器
  static tf::TransformBroadcaster broadcaster;

  ros::Rate rate(100.0);
  while (nh.ok())
  {
    for (const auto& sensor : sensors) 
    {
      // 创建一个tf变换
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(sensor.tf[0], sensor.tf[1], sensor.tf[2]));
      tf::Quaternion q;
      q.setRPY(sensor.tf[3], sensor.tf[4], sensor.tf[5]);
      transform.setRotation(q);

      // 广播tf变换
      broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor.parent, sensor.child));
    }

    // 休眠
    rate.sleep();
  }

  return 0;
}

