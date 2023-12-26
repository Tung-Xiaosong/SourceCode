#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


namespace pointcloud_to_laserscan
{

  PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet() {}

  //初始化函数
  void PointCloudToLaserScanNodelet::onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
    private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

    private_nh_.param<double>("angle_min", angle_min_, - M_PI);
    private_nh_.param<double>("angle_max", angle_max_, M_PI);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
    private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
    private_nh_.param<double>("range_min", range_min_, 0.0);
    private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
    private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

    private_nh_.param<double>("cut_dist", cut_dist_, 0.5);

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);

    roll_ = 0;
    pitch_ = 0;

    //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
      nh_ = getNodeHandle();
    }
    else
    {
      nh_ = getMTNodeHandle();
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
      input_queue_size_ = concurrency_level;
    }
    else
    {
      input_queue_size_ = boost::thread::hardware_concurrency();
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
      message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
      message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
    }
    else // otherwise setup direct subscription
    {
      sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    }

    set_region_sub_ = nh_.subscribe("set_region", 1, &PointCloudToLaserScanNodelet::setRegionCb, this);

    imu_sub_ = nh_.subscribe("imu_data", 1, &PointCloudToLaserScanNodelet::imuCb, this);

    in_narrow_region_sub_ = nh_.subscribe<std_msgs::Bool>("is_in_narrow_region", 1, &PointCloudToLaserScanNodelet::is_in_narrow_region_callback, this);//dxs add

    geometry_msgs::PoseArray polygon2;

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
                                                 boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                                 boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  }

  //窄道区域回调函数 dxs add
  void PointCloudToLaserScanNodelet::is_in_narrow_region_callback(const std_msgs::Bool::ConstPtr &msg) {

    //std::lock_guard<std::mutex> lock(mutex_);
    boost::mutex::scoped_lock lock(connect_mutex_);
    in_narrow_region_ = msg->data;
  }

  //斜坡区域回调函数
  void PointCloudToLaserScanNodelet::setRegionCb(const yhs_msgs::SetRegionConstPtr &region_msg)
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    
    for(int i = 0; i < region_msg->slope_area.size(); i++)
    {
      slope_region_.push_back(region_msg->slope_area[i]);
    }
    //dxs
    // for(int i = 0; i < region_msg->narrow_area.size(); i++)
    // {
    //   narrow_region_.push_back(region_msg->narrow_area[i]);
    // }
  }
  

  //IMU回调函数
  void PointCloudToLaserScanNodelet::imuCb(const sensor_msgs::ImuConstPtr& imu_msg)
  {
    boost::mutex::scoped_lock lock(connect_mutex_);

    tf2::Quaternion q;
    tf2::fromMsg(imu_msg->orientation, q);

    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

  }

  void PointCloudToLaserScanNodelet::connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
    {
      NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
      sub_.subscribe(nh_, "cloud_in", input_queue_size_);
    }
  }

  void PointCloudToLaserScanNodelet::disconnectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
      sub_.unsubscribe();
    }
  }

  void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                               tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
        << message_filter_->getTargetFramesString() << " at time " << cloud_msg->header.stamp << ", reason: " << reason);
  }

  //点云回调函数
  void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    boost::mutex::scoped_lock lock(connect_mutex_);

    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tf2_->lookupTransform("base_link", cloud_msg->header.frame_id,
                                                  ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    tf2::Quaternion q(transformStamped.transform.rotation.x,
                      transformStamped.transform.rotation.y,
                      transformStamped.transform.rotation.z,
                      transformStamped.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);


    // 将点云转换为 pcl::PointCloud 类型
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    //将点云转换为跟base_link一样的角度
    Eigen::Affine3f transform_yaw = Eigen::Affine3f::Identity();
    transform_yaw.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f transform_pitch = Eigen::Affine3f::Identity();
    transform_pitch.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));

    Eigen::Affine3f transform_init = transform_yaw * transform_pitch;

    pcl::PointCloud<pcl::PointXYZ>::Ptr toBaseLinkCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *toBaseLinkCloud, transform_init);
    

    //创建旋转矩阵  IMU
    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
    transform_y.rotate(Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY()));

    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    transform_x.rotate(Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()));

    Eigen::Affine3f transform = transform_y * transform_x;

    lock.unlock();

    // 应用旋转矩阵到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*toBaseLinkCloud, *rotatedCloud, transform);


    //还原回去
    Eigen::Affine3f transform_back_z = Eigen::Affine3f::Identity();
    transform_back_z.rotate(Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f transform_back_y = Eigen::Affine3f::Identity();
    transform_back_y.rotate(Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()));

    transform_init = transform_back_z * transform_back_y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr backCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*rotatedCloud, *backCloud, transform_init);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*backCloud, output_msg);

    output_msg.header = cloud_msg->header;

    // 创建一个指向常量的共享指针
    boost::shared_ptr<const sensor_msgs::PointCloud2> cloudPtr = boost::make_shared<const sensor_msgs::PointCloud2>(output_msg);

    // 将共享指针转换为 sensor_msgs::PointCloud2::ConstPtr 类型
    sensor_msgs::PointCloud2ConstPtr constCloudPtr = boost::const_pointer_cast<const sensor_msgs::PointCloud2>(cloudPtr);


    //build laserscan output
    sensor_msgs::LaserScan output;
    output.header = constCloudPtr->header;
    if (!target_frame_.empty())
    {
      output.header.frame_id = target_frame_;
    }

    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    // if(in_narrow_region_) output.range_min = 0.3;//dxs add//0.6
    // else output.range_min = range_min_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
      output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
      output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
    }

    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;

    // Transform cloud if necessary
    if (!(output.header.frame_id == constCloudPtr->header.frame_id))
    {
      try
      {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*constCloudPtr, *cloud, target_frame_, ros::Duration(tolerance_));
        cloud_out = cloud;
      }
      catch (tf2::TransformException &ex)
      {
        NODELET_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
      cloud_out = constCloudPtr;
    }

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
              iter_x != iter_x.end();
              iter_x += 1, iter_y += 1, iter_z += 1)
    {
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
      {
        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_)
      {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_)
      {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }
      if (range > range_max_)
      {
        NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);

      if(constCloudPtr->header.frame_id == "laser_link")
      {
        if (angle < output.angle_min || angle > output.angle_max /*angle > -1.3 && angle < 1.3*/)
        {
          NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
          continue;
        }
      }
      else
      {
        if (angle < output.angle_min || angle > output.angle_max)
        {
          NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
          continue;
        }
      }

      if (angle < output.angle_min || angle > output.angle_max /*angle > -1.3 && angle < 1.3 && cloud_msg->header.frame_id == "laser_link"*/)
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
      }

      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }

      //判断 > cut_dist_处 
      if( fabs(*iter_x) > cut_dist_)
      {

        // 转换激光点到地图坐标系
        geometry_msgs::PointStamped laser_point;
        laser_point.header.frame_id = constCloudPtr->header.frame_id;
        laser_point.header.stamp = ros::Time(0);
        laser_point.point.x = output.ranges[index] * cos(output.angle_min + index * output.angle_increment);
        laser_point.point.y = output.ranges[index] * sin(output.angle_min + index * output.angle_increment);
        laser_point.point.z = 0.0;

        geometry_msgs::PointStamped map_point;
        try {
          tf2_->transform(laser_point, map_point, "map");
        }
        catch (tf2::TransformException& ex) {
          //          ROS_ERROR("%s", ex.what());
          continue;
        }
        
        for(int i = 0; i < slope_region_.size(); i++)
        {
          // 判断点是否在多边形内部
          bool inside = pointInPolygon(map_point, slope_region_[i]);

          if(inside)
          {
            output.ranges[index] = std::numeric_limits<double>::infinity();
            break;
          }
        }
        //dxs
        // for(int i = 0; i < narrow_region_.size(); i++)
        // {
        //   // 判断点是否在多边形内部
        //   bool inside = pointInPolygon(map_point, narrow_region_[i]);

        //   if(inside)
        //   {
        //     output.ranges[index] = std::numeric_limits<double>::infinity();
        //     break;
        //   }
        // }
  
      }

    }
    pub_.publish(output);
  }

  // 判断点是否在多边形内部
  bool PointCloudToLaserScanNodelet::pointInPolygon(const geometry_msgs::PointStamped& point, const geometry_msgs::PoseArray& polygon)
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

}

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)
