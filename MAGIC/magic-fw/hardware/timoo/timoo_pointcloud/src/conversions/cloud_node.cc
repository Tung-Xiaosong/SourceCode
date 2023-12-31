/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw timoo LIDAR packets to PointCloud2.

*/

#include <ros/ros.h>
#include "timoo_pointcloud/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "timoo_cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  timoo_pointcloud::Convert conv(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
