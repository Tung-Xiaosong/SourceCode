
/*
* v1.0
*/

#include <ros_backend_interface/ros_backend_interface.h>

int main(int argc, char** argv){
    
  ros::init(argc, argv, "ros_backend_interface_node");
  tf::TransformListener tf(ros::Duration(10));

  ros_backend_interface::RosBackendInterface ros_backend_interface( tf );

  ros::MultiThreadedSpinner spinner(4);

  sleep(3);
  spinner.spin();
//  ros::spin();

  return(0);
}
